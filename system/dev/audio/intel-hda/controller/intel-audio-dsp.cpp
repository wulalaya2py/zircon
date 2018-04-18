// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fbl/alloc_checker.h>

#include "intel-audio-dsp.h"
#include "intel-dsp-code-loader.h"
#include "intel-hda-controller.h"

namespace audio {
namespace intel_hda {

namespace {

// ADSP SRAM windows
constexpr size_t SKL_ADSP_SRAM0_OFFSET  = 0x8000; // Shared betwee Skylake and Kabylake
constexpr size_t SKL_ADSP_SRAM1_OFFSET  = 0xA000;

// Mailbox offsets
constexpr size_t ADSP_MAILBOX_IN_OFFSET = 0x1000; // Section 5.5 Offset from SRAM0
constexpr size_t ADSP_MAILBOX_IN_SIZE   = 0x1000;
constexpr size_t ADSP_MAILBOX_OUT_SIZE  = 0x1000;

constexpr const char* ADSP_FIRMWARE_PATH = "/boot/lib/firmware/dsp_fw_kbl_v3266.bin";

constexpr zx_time_t INTEL_ADSP_TIMEOUT_NSEC              = ZX_MSEC( 50); // 50mS Arbitrary
constexpr zx_time_t INTEL_ADSP_POLL_NSEC                 = ZX_USEC(500); // 500uS Arbitrary
constexpr zx_time_t INTEL_ADSP_ROM_INIT_TIMEOUT_NSEC     = ZX_SEC (  1); // 1S Arbitrary
constexpr zx_time_t INTEL_ADSP_BASE_FW_INIT_TIMEOUT_NSEC = ZX_SEC (  3); // 3S Arbitrary
constexpr zx_time_t INTEL_ADSP_POLL_FW_NSEC              = ZX_MSEC(  1); //.1mS Arbitrary
}  // anon namespace

fbl::unique_ptr<IntelAudioDSP> IntelAudioDSP::Create(IntelHDAController& controller,
                                                     hda_pp_registers_t* pp_regs,
                                                     const fbl::RefPtr<RefCountedBti>& pci_bti) {
    ZX_DEBUG_ASSERT(pp_regs != nullptr);

    fbl::AllocChecker ac;
    fbl::unique_ptr<IntelAudioDSP> ret(new (&ac) IntelAudioDSP(controller, pp_regs, pci_bti));
    if (!ac.check()) {
        GLOBAL_LOG(ERROR, "Out of memory attempting to allocate audio DSP\n");
        return nullptr;
    }

    if (ret->Initialize() != ZX_OK) {
        return nullptr;
    }

    return ret;
}

IntelAudioDSP::IntelAudioDSP(IntelHDAController& controller,
                             hda_pp_registers_t* pp_regs,
                             const fbl::RefPtr<RefCountedBti>& pci_bti)
    : IntelAudioDSPDeviceType(controller.dev_node()),
      controller_(controller),
      pp_regs_(pp_regs),
      pci_bti_(pci_bti) {

    snprintf(log_prefix_, sizeof(log_prefix_), "IHDA Audio DSP %02x:%02x.%01x",
             controller.dev_info().bus_id,
             controller.dev_info().dev_id,
             controller.dev_info().func_id);
}

adsp_fw_registers_t* IntelAudioDSP::fw_regs() const {
    return reinterpret_cast<adsp_fw_registers_t*>(static_cast<uint8_t*>(mapped_regs_.start()) +
                                                  SKL_ADSP_SRAM0_OFFSET);
}

zx_status_t IntelAudioDSP::Initialize() {
    // Fetch the bar which holds the Audio DSP registers (BAR 4), then sanity
    // check the type and size.
    zx_pci_bar_t bar_info;
    zx_status_t res = pci_get_bar(controller_.pci(), 4u, &bar_info);
    if (res != ZX_OK) {
        LOG(ERROR, "Error attempting to fetch registers from PCI (res %d)\n", res);
        return res;
    }

    if (bar_info.type != PCI_BAR_TYPE_MMIO) {
        LOG(ERROR, "Bad register window type (expected %u got %u)\n",
            PCI_BAR_TYPE_MMIO, bar_info.type);
        return ZX_ERR_INTERNAL;
    }

    // We should have a valid handle now, make sure we don't leak it.
    zx::vmo bar_vmo(bar_info.handle);
    if (bar_info.size != sizeof(adsp_registers_t)) {
        LOG(ERROR, "Bad register window size (expected 0x%zx got 0x%zx)\n",
            sizeof(adsp_registers_t), bar_info.size);
        return ZX_ERR_INTERNAL;
    }

    // Since this VMO provides access to our registers, make sure to set the
    // cache policy to UNCACHED_DEVICE
    res = bar_vmo.set_cache_policy(ZX_CACHE_POLICY_UNCACHED_DEVICE);
    if (res != ZX_OK) {
        LOG(ERROR, "Error attempting to set cache policy for PCI registers (res %d)\n", res);
        return res;
    }

    // Map the VMO in, make sure to put it in the same VMAR as the rest of our
    // registers.
    constexpr uint32_t CPU_MAP_FLAGS = ZX_VM_FLAG_PERM_READ | ZX_VM_FLAG_PERM_WRITE;
    res = mapped_regs_.Map(bar_vmo, 0, bar_info.size, CPU_MAP_FLAGS, DriverVmars::registers());
    if (res != ZX_OK) {
        LOG(ERROR, "Error attempting to map registers (res %d)\n", res);
        return res;
    }

    state_ = State::INITIALIZING;

    // Perform hardware initializastion in a thread.
    int c11_res = thrd_create(
            &init_thread_,
            [](void* ctx) -> int { return static_cast<IntelAudioDSP*>(ctx)->InitThread(); },
            this);
    if (c11_res < 0) {
        LOG(ERROR, "Failed to create init thread (res = %d)\n", c11_res);
        state_ = State::ERROR;
        return ZX_ERR_INTERNAL;
    } else {
        return ZX_OK;
    }
}

void IntelAudioDSP::ProcessIRQ() {
    // Nop if DSP is not initialized.
    if (state_ != State::OPERATING) {
        return;
    }

    uint32_t ppsts = REG_RD(&pp_regs()->ppsts);
    if (!(ppsts & HDA_PPSTS_PIS)) {
        return;
    }

    // TODO(yky) Just ack the IRQ for now.
    REG_SET_BITS(&regs()->hipct, (1u << 31));
}

void IntelAudioDSP::Shutdown() {
    if (state_ == State::INITIALIZING) {
        thrd_join(init_thread_, NULL);
    }

    PowerDownCore(ADSP_REG_ADSPCS_CORE0_MASK);

    // Disable Audio DSP and HDA interrupt
    REG_CLR_BITS<uint32_t>(&pp_regs()->ppctl, HDA_PPCTL_GPROCEN | HDA_PPCTL_PIE);

    state_ = State::SHUT_DOWN;
}

void IntelAudioDSP::DdkUnbind() {
    Shutdown();
}

void IntelAudioDSP::DdkRelease() {
}

int IntelAudioDSP::InitThread() {
    // Enable Audio DSP and HDA interrupt
    // Note: The GPROCEN bit does not really enable or disable the Audio DSP
    // operation, but mainly to work around some legacy Intel HD Audio driver
    // software such that if GPROCEN = 0, ADSPxBA (BAR2) is mapped to the Intel
    // HD Audio memory mapped configuration registers, for compliancy with some
    // legacy SW implementation. If GPROCEN = 1, only then ADSPxBA (BAR2) is
    // mapped to the actual Audio DSP memory mapped configuration registers.
    REG_SET_BITS<uint32_t>(&pp_regs()->ppctl, HDA_PPCTL_GPROCEN | HDA_PPCTL_PIE);

    // The HW loads the DSP base firmware from ROM during the initialization,
    // when the Tensilica Core is out of reset, but halted.
    zx_status_t st = Boot();
    if (st != ZX_OK) {
        LOG(ERROR, "Error in DSP boot (err %d)\n", st);
        return -1;
    }

    // Wait for ROM initialization done
    st = WaitCondition(INTEL_ADSP_ROM_INIT_TIMEOUT_NSEC,
                       INTEL_ADSP_POLL_FW_NSEC,
                       [](void* r) -> bool {
                           auto fw_regs = reinterpret_cast<adsp_fw_registers_t*>(r);
                           return ((REG_RD(&fw_regs->fw_status) & ADSP_FW_STATUS_STATE_MASK) ==
                                   ADSP_FW_STATUS_STATE_INITIALIZATION_DONE);
                       },
                       fw_regs());
    if (st != ZX_OK) {
        LOG(ERROR, "Error waiting for DSP ROM init (err %d)\n", st);
        return -1;
    }

    EnableInterrupts();

    // Load DSP Firmware
    st = LoadFirmware();
    if (st != ZX_OK) {
        LOG(ERROR, "Error loading firmware (err %d)\n", st);
        return -1;
    }

    // DSP Firmware is now ready. Add a device.
    char dev_name[ZX_DEVICE_NAME_MAX] = { 0 };
    snprintf(dev_name, sizeof(dev_name), "intel-audio-dsp-%03u", controller_.id());

    st = DdkAdd(dev_name);
    if (st != ZX_OK) {
        LOG(ERROR, "Failed to add DSP device (err %d)\n", st);
        return -1;
    }

    state_ = State::OPERATING;

    return 0;
}

zx_status_t IntelAudioDSP::Boot() {
    zx_status_t st = ZX_OK;

    // Put core into reset
    if ((st = ResetCore(ADSP_REG_ADSPCS_CORE0_MASK)) != ZX_OK) {
        LOG(ERROR, "Error attempting to enter reset on core 0 (err %d)\n", st);
        return st;
    }

    // Power down core
    if ((st = PowerDownCore(ADSP_REG_ADSPCS_CORE0_MASK)) != ZX_OK) {
        LOG(ERROR, "Error attempting to power down core 0 (err %d)\n", st);
        return st;
    }

    // Power up core
    if ((st = PowerUpCore(ADSP_REG_ADSPCS_CORE0_MASK)) != ZX_OK) {
        LOG(ERROR, "Error attempting to power up core 0 (err %d)\n", st);
        return st;
    }

    // Take core out of reset
    if ((st = UnResetCore(ADSP_REG_ADSPCS_CORE0_MASK)) != ZX_OK) {
        LOG(ERROR, "Error attempting to take core 0 out of reset (err %d)\n", st);
        return st;
    }

    // Run core
    RunCore(ADSP_REG_ADSPCS_CORE0_MASK);
    if (!IsCoreEnabled(ADSP_REG_ADSPCS_CORE0_MASK)) {
        LOG(ERROR, "Failed to start core 0\n");
        ResetCore(ADSP_REG_ADSPCS_CORE0_MASK);
        return st;
    }

    LOG(TRACE, "DSP core 0 booted!\n");
    return ZX_OK;
}

zx_status_t IntelAudioDSP::LoadFirmware() {
    IntelDSPCodeLoader loader(&regs()->cldma, pci_bti_);
    zx_status_t st = loader.Initialize();
    if (st != ZX_OK) {
        LOG(ERROR, "Error initializing firmware code loader (err %d)\n", st);
        return st;
    }

    // Get the VMO containing the firmware.
    zx::vmo fw_vmo;
    size_t fw_size;
    st = load_firmware(controller_.dev_node(), ADSP_FIRMWARE_PATH,
                       fw_vmo.reset_and_get_address(), &fw_size);
    if (st != ZX_OK) {
        LOG(ERROR, "Error fetching firmware (err %d)\n", st);
        return st;
    }

    // Transfer firmware to DSP
    st = loader.TransferFirmware(fw_vmo, fw_size);
    if (st != ZX_OK) {
        return st;
    }

    // Wait for firwmare boot
    st = WaitCondition(INTEL_ADSP_BASE_FW_INIT_TIMEOUT_NSEC,
                       INTEL_ADSP_POLL_FW_NSEC,
                       [](void* r) -> bool {
                           auto fw_regs = reinterpret_cast<adsp_fw_registers_t*>(r);
                           return ((REG_RD(&fw_regs->fw_status) & ADSP_FW_STATUS_STATE_MASK) ==
                                   ADSP_FW_STATUS_STATE_ENTER_BASE_FW);
                       },
                       fw_regs());
    if (st != ZX_OK) {
        LOG(ERROR, "Error waiting for DSP base firmware entry (err %d)\n", st);
        LOG(ERROR, "FW_STATUS  0x%08x\n", REG_RD(&fw_regs()->fw_status));
        LOG(ERROR, "ERROR_CODE 0x%08x\n", REG_RD(&fw_regs()->error_code));
        return st;
    }

    return ZX_OK;
}

bool IntelAudioDSP::IsCoreEnabled(uint8_t core_mask) {
    uint32_t val = REG_RD(&regs()->adspcs);
    bool enabled = (val & ADSP_REG_ADSPCS_CPA(core_mask)) &&
                   (val & ADSP_REG_ADSPCS_SPA(core_mask)) &&
                   !(val & ADSP_REG_ADSPCS_CSTALL(core_mask)) &&
                   !(val & ADSP_REG_ADSPCS_CRST(core_mask));
    return enabled;
}

zx_status_t IntelAudioDSP::ResetCore(uint8_t core_mask) {
    // Stall cores
    REG_SET_BITS(&regs()->adspcs, ADSP_REG_ADSPCS_CSTALL(core_mask));

    // Put cores in reset
    REG_SET_BITS(&regs()->adspcs, ADSP_REG_ADSPCS_CRST(core_mask));

    // Wait for success
    return WaitCondition(INTEL_ADSP_TIMEOUT_NSEC,
                         INTEL_ADSP_POLL_NSEC,
                         [&core_mask](void* r) -> bool {
                             auto regs = reinterpret_cast<adsp_registers_t*>(r);

                             return (REG_RD(&regs->adspcs) & ADSP_REG_ADSPCS_CRST(core_mask)) != 0;
                         },
                         regs());
}

zx_status_t IntelAudioDSP::UnResetCore(uint8_t core_mask) {
    REG_CLR_BITS(&regs()->adspcs, ADSP_REG_ADSPCS_CRST(core_mask));
    return WaitCondition(INTEL_ADSP_TIMEOUT_NSEC,
                         INTEL_ADSP_POLL_NSEC,
                         [&core_mask](void* r) -> bool {
                             auto regs = reinterpret_cast<adsp_registers_t*>(r);
                             return (REG_RD(&regs->adspcs) & ADSP_REG_ADSPCS_CRST(core_mask)) == 0;
                         },
                         regs());
}

zx_status_t IntelAudioDSP::PowerDownCore(uint8_t core_mask) {
    REG_CLR_BITS(&regs()->adspcs, ADSP_REG_ADSPCS_SPA(core_mask));
    return WaitCondition(INTEL_ADSP_TIMEOUT_NSEC,
                         INTEL_ADSP_POLL_NSEC,
                         [&core_mask](void* r) -> bool {
                             auto regs = reinterpret_cast<adsp_registers_t*>(r);
                             return (REG_RD(&regs->adspcs) & ADSP_REG_ADSPCS_SPA(core_mask)) == 0;
                         },
                         regs());
}

zx_status_t IntelAudioDSP::PowerUpCore(uint8_t core_mask) {
    REG_SET_BITS(&regs()->adspcs, ADSP_REG_ADSPCS_SPA(core_mask));
    return WaitCondition(INTEL_ADSP_TIMEOUT_NSEC,
                         INTEL_ADSP_POLL_NSEC,
                         [&core_mask](void* r) -> bool {
                             auto regs = reinterpret_cast<adsp_registers_t*>(r);
                             return (REG_RD(&regs->adspcs) & ADSP_REG_ADSPCS_SPA(core_mask)) != 0;
                         },
                         regs());
}

void IntelAudioDSP::RunCore(uint8_t core_mask) {
    REG_CLR_BITS(&regs()->adspcs, ADSP_REG_ADSPCS_CSTALL(core_mask));
}

void IntelAudioDSP::EnableInterrupts() {
    REG_SET_BITS(&regs()->adspic, ADSP_REG_ADSPIC_IPC);
    REG_SET_BITS(&regs()->hipcctl, ADSP_REG_HIPCCTL_IPCTDIE | ADSP_REG_HIPCCTL_IPCTBIE);
}

}  // namespace intel_hda
}  // namespace audio
