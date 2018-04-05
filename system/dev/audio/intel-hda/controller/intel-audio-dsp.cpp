// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fbl/alloc_checker.h>
#include <fbl/auto_call.h>
#include <fbl/auto_lock.h>
#include <string.h>

#include <pretty/hexdump.h>

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
constexpr const char* I2S_CFG_PATH       = "/boot/lib/firmware/max98927-render-2ch-48khz-24b.bin";

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

    // Initialize mailboxes
    uint8_t* mapped_base = static_cast<uint8_t*>(mapped_regs_.start());
    mailbox_in_.Initialize(static_cast<void*>(mapped_base + SKL_ADSP_SRAM0_OFFSET +
                                              ADSP_MAILBOX_IN_OFFSET),
                           ADSP_MAILBOX_IN_SIZE);
    mailbox_out_.Initialize(static_cast<void*>(mapped_base + SKL_ADSP_SRAM1_OFFSET),
                            ADSP_MAILBOX_OUT_SIZE);

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

    IpcMessage message(REG_RD(&regs()->hipct), REG_RD(&regs()->hipcte));
    if (message.primary & ADSP_REG_HIPCT_BUSY) {
        // Ack the IRQ.
        REG_SET_BITS(&regs()->hipct, ADSP_REG_HIPCT_BUSY);

        if (message.is_notif()) {
            LOG(INFO, "got notification type %u\n", message.notif_type());
        } else if (message.is_reply()) {
            ProcessIpcReply(message);
        }
    }
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

    SendGetModulesInfoIpc();

    SendGetFirmwareConfigIpc();

    SendGetHardwareConfigIpc();

    ZX_DEBUG_ASSERT(mixout_module_id_ != 0);
    ZX_DEBUG_ASSERT(copier_module_id_ != 0);

    // Create a pipeline with mixout and copier modules to output DMA gateway
    uint8_t PIPELINE_ID = 0;
    SendCreatePipelineIpc(PIPELINE_ID, 0, 6, true);

    uint8_t MIXOUT_ID = 1;
    uint8_t COPIER_ID = 2;

    // Create the mixout module
    const BaseModuleCfg mixout_cfg = {
        .cpc = 100000,
        .ibs = 384,
        .obs = 384,
        .is_pages = 0,
        .audio_fmt = {
            .sampling_frequency = SamplingFrequency::FS_48000HZ,
            .bit_depth = BitDepth::DEPTH_32BIT,
            .channel_map = 0xFFFFFF10,
            .channel_config = ChannelConfig::CONFIG_STEREO,
            .interleaving_style = InterleavingStyle::PER_CHANNEL,
            .number_of_channels = 2,
            .valid_bit_depth = 32,
            .sample_type = SampleType::INT_MSB,
            .reserved = 0,
        },
    };
    SendInitInstanceIpc(mixout_module_id_, MIXOUT_ID, ProcDomain::LOW_LATENCY, 0,
                        PIPELINE_ID, static_cast<uint16_t>(sizeof(mixout_cfg)),
                        static_cast<const void*>(&mixout_cfg));

    // Get the VMO containing the I2S config blob
    // TODO(yky): this should come from ACPI (NHLT table)
    zx::vmo blob_vmo;
    size_t blob_size;
    st = load_firmware(controller_.dev_node(), I2S_CFG_PATH,
                       blob_vmo.reset_and_get_address(), &blob_size);
    if (st != ZX_OK) {
        LOG(ERROR, "Error getting I2S config blob (err %d)\n", st);
        return st;
    }

    CopierCfg copier_cfg = {
        .base_cfg = {
            .cpc = 100000,
            .ibs = 384,
            .obs = 384,
            .is_pages = 0,
            .audio_fmt = {
                .sampling_frequency = SamplingFrequency::FS_48000HZ,
                .bit_depth = BitDepth::DEPTH_32BIT,
                .channel_map = 0xFFFFFF10,
                .channel_config = ChannelConfig::CONFIG_STEREO,
                .interleaving_style = InterleavingStyle::PER_CHANNEL,
                .number_of_channels = 2,
                .valid_bit_depth = 24,
                .sample_type = SampleType::INT_MSB,
                .reserved = 0,
            },
        },
        .out_fmt = {
            .sampling_frequency = SamplingFrequency::FS_48000HZ,
            .bit_depth = BitDepth::DEPTH_32BIT,
            .channel_map = 0xFFFFFF10,
            .channel_config = ChannelConfig::CONFIG_STEREO,
            .interleaving_style = InterleavingStyle::PER_CHANNEL,
            .number_of_channels = 2,
            .valid_bit_depth = 24,
            .sample_type = SampleType::INT_MSB,
            .reserved = 0,
        },
        .copier_feature_mask = 0,
        .gtw_cfg = {
            .node_id = I2S_GATEWAY_CFG_NODE_ID(DMA_TYPE_I2S_LINK_INPUT, 0, 0),
            .dma_buffer_size = 2 * 384,
            .config_length = static_cast<uint32_t>(blob_size),
        },
    };

    size_t copier_cfg_size = sizeof(copier_cfg) + blob_size;
    ZX_DEBUG_ASSERT(copier_cfg_size <= UINT16_MAX);
    uint8_t* copier_cfg_buf = static_cast<uint8_t*>(malloc(copier_cfg_size));
    auto copier_cleanup = fbl::MakeAutoCall([&copier_cfg_buf] {
                                                if (copier_cfg_buf) {
                                                    free(copier_cfg_buf);
                                                }
                                            });
    if (copier_cfg_buf == nullptr) {
        LOG(ERROR, "out of memory while attempting to allocate copier config buffer\n");
        return ZX_ERR_NO_MEMORY;
    }
    memcpy(copier_cfg_buf, &copier_cfg, sizeof(copier_cfg));
    st = blob_vmo.read(copier_cfg_buf + sizeof(copier_cfg), 0, blob_size);
    if (st != ZX_OK) {
        LOG(ERROR, "Error reading I2S config blob VMO (err %d)\n", st);
        return st;
    }

    // Create the copier module
    SendInitInstanceIpc(copier_module_id_, COPIER_ID, ProcDomain::LOW_LATENCY, 0,
                        PIPELINE_ID, static_cast<uint16_t>(copier_cfg_size), copier_cfg_buf);

    // Bind mixout pin 0 to copier pin 0
    SendBindIpc(mixout_module_id_, MIXOUT_ID, 0, copier_module_id_, COPIER_ID, 0);

    SendSetPipelineStateIpc(PIPELINE_ID, PipelineState::STARTED, true);

    SendGetPipelinePropsIpc();

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

void IntelAudioDSP::ProcessIpcReply(const IpcMessage& reply) {
    fbl::AutoLock ipc_lock(&ipc_lock_);
    if (pending_txn_ == nullptr) {
        LOG(INFO, "got spurious reply message\n");
        return;
    }

    // Check if the reply matches the pending request.
    IpcMessage* pending = &pending_txn_->request;
    if ((pending->msg_tgt() != reply.msg_tgt()) || (pending->type() != reply.type())) {
        LOG(INFO, "reply msg mismatch, got pri 0x%08x ext 0x%08x, expect pri 0x%08x ext 0x%08x\n",
            reply.primary, reply.extension, pending->primary, pending->extension);
        return;
    }

    LOG(INFO, "got reply (status %u) for pending msg, pri 0x%08x ext 0x%08x\n",
              to_underlying(reply.status()), reply.primary, reply.extension);

    pending_txn_->reply = reply;

    if (reply.msg_tgt() == MsgTarget::MODULE_MSG) {
        ModuleMsgType type = static_cast<ModuleMsgType>(reply.type());
        switch (type) {
        case ModuleMsgType::LARGE_CONFIG_GET:
            ProcessLargeConfigGetReply(pending_txn_);
            break;
        default:
            break;
        }
    }

    completion_signal(&pending_txn_->completion);
    pending_txn_ = nullptr;
}

void IntelAudioDSP::ProcessLargeConfigGetReply(IpcTxn* txn) {
    ZX_DEBUG_ASSERT(txn->request.instance_id() == txn->reply.instance_id());
    ZX_DEBUG_ASSERT(txn->request.module_id() == txn->reply.module_id());

    ZX_DEBUG_ASSERT_MSG(txn->request.large_param_id() == txn->reply.large_param_id(),
                        "large_param_id mismatch, expected %u got %u\n",
                        txn->request.large_param_id(), txn->reply.large_param_id());

    LOG(INFO, "got LARGE_CONFIG_GET reply, id %u init_block %d final_block %d data_off_size %u\n",
        txn->reply.large_param_id(), txn->reply.init_block(), txn->reply.final_block(),
        txn->reply.data_off_size());

    // Only support single reads for now.
    uint32_t size = txn->reply.data_off_size();
    ZX_DEBUG_ASSERT(txn->reply.init_block());
    ZX_DEBUG_ASSERT(txn->reply.final_block());
    ZX_DEBUG_ASSERT(size > 0);
    ZX_DEBUG_ASSERT(size <= txn->rx_size);

    mailbox_in_.Read(txn->rx_data, size);
    txn->rx_actual = size;
}

void IntelAudioDSP::DumpFirmwareConfig(const TLVHeader* config, size_t length) {
    LOG(INFO, "===== Firmware Config =====\n");
    size_t bytes = 0;
    while (bytes < length) {
        if (length - bytes <= sizeof(*config)) {
            LOG(ERROR, "Got short firmware config TLV entry\n");
            break;
        }

        auto ptr = reinterpret_cast<const uint8_t*>(config);
        auto cfg = reinterpret_cast<const TLVHeader*>(ptr + bytes);
        auto type = static_cast<FirmwareConfigType>(cfg->type);
        switch (type) {
        case FirmwareConfigType::FW_VERSION: {
            auto version = reinterpret_cast<const uint16_t*>(cfg->data);
            LOG(INFO, "                fw_version: %u.%u hotfix %u (build %u)\n",
                      version[0], version[1], version[2], version[3]);
            break;
        }
        case FirmwareConfigType::MEMORY_RECLAIMED: {
            auto memory = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "          memory_reclaimed: %u\n", *memory);
            break;
        }
        case FirmwareConfigType::SLOW_CLOCK_FREQ_HZ: {
            auto freq = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "                  osc_freq: %u\n", *freq);
            break;
        }
        case FirmwareConfigType::FAST_CLOCK_FREQ_HZ: {
            auto freq = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "                  pll_freq: %u\n", *freq);
            break;
        }
        case FirmwareConfigType::DMA_BUFFER_CONFIG: {
            auto buf_cfg = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "             dma_buf_count: %u\n", cfg->length / 8);
            for (uint32_t i = 0; i < cfg->length / 8; i++) {
                LOG(INFO, "          dma_min_size[%02u]: %u\n", i, buf_cfg[i * 2]);
                LOG(INFO, "          dma_max_size[%02u]: %u\n", i, buf_cfg[(i * 2) + 1]);
            }
            break;
        }
        case FirmwareConfigType::ALH_SUPPORT_LEVEL: {
            auto level = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "         alh_support_level: %u\n", *level);
            break;
        }
        case FirmwareConfigType::IPC_DL_MAILBOX_BYTES: {
            auto bytes = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "           mailbox_in_size: %u\n", *bytes);
            break;
        }
        case FirmwareConfigType::IPC_UL_MAILBOX_BYTES: {
            auto bytes = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "          mailbox_out_size: %u\n", *bytes);
            break;
        }
        case FirmwareConfigType::TRACE_LOG_BYTES: {
            auto bytes = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "            trace_log_size: %u\n", *bytes);
            break;
        }
        case FirmwareConfigType::MAX_PPL_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "             max_ppl_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::MAX_ASTATE_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "          max_astate_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::MAX_MODULE_PIN_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "      max_module_pin_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::MODULES_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "             modules_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::MAX_MOD_INST_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "        max_mod_inst_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::MAX_LL_TASKS_PER_PRI_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "max_ll_tasks_per_pri_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::LL_PRI_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "              ll_pri_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::MAX_DP_TASKS_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "        max_dp_tasks_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::MAX_LIBS_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "            max_libs_count: %u\n", *count);
            break;
        }
        case FirmwareConfigType::SCHEDULER_CONFIG: {
            // Skip dumping this one
            break;
        }
        case FirmwareConfigType::XTAL_FREQ_HZ: {
            auto freq = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "              xtal_freq_hz: %u\n", *freq);
            break;
        }
        default:
            LOG(ERROR, "Unknown firmware config type %u\n", cfg->type);
            break;
        }
        bytes += sizeof(*cfg) + cfg->length;
    }
}

void IntelAudioDSP::DumpHardwareConfig(const TLVHeader* config, size_t length) {
    LOG(INFO, "===== Hardware Config =====\n");
    size_t bytes = 0;
    while (bytes < length) {
        if (length - bytes <= sizeof(*config)) {
            LOG(ERROR, "Got short hardware config TLV entry\n");
            break;
        }

        auto ptr = reinterpret_cast<const uint8_t*>(config);
        auto cfg = reinterpret_cast<const TLVHeader*>(ptr + bytes);
        auto type = static_cast<HardwareConfigType>(cfg->type);
        switch (type) {
        case HardwareConfigType::CAVS_VERSION: {
            auto version = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "        cavs_version: 0x%08x\n", *version);
            break;
        }
        case HardwareConfigType::DSP_CORES: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "           dsp_cores: %u\n", *count);
            break;
        }
        case HardwareConfigType::MEM_PAGE_BYTES: {
            auto bytes = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "      mem_page_bytes: %u\n", *bytes);
            break;
        }
        case HardwareConfigType::TOTAL_PHYS_MEM_PAGES: {
            auto pages = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "total_phys_mem_pages: %u\n", *pages);
            break;
        }
        case HardwareConfigType::I2S_CAPS: {
            // Skip dumping this one
            break;
        }
        case HardwareConfigType::GPDMA_CAPS: {
            // Skip dumping this one
            break;
        }
        case HardwareConfigType::GATEWAY_COUNT: {
            auto count = reinterpret_cast<const uint32_t*>(cfg->data);
            LOG(INFO, "       gateway_count: %u\n", *count);
            break;
        }
        case HardwareConfigType::HP_EBB_COUNT: {
            // Skip dumping this one
            break;
        }
        case HardwareConfigType::LP_EBB_COUNT: {
            // Skip dumping this one
            break;
        }
        case HardwareConfigType::EBB_SIZE_BYTES: {
            // Skip dumping this one
            break;
        }
        default:
            LOG(ERROR, "Unknown hardware config type %u\n", cfg->type);
            break;
        }
        bytes += sizeof(*cfg) + cfg->length;
    }
}

void IntelAudioDSP::DumpModulesInfo(const ModuleEntry* info, uint32_t count) {
    LOG(INFO, "num modules: %u\n", count);
    for (uint32_t i = 0; i < count; i++) {
        LOG(INFO, "[%02u]:\n", i);
        LOG(INFO, "  module_id: %u\n", info[i].module_id);
        LOG(INFO, "  state_flags: 0x%04x\n", info[i].state_flags);
        char name[9] = { 0 };
        strncpy(name, reinterpret_cast<const char*>(info[i].name), sizeof(name-1));
        LOG(INFO, "         name: %s\n", name);
        LOG(INFO, "         uuid: %08X-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X\n",
                  info[i].uuid[0],
                  info[i].uuid[1] & 0xFFFF, info[i].uuid[1] >> 16,
                  info[i].uuid[2] & 0xFF,
                  (info[i].uuid[2] >> 8) & 0xFF,
                  (info[i].uuid[2] >> 16) & 0xFF,
                  (info[i].uuid[2] >> 24) & 0xFF,
                  info[i].uuid[3] & 0xFF,
                  (info[i].uuid[3] >> 8) & 0xFF,
                  (info[i].uuid[3] >> 16) & 0xFF,
                  (info[i].uuid[3] >> 24) & 0xFF);
    }
}

void IntelAudioDSP::DumpPipelineListInfo(const PipelineListInfo* info) {
    LOG(INFO, "num pipelines: %u\n", info->ppl_count);
    for (uint32_t i = 0; i < info->ppl_count; i++) {
        LOG(INFO, "[%02u]: id %u\n", i, info->ppl_id[i]);
    }
}

void IntelAudioDSP::DumpPipelineProps(const PipelineProps* props) {
    LOG(INFO, "                   id: %u\n", props->id);
    LOG(INFO, "             priority: %u\n", props->priority);
    LOG(INFO, "                state: %u\n", props->state);
    LOG(INFO, "   total_memory_bytes: %u\n", props->total_memory_bytes);
    LOG(INFO, "    used_memory_bytes: %u\n", props->used_memory_bytes);
    LOG(INFO, "        context_pages: %u\n", props->context_pages);
    LOG(INFO, "module_instance_count: %u\n", props->module_instances.module_instance_count);
    for (uint32_t i = 0; i < props->module_instances.module_instance_count; i++) {
        LOG(INFO, " module_instance[%1u]: id 0x%08x\n",
                  i, props->module_instances.module_instance_id[i]);
    }
}

void IntelAudioDSP::SendIpc(IpcTxn* txn) {
    fbl::AutoLock ipc_lock(&ipc_lock_);
    // 1 at a time
    ZX_DEBUG_ASSERT(pending_txn_ == nullptr);

    // Copy tx data to outbox
    if (txn->tx_size > 0) {
        mailbox_out_.Write(txn->tx_data, txn->tx_size);
    }
    // HIPCIE must be programmed before setting HIPCI.BUSY
    REG_WR(&regs()->hipcie, txn->request.extension);
    REG_WR(&regs()->hipci, txn->request.primary | ADSP_REG_HIPCI_BUSY);

    // Wait for completion
    pending_txn_ = txn;
}

void IntelAudioDSP::SendIpcWait(IpcTxn* txn) {
    SendIpc(txn);
    completion_wait(&txn->completion, ZX_TIME_INFINITE);
}

void IntelAudioDSP::SendInitInstanceIpc(uint16_t module_id, uint8_t instance_id,
                                        ProcDomain proc_domain, uint8_t core_id,
                                        uint8_t ppl_instance_id,
                                        uint16_t param_block_size, const void* param_data) {
    LOG(DEBUG1, "INIT_INSTANCE (mod %u inst %u)\n", module_id, instance_id);

    IpcTxn txn(IPC_PRI(MsgTarget::MODULE_MSG, MsgDir::MSG_REQUEST,
                           ModuleMsgType::INIT_INSTANCE, instance_id, module_id),
               IPC_INIT_INSTANCE_EXT(proc_domain, core_id, ppl_instance_id, param_block_size),
               param_data, param_block_size, nullptr, 0);

    SendIpcWait(&txn);

    if (txn.reply.status() != MsgStatus::IPC_SUCCESS) {
        LOG(ERROR, "INIT_INSTANCE (mod %u inst %u) failed (err %d)\n",
                   module_id, instance_id, to_underlying(txn.reply.status()));
    } else {
        LOG(DEBUG1, "INIT_INSTANCE (mod %u inst %u) success\n", module_id, instance_id);
    }
}

void IntelAudioDSP::SendBindIpc(uint16_t src_module_id, uint8_t src_instance_id, uint8_t src_queue,
                                uint16_t dst_module_id, uint8_t dst_instance_id, uint8_t dst_queue) {
    LOG(DEBUG1, "BIND (mod %u inst %u -> mod %u inst %u)\n",
                src_module_id, src_instance_id, dst_module_id, dst_instance_id);

    IpcTxn txn(IPC_PRI(MsgTarget::MODULE_MSG, MsgDir::MSG_REQUEST, ModuleMsgType::BIND,
                       src_instance_id, src_module_id),
               IPC_BIND_UNBIND_EXT(dst_module_id, dst_instance_id, dst_queue, src_queue),
               nullptr, 0, nullptr, 0);
    SendIpcWait(&txn);

    if (txn.reply.status() != MsgStatus::IPC_SUCCESS) {
        LOG(ERROR, "BIND (mod %u inst %u -> mod %u inst %u) failed (err %d)\n",
                    src_module_id, src_instance_id, dst_module_id, dst_instance_id,
                    to_underlying(txn.reply.status()));
    } else {
        LOG(DEBUG1, "BIND (mod %u inst %u -> mod %u inst %u) success\n",
                    src_module_id, src_instance_id, dst_module_id, dst_instance_id);
    }
}

void IntelAudioDSP::SendCreatePipelineIpc(uint8_t instance_id,
                                          uint8_t ppl_priority, uint16_t ppl_mem_size,
                                          bool lp) {
    LOG(DEBUG1, "CREATE_PIPELINE (inst %u)\n", instance_id);

    IpcTxn txn(IPC_CREATE_PIPELINE_PRI(instance_id, ppl_priority, ppl_mem_size),
               IPC_CREATE_PIPELINE_EXT(lp),
               nullptr, 0, nullptr, 0);
    SendIpcWait(&txn);

    if (txn.reply.status() != MsgStatus::IPC_SUCCESS) {
        LOG(ERROR, "CREATE_PIPELINE (inst %u) failed (err %d)\n",
                   instance_id, to_underlying(txn.reply.status()));
    } else {
        LOG(DEBUG1, "CREATE_PIPELINE (inst %u) success\n", instance_id);
    }
}

void IntelAudioDSP::SendSetPipelineStateIpc(uint8_t ppl_id, PipelineState state,
                                            bool sync_stop_start) {
    LOG(DEBUG1, "SET_PIPELINE_STATE (inst %u)\n", ppl_id);

    IpcTxn txn(IPC_SET_PIPELINE_STATE_PRI(ppl_id, state),
               IPC_SET_PIPELINE_STATE_EXT(false, sync_stop_start),
               nullptr, 0, nullptr, 0);
    SendIpcWait(&txn);

    if (txn.reply.status() != MsgStatus::IPC_SUCCESS) {
        LOG(ERROR, "SET_PIPELINE_STATE (inst %u) failed (err %d)\n",
                   ppl_id, to_underlying(txn.reply.status()));
    } else {
        LOG(DEBUG1, "SET_PIPELINE_STATE (inst %u) success\n", ppl_id);
    }
}

void IntelAudioDSP::SendGetFirmwareConfigIpc() {
    uint8_t data[ADSP_MAILBOX_IN_SIZE];
    IpcTxn txn(IPC_PRI(MsgTarget::MODULE_MSG, MsgDir::MSG_REQUEST,
                       ModuleMsgType::LARGE_CONFIG_GET, 0, 0),
               IPC_LARGE_CONFIG_EXT(true, true, to_underlying(BaseFWParamType::FIRMWARE_CONFIG),
                                    sizeof(data)),
               nullptr, 0, data, sizeof(data));
    SendIpcWait(&txn);

    DumpFirmwareConfig(reinterpret_cast<const TLVHeader*>(txn.rx_data), txn.rx_actual);
}

void IntelAudioDSP::SendGetHardwareConfigIpc() {
    uint8_t data[ADSP_MAILBOX_IN_SIZE];
    IpcTxn txn(IPC_PRI(MsgTarget::MODULE_MSG, MsgDir::MSG_REQUEST,
                       ModuleMsgType::LARGE_CONFIG_GET, 0, 0),
               IPC_LARGE_CONFIG_EXT(true, true, to_underlying(BaseFWParamType::HARDWARE_CONFIG),
                                    sizeof(data)),
               nullptr, 0, data, sizeof(data));
    SendIpcWait(&txn);

    DumpHardwareConfig(reinterpret_cast<const TLVHeader*>(txn.rx_data), txn.rx_actual);
}

void IntelAudioDSP::SendGetModulesInfoIpc() {
    uint8_t data[ADSP_MAILBOX_IN_SIZE];
    IpcTxn txn(IPC_PRI(MsgTarget::MODULE_MSG, MsgDir::MSG_REQUEST,
                       ModuleMsgType::LARGE_CONFIG_GET, 0, 0),
               IPC_LARGE_CONFIG_EXT(true, true, to_underlying(BaseFWParamType::MODULES_INFO),
                                    sizeof(data)),
               nullptr, 0, data, sizeof(data));
    SendIpcWait(&txn);

    const ModulesInfo* info = reinterpret_cast<const ModulesInfo*>(txn.rx_data);
    uint32_t count = info->module_count;

    ZX_DEBUG_ASSERT(txn.rx_actual >= (sizeof(ModulesInfo) + (count * sizeof(ModuleEntry))));

    DumpModulesInfo(info->module_info, count);

    for (uint32_t i = 0; i < count; i++) {
        if (!strncmp(reinterpret_cast<const char*>(info->module_info[i].name),
                     "COPIER", sizeof("COPIER"))) {
            copier_module_id_ = info->module_info[i].module_id;
        } else if (!strncmp(reinterpret_cast<const char*>(info->module_info[i].name),
                            "MIXOUT", sizeof("MIXOUT"))) {
            mixout_module_id_ = info->module_info[i].module_id;
        }
    }

}

void IntelAudioDSP::SendGetPipelineListInfoIpc() {
    LOG(DEBUG1, "LARGE_CONFIG_GET (pipeline list info)\n");

    uint8_t data[ADSP_MAILBOX_IN_SIZE];
    IpcTxn txn(IPC_PRI(MsgTarget::MODULE_MSG, MsgDir::MSG_REQUEST,
                       ModuleMsgType::LARGE_CONFIG_GET, 0, 0),
               IPC_LARGE_CONFIG_EXT(true, true, to_underlying(BaseFWParamType::PIPELINE_LIST_INFO),
                                    sizeof(data)),
               nullptr, 0, data, sizeof(data));
    SendIpcWait(&txn);

    DumpPipelineListInfo(reinterpret_cast<const PipelineListInfo*>(txn.rx_data));
}

void IntelAudioDSP::SendGetPipelinePropsIpc() {
    LOG(DEBUG1, "LARGE_CONFIG_GET (pipeline props)\n");

    uint32_t param = static_cast<uint32_t>(BaseFWParamType::PIPELINE_PROPS);
    uint8_t data[ADSP_MAILBOX_IN_SIZE];
    IpcTxn txn(IPC_PRI(MsgTarget::MODULE_MSG, MsgDir::MSG_REQUEST,
                       ModuleMsgType::LARGE_CONFIG_GET, 0, 0),
               IPC_LARGE_CONFIG_EXT(true, true, to_underlying(BaseFWParamType::PIPELINE_PROPS),
                                    sizeof(data)),
               &param, sizeof(param), data, sizeof(data));
    SendIpcWait(&txn);

    DumpPipelineProps(static_cast<PipelineProps*>(txn.rx_data));
}

}  // namespace intel_hda
}  // namespace audio
