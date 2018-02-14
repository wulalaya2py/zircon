// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/binding.h>
#include <ddk/device.h>
#include <ddk/protocol/intel-hda-codec.h>
#include <ddktl/device.h>
#include <fbl/unique_ptr.h>
#include <fbl/vmo_mapper.h>
#include <stdint.h>
#include <string.h>
#include <threads.h>

#include <intel-hda/utils/intel-hda-registers.h>

#include "debug-logging.h"
#include "utils.h"

namespace audio {
namespace intel_hda {

class IntelHDAController;

class IntelAudioDSP;
using IntelAudioDSPDeviceType = ddk::Device<IntelAudioDSP, ddk::Unbindable>;

class IntelAudioDSP : public IntelAudioDSPDeviceType {
public:
    static fbl::unique_ptr<IntelAudioDSP> Create(IntelHDAController& controller,
                                                 hda_pp_registers_t* pp_regs,
                                                 const fbl::RefPtr<RefCountedBti>& pci_bti);

    const char*  log_prefix() const { return log_prefix_; }
    const char*  dev_name() const   { return device_get_name(dev_node_); }
    zx_device_t* dev_node() const   { return dev_node_; }

    zx_status_t Initialize() __WARN_UNUSED_RESULT;
    void        ProcessIRQ();
    void        Shutdown();

    // Ddktl device interface implementation
    void DdkUnbind();
    void DdkRelease();

private:
    friend class fbl::unique_ptr<IntelAudioDSP>;

    IntelAudioDSP(IntelHDAController& controller,
                  hda_pp_registers_t* pp_regs,
                  const fbl::RefPtr<RefCountedBti>& pci_bti);
    ~IntelAudioDSP() { }

    // Accessor for our mapped registers
    adsp_registers_t* regs() const {
        return reinterpret_cast<adsp_registers_t*>(mapped_regs_.start());
    }
    hda_pp_registers_t*  pp_regs() const { return pp_regs_; }
    adsp_fw_registers_t* fw_regs() const;

    int InitThread();

    zx_status_t Boot();
    zx_status_t LoadFirmware();

    bool IsCoreEnabled(uint8_t core_mask);

    zx_status_t ResetCore(uint8_t core_mask);
    zx_status_t UnResetCore(uint8_t core_mask);
    zx_status_t PowerDownCore(uint8_t core_mask);
    zx_status_t PowerUpCore(uint8_t core_mask);
    void        RunCore(uint8_t core_mask);

    void EnableInterrupts();

    // Init thread
    thrd_t init_thread_;
    bool   initialized_ = false;

    // Log prefix storage
    char log_prefix_[LOG_PREFIX_STORAGE] = { 0 };

    // Reference to our owner.
    IntelHDAController& controller_;

    // Audio DSP device node.
    zx_device_t* dev_node_ = nullptr;

    // PCI registers
    fbl::VmoMapper      mapped_regs_;
    hda_pp_registers_t* pp_regs_ = nullptr;

    // A reference to our controller's BTI. This is needed to load firmware to the DSP.
    const fbl::RefPtr<RefCountedBti> pci_bti_;
};

}  // namespace intel_hda
}  // namespace audio
