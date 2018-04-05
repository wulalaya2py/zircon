// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/binding.h>
#include <ddk/device.h>
#include <ddk/protocol/intel-hda-codec.h>
#include <ddktl/device.h>
#include <fbl/mutex.h>
#include <fbl/unique_ptr.h>
#include <fbl/vmo_mapper.h>

#include <sync/completion.h>
#include <stdint.h>
#include <string.h>
#include <threads.h>

#include <intel-hda/utils/intel-audio-dsp-ipc.h>
#include <intel-hda/utils/intel-hda-registers.h>

#include "debug-logging.h"
#include "thread-annotations.h"
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

    enum class State : uint8_t {
        START,
        INITIALIZING,  // init thread running
        OPERATING,
        SHUT_DOWN,
        ERROR = 0xFF,
    };
    State state_ = State::START;

    //
    // IPC
    //

    struct IpcTxn {
        IpcTxn(uint32_t pri, uint32_t ext, const void* tx, size_t txs, void* rx, size_t rxs)
            : request(pri, ext), tx_data(tx), tx_size(txs), rx_data(rx), rx_size(rxs) { }

        IpcMessage request;
        IpcMessage reply;

        const void* tx_data = nullptr;
        size_t      tx_size = 0;
        void*       rx_data = nullptr;
        size_t      rx_size = 0;
        size_t      rx_actual = 0;

        completion_t completion;
    };

    // Send an IPC message
    void SendIpc(IpcTxn* txn);

    // Send an IPC message and wait for response
    void SendIpcWait(IpcTxn* txn);

    // Library & Module Management IPC
    void SendInitInstanceIpc(uint16_t module_id, uint8_t instance_id,
                             ProcDomain proc_domain, uint8_t core_id, uint8_t ppl_instance_id,
                             uint16_t param_block_size, const void* param_data);
    void SendBindIpc(uint16_t src_module_id, uint8_t src_instance_id, uint8_t src_queue,
                     uint16_t dst_module_id, uint8_t dst_instance_id, uint8_t dst_queue);

    // Pipeline Management IPC
    void SendCreatePipelineIpc(uint8_t instance_id, uint8_t ppl_priority,
                               uint16_t ppl_mem_size, bool lp);
    void SendSetPipelineStateIpc(uint8_t ppl_id, PipelineState state, bool sync_stop_start);

    // Base FW IPC
    void SendGetFirmwareConfigIpc();
    void SendGetHardwareConfigIpc();
    void SendGetModulesInfoIpc();
    void SendGetPipelineListInfoIpc();
    void SendGetPipelinePropsIpc();

    // Process a reply from DSP
    void ProcessIpcReply(const IpcMessage& reply);
    void ProcessLargeConfigGetReply(IpcTxn* txn);

    void DumpFirmwareConfig(const TLVHeader* config, size_t length);
    void DumpHardwareConfig(const TLVHeader* config, size_t length);
    void DumpModulesInfo(const ModuleEntry* info, uint32_t count);
    void DumpPipelineListInfo(const PipelineListInfo* info);
    void DumpPipelineProps(const PipelineProps* props);

    // Pending IPC
    fbl::Mutex ipc_lock_;
    IpcTxn* pending_txn_ TA_GUARDED(ipc_lock_) = nullptr;

    // IPC Mailboxes
    class Mailbox {
    public:
        void Initialize(void* base, size_t size) {
            base_ = base;
            size_ = size;
        }

        size_t size() const { return size_; }

        void Write(const void* data, size_t size) {
            // It is the caller's responsibility to ensure size fits in the mailbox.
            ZX_DEBUG_ASSERT(size <= size_);
            memcpy(base_, data, size);
        }
        void Read(void* data, size_t size) {
            // It is the caller's responsibility to ensure size fits in the mailbox.
            ZX_DEBUG_ASSERT(size <= size_);
            memcpy(data, base_, size);
        }
    private:
        void*  base_;
        size_t size_;
    };
    Mailbox mailbox_in_ TA_GUARDED(ipc_lock_);
    Mailbox mailbox_out_ TA_GUARDED(ipc_lock_);

    // Cached module ids
    uint16_t copier_module_id_ = 0;
    uint16_t mixout_module_id_ = 0;

    // Init thread
    thrd_t init_thread_;

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
