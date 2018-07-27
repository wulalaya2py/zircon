// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddktl/device.h>
#include <ddktl/protocol/platform-device.h>
#include <lib/zx/handle.h>

namespace platform_bus {

class ProxyDevice;
using ProxyDeviceType = ddk::Device<ProxyDevice>;

class ProxyDevice : public ProxyDeviceType, public ddk::PdevProtocol<ProxyDevice>  {
public:
    explicit ProxyDevice(zx_device_t* parent, zx_handle_t rpc_channel)
        : ProxyDeviceType(parent), rpc_channel_(rpc_channel) {}

    zx_status_t Create(const char* name);

    void DdkRelease();

    // platform device protocol implementation
    zx_status_t MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, size_t* out_size,
                        zx_paddr_t* out_paddr, zx_handle_t* out_handle);
    zx_status_t MapInterrupt(uint32_t index, uint32_t flags, zx_handle_t* out_handle);
    zx_status_t GetBti(uint32_t index, zx_handle_t* out_handle);
    zx_status_t GetDeviceInfo(pdev_device_info_t* out_info);

private:
    DISALLOW_COPY_ASSIGN_AND_MOVE(ProxyDevice);

    zx::handle rpc_channel_;
};

} // namespace platform_bus

__BEGIN_CDECLS
zx_status_t platform_proxy_create(void* ctx, zx_device_t* parent, const char* name,
                                  const char* args, zx_handle_t rpc_channel);
__END_CDECLS
