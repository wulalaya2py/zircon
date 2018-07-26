// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/protocol/nand.h>
#include <fbl/type_support.h>

namespace ddk {
namespace internal {

DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_map_mmio, MapMmio,
                                     void (C::*)(uint32_t, uint32_t, void**, size_t*, zx_paddr_t*,
                                                 zx_handle_t*));
DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_map_interrupt, MapInterrupt,
                                     void (C::*)(uint32_t, uint32_t, zx_handle_t*));
DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_get_bti, GetBti,
                                     void (C::*)(uint32_t, zx_handle_t*));
DECLARE_HAS_MEMBER_FN_WITH_SIGNATURE(has_get_device_info, GetDeviceInfo,
                                     void (C::*)(pdev_device_info_t*));

template <typename D>
constexpr void CheckNandProtocolSubclass() {
    static_assert(internal::has_map_mmio<D>::value,
                  "NandProtocol subclasses must implement "
                  "MapMmio(uint32_t index, uint32_t cache_policy, void** out_vaddr, "
                  "size_t* out_size, zx_paddr_t* out_paddr, zx_handle_t* out_handle)");
    static_assert(internal::has_map_interrupt<D>::value,
                  "NandProtocol subclasses must implement "
                  "MapInterrupt(uint32_t index, uint32_t flags, zx_handle_t* out_handle)");
    static_assert(internal::has_get_bti<D>::value,
                  "NandProtocol subclasses must implement "
                  "GetBti(uint32_t index, zx_handle_t* out_handle)");
    static_assert(internal::has_get_device_info<D>::value,
                  "NandProtocol subclasses must implement "
                  "GetDeviceInfo(pdev_device_info_t* out_info)");
 }

}  // namespace internal
}  // namespace ddk
