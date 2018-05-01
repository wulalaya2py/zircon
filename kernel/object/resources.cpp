// Copyright 2017 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <object/resources.h>

#include <fbl/ref_ptr.h>
#include <object/process_dispatcher.h>
#include <object/resource_dispatcher.h>
#include <zircon/syscalls/resource.h>

static constexpr zx_status_t check_kind(uint32_t res_kind, uint32_t req_kind) {
    if (res_kind == req_kind || res_kind == ZX_RSRC_KIND_ROOT) {
        return ZX_OK;
    }

    return ZX_ERR_ACCESS_DENIED;
}

zx_status_t validate_resource(zx_handle_t handle, uint32_t kind) {
    auto up = ProcessDispatcher::GetCurrent();
    fbl::RefPtr<ResourceDispatcher> resource;
    auto status = up->GetDispatcher(handle, &resource);
    if (status != ZX_OK) {
        return status;
    }

    return check_kind(resource->get_kind(), kind);
}

zx_status_t validate_ranged_resource(zx_handle_t handle,
                                     uint32_t kind,
                                     uint64_t base,
                                     uint64_t len) {
    auto up = ProcessDispatcher::GetCurrent();
    fbl::RefPtr<ResourceDispatcher> resource;
    auto status = up->GetDispatcher(handle, &resource);
    if (status != ZX_OK) {
        return status;
    }

    status = check_kind(resource->get_kind(), kind);
    if (status != ZX_OK) {
        return status;
    }

    // XXX: Check range here
    return ZX_OK;
}
