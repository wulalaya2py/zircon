// Copyright 2016 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <object/resource_dispatcher.h>

#include <zircon/rights.h>
#include <fbl/alloc_checker.h>

#include <kernel/auto_lock.h>
#include <lib/pasm/pasm.h>
#include <string.h>

bool ResourceDispatcher::root_created_ = false;

namespace {
static constexpr uint32_t resource_kind_to_allocator(uint64_t kind) {
    DEBUG_ASSERT(kind == ZX_RSRC_KIND_MMIO || kind == ZX_RSRC_KIND_IOPORT ||
                 kind == ZX_RSRC_KIND_IRQ);
    switch (kind) {
        case ZX_RSRC_KIND_MMIO:
            return Pasm::kMmioAllocator;
        case ZX_RSRC_KIND_IOPORT:
            return Pasm::kIoAllocator;
        case ZX_RSRC_KIND_IRQ:
            return Pasm::kIrqAllocator;
    }

    // Return an invalid Pasm region which will return an error in the Pasm calls.
    return UINT32_MAX;
}
} // namespace anon

zx_status_t ResourceDispatcher::Create(fbl::RefPtr<ResourceDispatcher>* dispatcher,
                                       zx_rights_t* rights, uint32_t kind,
                                       uint64_t low, uint64_t high) {
    if (kind >= ZX_RSRC_KIND_COUNT) {
        return ZX_ERR_INVALID_ARGS;
    }

    if (kind == ZX_RSRC_KIND_ROOT && root_created_) {
        return ZX_ERR_ALREADY_EXISTS;
    }

    fbl::AllocChecker ac;
    ResourceDispatcher* disp = new (&ac) ResourceDispatcher(kind, low, high);
    if (!ac.check()) {
        return ZX_ERR_NO_MEMORY;
    }

    zx_status_t st = disp->Initialize();
    if (st != ZX_OK) {
        delete disp;
        return st;
    }

    *rights = ZX_DEFAULT_RESOURCE_RIGHTS;
    *dispatcher = fbl::AdoptRef<ResourceDispatcher>(disp);
    return ZX_OK;
}

zx_status_t ResourceDispatcher::Initialize() {
    zx_status_t st = ZX_OK;
#ifdef RESOURCES_USE_PASM
    if (kind_ == ZX_RSRC_KIND_ROOT) {
        return st;
    }

    auto pasm = Pasm::Get()->Get();
    DEBUG_ASSERT(pasm);

    // TODO(cja): resources may want to take base/len instead of low/high to match the lib
    st = pasm->ReserveAddressSpace(resource_kind_to_allocator(kind_), low_, high_ - low_, region_);
    if (st != ZX_OK) {
        return st;
    }

    printf("resource: got region %#" PRIxPTR " size %#lx\n", region_->base, region_->size);
#endif
    return st;
}

ResourceDispatcher::ResourceDispatcher(uint32_t kind, uint64_t low, uint64_t high) :
    kind_(kind), low_(low), high_(high) {
    if (kind == ZX_RSRC_KIND_ROOT) {
        root_created_ = true;
    }
}

ResourceDispatcher::~ResourceDispatcher() {
}
