// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "boot-shim.h"
#include "debug.h"
#include "devicetree.h"
#include "util.h"

#include <stddef.h>

#define ROUNDUP(a, b) (((a) + ((b)-1)) & ~((b)-1))

typedef enum {
    NODE_NONE,
    NODE_CHOSEN,
    NODE_MEMORY,
} node_t;

typedef struct {
    node_t  node;
    uintptr_t initrd_start;
    size_t memory;
    char* cmdline;
    size_t cmdline_length;
} device_tree_context_t;

static int node_callback(int depth, const char *name, void *cookie) {
    device_tree_context_t* ctx = cookie;

    if (!strcmp(name, "chosen")) {
        ctx->node = NODE_CHOSEN;
    } else if (!strcmp(name, "memory")) {
        ctx->node = NODE_MEMORY;
    } else {
        ctx->node = NODE_NONE;
    }

    return 0;
}

static int prop_callback(const char *name, uint8_t *data, uint32_t size, void *cookie) {
    device_tree_context_t* ctx = cookie;

    if (ctx->node == NODE_CHOSEN) {
        if (!strcmp(name, "linux,initrd-start")) {
            if (size == sizeof(uint32_t)) {
                ctx->initrd_start = dt_rd32(data);
            } else if (size == sizeof(uint64_t)) {
                ctx->initrd_start = dt_rd64(data);
            } else {
                fail("bad size for linux,initrd-start in device tree\n");
            }
        } else if (!strcmp(name, "bootargs")) {
            ctx->cmdline = (char *)data;
            ctx->cmdline_length = size;
        }
    } else if (ctx->node == NODE_MEMORY) {
        if (!strcmp(name, "reg") && size == 16) {
            // memory size is second uint64_t in memory descriptor
            ctx->memory = dt_rd64(data + sizeof(uint64_t));
        }
    }

    return 0;
}

static void read_device_tree(void* device_tree, device_tree_context_t* ctx) {
    devicetree_t dt;
    dt.error = uart_print;

    dt_init(&dt, device_tree, 0xffffffff);
    dt_walk(&dt, node_callback, prop_callback, ctx);
}

static void append_bootdata(bootdata_t* container, uint32_t type, void* payload, uint32_t length) {
    bootdata_t* dest = (bootdata_t*)((uintptr_t)container + container->length + sizeof(bootdata_t));

    dest->type = type;
    dest->length = length;
    dest->extra = 0;
    dest->flags = 0;
    dest->reserved0 = 0;
    dest->reserved1 = 0;
    dest->magic = BOOTITEM_MAGIC;
    dest->crc32 = BOOTITEM_NO_CRC32;

    if (length) {
        memcpy(dest + 1, payload, length);
    }
    length = BOOTDATA_ALIGN(length + sizeof(bootdata_t));
    container->length += length;
}

uint64_t boot_shim(void* device_tree, bootdata_t* bootdata) {
    uart_print("boot_shim: hi there!\n");

    // sanity check the bootdata headers
    // it must start with a container record followed by a kernel record
    zircon_kernel_t* kernel = (zircon_kernel_t*)bootdata;
    if (kernel->hdr_file.type != BOOTDATA_CONTAINER || kernel->hdr_file.extra != BOOTDATA_MAGIC ||
        kernel->hdr_file.magic != BOOTITEM_MAGIC || kernel->hdr_kernel.type != BOOTDATA_KERNEL ||
        kernel->hdr_kernel.magic != BOOTITEM_MAGIC) {
        fail("zircon_kernel_t sanity check failed\n");
    }

    uintptr_t kernel_base;
    uint32_t bootdata_size = kernel->hdr_file.length + sizeof(bootdata_t);
    uint32_t kernel_size = kernel->hdr_kernel.length + 2 * sizeof(bootdata_t);

    if (bootdata_size > kernel_size) {
        // we have more bootdata following the kernel.
        // we must relocate the kernel after the rest of the bootdata.

        // round up to align new kernel location
        bootdata_size = ROUNDUP(bootdata_size, KERNEL_ALIGN);
        kernel_base = (uintptr_t)kernel + bootdata_size;

        memcpy((void *)kernel_base, kernel, BOOTDATA_ALIGN(kernel_size));

    } else {
        kernel_base = (uintptr_t)kernel;
        // we are just a kernel image. we will need to find bootdata in device tree.
        bootdata = NULL;
    }

    device_tree_context_t ctx;
    ctx.node = NODE_NONE;
    ctx.initrd_start = 0;
    ctx.memory = 0;
    ctx.cmdline = NULL;
    read_device_tree(device_tree, &ctx);

    // check for ramdisk first
    if (!bootdata) {
        if (ctx.initrd_start) {
            bootdata = (bootdata_t*)ctx.initrd_start;
            if (bootdata->type != BOOTDATA_CONTAINER || bootdata->extra != BOOTDATA_MAGIC ||
                bootdata->magic != BOOTITEM_MAGIC) {
                fail("bad magic for bootdata in device tree\n");
            }
        } else {
            fail("could not find bootdata in device tree\n");
        }
    }

    if (ctx.memory) {
        bootdata_mem_range_t mem_range;
        mem_range.paddr = 0;
        mem_range.vaddr = 0;
        mem_range.length = ctx.memory;
        mem_range.type = BOOTDATA_MEM_RANGE_RAM;

        append_bootdata(bootdata, BOOTDATA_MEM_CONFIG, &mem_range, sizeof(mem_range));
    }

    if (ctx.cmdline && ctx.cmdline_length) {
        append_bootdata(bootdata, BOOTDATA_CMDLINE, ctx.cmdline, ctx.cmdline_length);
    }

    if (ctx.cmdline && ctx.cmdline_length) {
        append_bootdata(bootdata, BOOTDATA_CMDLINE, ctx.cmdline, ctx.cmdline_length);
    }

    // return pointer to bootdata in bootdata_return
    bootdata_return = bootdata;

    // return kernel entry point address
    return kernel_base + kernel->data_kernel.entry64;
}
