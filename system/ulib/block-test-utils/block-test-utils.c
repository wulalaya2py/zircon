// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <zircon/syscalls.h>
#include <zircon/device/block.h>
#include <zircon/misc/xorshiftrand.h>
#include <zircon/process.h>
#include <block-test-utils/block-test-utils.h>
#include <block-client/client.h>

void blkdev_close(blkdev_t* blk, fifo_client_t* client, void* buf) {
    if (blk->fd >= 0) {
        close(blk->fd);
    }
    zx_handle_close(blk->vmo);
    zx_handle_close(blk->fifo);
    zx_vmar_unmap(blk->vmo, (uintptr_t) buf, blk->bufsz);
    memset(blk, 0, sizeof(blkdev_t));
    block_fifo_release_client(client);
    blk->fd = -1;
}

zx_status_t blkdev_open(int fd, const char* dev, size_t bufsz,
                blkdev_t* blk, fifo_client_t** client, void** buf_addr) {
    memset(blk, 0, sizeof(blkdev_t));
    blk->fd = fd;
    blk->bufsz = bufsz;
    zx_status_t r = 1;  // 1 means unknown error (TODO(zapatoshoe) determine what unknown errors could be)
    if (ioctl_block_get_info(fd, &blk->info) != sizeof(block_info_t)) {
        fprintf(stderr, "error: cannot get block device info for '%s'\n", dev);
        goto fail;
    }
    if (ioctl_block_get_fifos(fd, &blk->fifo) != sizeof(zx_handle_t)) {
        fprintf(stderr, "error: cannot get fifo for '%s'\n", dev);
        goto fail;
    }
    if ((r = zx_vmo_create(bufsz, 0, &blk->vmo)) != ZX_OK) {
        fprintf(stderr, "error: out of memory %d\n", r);
        goto fail;
    }
    if ((r = zx_vmar_map(zx_vmar_root_self(), 0, blk->vmo, 0, bufsz, ZX_VM_FLAG_PERM_READ |
                ZX_VM_FLAG_PERM_WRITE, (uintptr_t*) buf_addr) != ZX_OK)) {
        fprintf(stderr, "error: failed to map vmo %d\n", r);
        goto fail;
    }

    // for (uint8_t n = 0; n < 128; n++) {
    //     if (ioctl_block_alloc_txn(fd, &blk->reqid) != sizeof(reqid_t)) {
    //         fprintf(stderr, "error: cannot allocate txn for '%s'\n", dev);
    //         r = ZX_ERR_NO_MEMORY;
    //         goto fail;
    //     }
    //     if (blk->reqid != n) {
    //         fprintf(stderr, "error: unexpected txid %u\n", blk->reqid);
    //         goto fail;
    //     }
    // }

    zx_handle_t dup;
    if ((r = zx_handle_duplicate(blk->vmo, ZX_RIGHT_SAME_RIGHTS, &dup)) != ZX_OK) {
        fprintf(stderr, "error: cannot duplicate handle %d\n", r);
        goto fail;
    }
    if (ioctl_block_attach_vmo(fd, &dup, &blk->vmoid) != sizeof(vmoid_t)) {
        fprintf(stderr, "error: cannot attach vmo for '%s'\n", dev);
        goto fail;
    }
    if ((r = block_fifo_create_client(blk->fifo, client)) != ZX_OK) {
        fprintf(stderr, "error: cannot create client %d\n", r);
        goto fail;
    }

    return ZX_OK;
fail:
    blkdev_close(blk, *client, buf_addr);
    return r == 1 ? ZX_ERR_INTERNAL : r;
}
