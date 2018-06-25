// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef __BLK_OP_H
#define __BLK_OP_H

#include <stdio.h>
#include <stdlib.h>

#include <zircon/device/block.h>
#include <zircon/types.h>
#include <block-client/client.h>


__BEGIN_CDECLS

typedef struct {
    int fd;
    zx_handle_t vmo;
    zx_handle_t fifo;
    reqid_t reqid;
    vmoid_t vmoid;
    size_t bufsz;
    block_info_t info;
} blkdev_t;

typedef struct {
    fifo_client_t* client;
    block_fifo_request_t* req;
    uint64_t thrd_num;
} block_thread_args_t;

// Closes the specified device and client
void blkdev_close(blkdev_t* blk, fifo_client_t* client, void* buf);

/**
 * Opens a block device into blk, and a fifo_client_t in client 
 * 
 * @param fd Is the file descriptor number of the device to be opened
 * @param dev Is the path to the device
 * @param bufsz Size of buffer to map vmar to in bytes
 * @param blk Struct to contain relevant information for block device
 * @param client The fifo_client_t to facilitate I/O operations on block device
 * @param buf_addr Address of the buffer being mapped
 */
zx_status_t blkdev_open(int fd, const char* dev, size_t bufsz,
                blkdev_t* blk, fifo_client_t** client, void** buf_addr);

__END_CDECLS

#endif
