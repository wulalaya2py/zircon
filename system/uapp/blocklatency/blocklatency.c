// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <threads.h>

#include <block-client/client.h>
#include <zircon/syscalls.h>
#include <zircon/device/block.h>
#include <zircon/misc/xorshiftrand.h>
#include <zircon/process.h>
#include <sync/completion.h>


void usage(void) {
    fprintf(stderr, "usage: blocklatency <args> <device>\n"
        "\n"
        "For all <num> use k/K, m/M, and g/G for ease of use/conversion\n"
        "args:  -wt <num> <num>    wait time is_wait_range in nanoseconds between operations {default: 10-100ns}\n"
        "       -it <num>       number of operations to perform {default: 100}\n"
        "       -bs <num>       transfer block size (multiple of 4K) {default: 32k}\n"
        "       -tt <num>       total bytes to transfer\n"
        "       -mo <num>       maximum outstanding ops (1..128)\n"
        "       -read           only performs reads {default}\n"
        "       -write          only performs writes\n"
        "       -read-write-reg alternates between read and write\n"
        "       -linear         transfers in linear order {default}\n"
        "       -random         transfers in random order\n"
        );

}

#define DEBUG(x...) fprintf(stderr, x)

//Macros for using str_to_number()
#define NUMMODE 1000    //used for standard orders of 1000 (seconds)
#define BYTEMODE 1024    //used for number of bytes

#define nextarg() do { argc--; argv++; } while (0)

//n is which index of argv has the parameter relative to current place
#define needparam(n) do { \
    argc--; argv++; \
    if (argc == 0) { \
        fprintf(stderr, "error: option %s needs one or more parameters\n", argv[n]); \
        usage(); \
        return -1; \
    } } while (0)

#define error(x...) do { fprintf(stderr, x); usage(); return -1; } while (0)

typedef struct {
    int fd;
    zx_handle_t vmo;
    zx_handle_t fifo;
    txnid_t txnid;
    vmoid_t vmoid;
    size_t bufsz;
    block_info_t info;
} blkdev_t;

typedef struct {
    blkdev_t* blk;
    size_t count;
    size_t xfer;
    uint64_t iters;
    zx_duration_t lbound;
    zx_duration_t ubound;
    uint64_t seed;
    int max_pending;
    bool linear;
    uint8_t operation;
    atomic_int pending;
    completion_t signal;
} latency_args_t;

static fifo_client_t* client;
static void* buf;
static const uint64_t bufsz = 8*1024*1024;

static uint64_t str_to_number(const char* str, uint16_t thousand) {
    char* end;
    uint64_t n = strtoull(str, &end, 10);

    uint64_t m = 1;
    switch (*end) {
    case 'G':
    case 'g':
        m = thousand*thousand*thousand;
        break;
    case 'M':
    case 'm':
        m = thousand*thousand;
        break;
    case 'K':
    case 'k':
        m = thousand;
        break;
    }
    return m * n;
}

static void bytes_per_second(uint64_t bytes, uint64_t nanos) {
    double s = ((double)nanos) / ((double)1000000000);
    double rate = ((double)bytes) / s;

    const char* unit = "B";
    if (rate > 1024*1024) {
        unit = "MB";
        rate /= 1024*1024;
    } else if (rate > 1024) {
        unit = "KB";
        rate /= 1024;
    }
    fprintf(stderr, "%g %s/s\n", rate, unit);
}

static void ops_per_second(uint64_t count, uint64_t nanos) {
    double s = ((double)nanos) / ((double)1000000000);
    double rate = ((double)count) / s;
    fprintf(stderr, "%g %s/s\n", rate, "ops");
}

static void nsec_to_str(uint64_t nanos) {
    float ftime;
    const char* unit;
    if (nanos > 1000*1000*1000) {
        unit = "seconds";
        ftime = nanos / (1000.0*1000.0*1000.0);
    } else if (nanos > 1000*1000) {
        unit = "millis";
        ftime = nanos / (1000.0*1000.0);
    } else if (nanos > 1000) {
        unit = "micros";
        ftime = nanos / 1000.0;
    } else {
        unit = "nanos";
        ftime = (float) nanos;
    }
    fprintf(stderr, "%07.3f %s", ftime, unit);
}

static void print_trial_data(zx_time_t res, size_t total, latency_args_t* a) {
    fprintf(stderr, "%zu bytes in ", total);
    nsec_to_str(res);
    fprintf(stderr, ": ");
    bytes_per_second(total, res);
    fprintf(stderr, "%zu ops in ", a->count);
    nsec_to_str(res);
    fprintf(stderr, ": ");
    ops_per_second(a->count, res);
}

static void blkdev_close(blkdev_t* blk) {
    if (blk->fd >= 0) {
        close(blk->fd);
    }
    zx_handle_close(blk->vmo);
    zx_handle_close(blk->fifo);
    memset(blk, 0, sizeof(blkdev_t));
    blk->fd = -1;
}

static zx_status_t blkdev_open(int fd, const char* dev, size_t bufsz, blkdev_t* blk) {
    memset(blk, 0, sizeof(blkdev_t));
    blk->fd = fd;
    blk->bufsz = bufsz;

    zx_status_t r;
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
    if((r = zx_vmar_map(zx_vmar_root_self(), 0, blk->vmo, 0, bufsz, ZX_VM_FLAG_PERM_READ | 
                ZX_VM_FLAG_PERM_WRITE, (uintptr_t*) &buf) != ZX_OK)) {
        fprintf(stderr, "error: failed to map vmo %d\n", r);
        goto fail;
    }

    for (uint8_t n = 0; n < 128; n++) {
        if (ioctl_block_alloc_txn(fd, &blk->txnid) != sizeof(txnid_t)) {
            fprintf(stderr, "error: cannot allocate txn for '%s'\n", dev);
            goto fail;
        }
        if (blk->txnid != n) {
            fprintf(stderr, "error: unexpected txid %u\n", blk->txnid);
            goto fail;
        }
    }

    zx_handle_t dup;
    if ((r = zx_handle_duplicate(blk->vmo, ZX_RIGHT_SAME_RIGHTS, &dup)) != ZX_OK) {
        fprintf(stderr, "error: cannot duplicate handle %d\n", r);
        goto fail;
    }
    if (ioctl_block_attach_vmo(fd, &dup, &blk->vmoid) != sizeof(vmoid_t)) {
        fprintf(stderr, "error: cannot attach vmo for '%s'\n", dev);
        goto fail;
    }

    return block_fifo_create_client(blk->fifo, &client);

fail:
    blkdev_close(blk);
    return ZX_ERR_INTERNAL;
}

size_t read_cline_args(int argc, char** argv, latency_args_t* bargs, char** dev_name) {
    size_t total = 0;
    while (argc > 0) {
        if (argv[0][0] != '-') {
            break;
        }
        if (!strcmp(argv[0], "-bs")) {
            needparam(-1);
            bargs->xfer = str_to_number(argv[0], BYTEMODE);
            if ((bargs->xfer == 0) || (bargs->xfer % 4096)) {
                error("error: block size must be multiple of 4K\n");
                return 0;
            }
        } else if (!strcmp(argv[0], "-tt")) {
            needparam(-1);
            total = str_to_number(argv[0], BYTEMODE);
        } else if (!strcmp(argv[0], "-mo")) {
            needparam(-1);
            size_t n = str_to_number(argv[0], BYTEMODE);
            if ((n < 1) || (n > 128)) {
                error("error: max pending must be between 1 and 128\n");
                return 0;
            }
            bargs->max_pending = n;
        } else if(!(strcmp(argv[0], "-it"))) { //str_to_number of iterations
            needparam(-1);
            bargs->iters = str_to_number(argv[0], NUMMODE);
        } else if(!(strcmp(argv[0], "-wt"))) {  //is_wait_range of times to wait in between trials
            needparam(-1);
            bargs->lbound = str_to_number(argv[0], NUMMODE);
            needparam(-2);
            bargs->ubound = str_to_number(argv[0], NUMMODE);
            if(bargs->lbound > bargs->ubound) {       //incorrect ordering of bounds
                uint64_t temp = bargs->lbound;
                bargs->lbound = bargs->ubound;
                bargs->ubound = temp;
            }
        } else if (!strcmp(argv[0], "-read")) {
            bargs->operation = BLOCKIO_READ;
        } else if (!strcmp(argv[0], "-write")) {
            bargs->operation = BLOCKIO_WRITE;
        } else if (!strcmp(argv[0], "-read-write-reg")) {
            bargs->operation = BLOCKIO_READ | BLOCKIO_WRITE;
        } else if (!strcmp(argv[0], "-linear")) {
            bargs->linear = true;
        } else if (!strcmp(argv[0], "-random")) {
            bargs->linear = false;
        } else if (!strcmp(argv[0], "-h")) {
            usage();
            return 0;
        } else {
            error("error: unknown option: %s\n", argv[0]);
        }
        nextarg();
    }
    if (argc == 0) {
        error("error: no device specified\n");
    }
    if (argc > 1) {
        error("error: unexpected arguments\n");
    }

    *dev_name = argv[0];    //path to device or device name

    return total;
}

static void fill_buffer(blkdev_t* blk, size_t len) {
    size_t i;
    uint64_t* buffer = buf;
    for(i = 0; i < len/8; i++) {
        buffer[i] = rand();
    }
}

static void print_buffer(uint64_t begin, uint64_t end) {
    uint64_t i, k = begin;
    uint8_t* buffer = buf;
    while(k < end) {
        for(i = 0; i < 66; i++) {
            fprintf(stderr, "%03d ", buffer[k]);
            if(++k >= end) break;
        }
        fprintf(stderr, "\n");
    }
}

//Sets up blk for use as well as reads other arguments
static int setup_blkdev(int argc, char** argv, latency_args_t* bargs) {

    nextarg();

    bargs->signal = COMPLETION_INIT;

    char* dev_name[50];       //path to device or device name

    size_t total = read_cline_args(argc, argv, bargs, dev_name);

    if(total <= 0 || total == (size_t) -1) return -1;

    zx_status_t r = 0;

    int fd;
    if ((fd = open(*dev_name, O_RDONLY)) < 0) {
        fprintf(stderr, "error: cannot open '%s'\n", *dev_name);
        return -1;
    }
    if ((r = blkdev_open(fd, *dev_name, bufsz, bargs->blk)) != ZX_OK) {
        return -1;
    }

    size_t devtotal = bargs->blk->info.block_count * bargs->blk->info.block_size;

    // default to entire device
    if ((total == 0) || (total > devtotal)) {
        total = devtotal;
    }
    bargs->count = total / bargs->xfer;
    bargs->count += total % bargs->xfer ? 1 : 0;    //Add one more txn if a full one can't be done at the end

    return 0;
}

static atomic_uint_fast64_t IDMAP0 = 0xFFFFFFFFFFFFFFFFULL;
static atomic_uint_fast64_t IDMAP1 = 0xFFFFFFFFFFFFFFFFULL;
static txnid_t GET(void) {
    uint64_t n = __builtin_ffsll(atomic_load(&IDMAP0));
    if (n > 0) {
        n--;
        atomic_fetch_and(&IDMAP0, ~(1ULL << n));
        return n;
    } else if ((n = __builtin_ffsll(atomic_load(&IDMAP1))) > 0) {
        n--;
        atomic_fetch_and(&IDMAP1, ~(1ULL << n));
        return n + 64;
    } else {
        fprintf(stderr, "FATAL OUT OF IDS\n");
        sleep(100);
        exit(-1);
    }
}

static void PUT(uint64_t n) {
    if (n > 127) {
        fprintf(stderr, "FATAL BAD ID %zu\n", n);
        sleep(100);
        exit(-1);
    }
    if (n > 63) {
        atomic_fetch_or(&IDMAP1, 1ULL << (n - 64));
    } else {
        atomic_fetch_or(&IDMAP0, 1ULL << n);
    }
}
static int bio_random_thread(void* arg) {
    latency_args_t* a = (latency_args_t*) arg;

    size_t off = 0;
    size_t count = a->count;
    size_t xfer = a->xfer;

    size_t blksize = a->blk->info.block_size;
    size_t blkcount = ((count * xfer) / blksize) - (xfer / blksize);

    rand64_t r64 = RAND63SEED(a->seed);

    zx_handle_t fifo = a->blk->fifo;
    size_t dev_off = 0;

    while (count > 0) {
        while (atomic_load(&a->pending) == a->max_pending) {
            completion_wait(&a->signal, ZX_TIME_INFINITE);
            completion_reset(&a->signal);
        }
        
        block_fifo_request_t req = {
            .txnid = GET(),
            .vmoid = a->blk->vmoid,
            // .opcode = BLOCKIO_READ | BLOCKIO_TXN_END,
            .length = xfer,
            .vmo_offset = off,
        };

        if (a->linear) {
            req.dev_offset = dev_off;
            dev_off += xfer;
        } else {
            req.dev_offset = (rand64(&r64) % blkcount) * blksize;
        }
        off += xfer;
        if ((off + xfer) > a->blk->bufsz) {
            off = 0;
        }

        req.length /= blksize;
        req.dev_offset /= blksize;
        req.vmo_offset /= blksize;

#if 0
        fprintf(stderr, "IO tid=%u vid=%u op=%x len=%zu vof=%zu dof=%zu\n",
                req.txnid, req.vmoid, req.opcode, req.length, req.vmo_offset, req.dev_offset);
#endif
        zx_status_t r = 0;
        switch(a->operation) {
        case BLOCKIO_READ: 
            req.opcode = BLOCKIO_READ | BLOCKIO_TXN_END;
            r = zx_fifo_read(fifo, sizeof(req), &req, 1, NULL);
            break;
        case BLOCKIO_WRITE: 
            req.opcode = BLOCKIO_WRITE | BLOCKIO_TXN_END; 
            r = zx_fifo_write(fifo, sizeof(req), &req, 1, NULL);
            break;
        }
        // zx_status_t r = zx_fifo_write(fifo, sizeof(req), &req, 1, NULL);
        // zx_status_t r = zx_fifo_read(fifo, sizeof(req), &req, 1, NULL);
        if (r == ZX_ERR_SHOULD_WAIT) {
            switch(a->operation) {
            case BLOCKIO_READ: 
                r = zx_object_wait_one(fifo, ZX_FIFO_READABLE | ZX_FIFO_PEER_CLOSED,
                                   ZX_TIME_INFINITE, NULL);
                break;
            case BLOCKIO_WRITE: 
                r = zx_object_wait_one(fifo, ZX_FIFO_WRITABLE | ZX_FIFO_PEER_CLOSED,
                                   ZX_TIME_INFINITE, NULL);
                break;
            }
            if (r != ZX_OK) {
                fprintf(stderr, "failed waiting for fifo\n");
                zx_handle_close(fifo);
                return -1;
            }
            continue;
        } else if (r < 0) {
            fprintf(stderr, "error: failed writing fifo\n");    //TODO or reading?
            zx_handle_close(fifo);
            return -1;
        }

        atomic_fetch_add(&a->pending, 1);
        count--;
    }
    return 0;
}


static zx_status_t bio_random(latency_args_t* a, uint64_t* _total, zx_time_t* _res) {

    thrd_t t;
    int r;

    size_t count = a->count;
    zx_handle_t fifo = a->blk->fifo;

    zx_time_t t0 = zx_clock_get(ZX_CLOCK_MONOTONIC);
    thrd_create(&t, bio_random_thread, a);

    while (count > 0) {
        block_fifo_response_t resp;
        zx_status_t r = 0;
        switch(a->operation) {
        case BLOCKIO_READ: 
            r = zx_fifo_read(fifo, sizeof(resp), &resp, 1, NULL);
            break;
        case BLOCKIO_WRITE: 
            r = zx_fifo_write(fifo, sizeof(resp), &resp, 1, NULL);
            break;
        }
        if (r == ZX_ERR_SHOULD_WAIT) {
            switch(a->operation) {
            case BLOCKIO_READ: 
                r = zx_object_wait_one(fifo, ZX_FIFO_READABLE | ZX_FIFO_PEER_CLOSED,
                                   ZX_TIME_INFINITE, NULL);
                break;
            case BLOCKIO_WRITE: 
                r = zx_object_wait_one(fifo, ZX_FIFO_WRITABLE | ZX_FIFO_PEER_CLOSED,
                                   ZX_TIME_INFINITE, NULL);
                break;
            }
            if (r != ZX_OK) {
                fprintf(stderr, "failed waiting for fifo: %d\n", r);
                goto fail;
            }
            continue;
        } else if (r < 0) {
            fprintf(stderr, "error: failed reading fifo: %d\n", r);     //TODO or writing?
            goto fail;
        }
        if (resp.status != ZX_OK) {
            fprintf(stderr, "error: io txn failed %d (%zu remaining)\n",
                    resp.status, count);
            goto fail;
        }
        PUT(resp.txnid);
        count--;
        if (atomic_fetch_sub(&a->pending, 1) == a->max_pending) {
            completion_signal(&a->signal);
        }
    }

    zx_time_t t1 = zx_clock_get(ZX_CLOCK_MONOTONIC);

    fprintf(stderr, "waiting for thread to exit...\n");
    thrd_join(t, &r);

    *_res = t1 - t0;
    *_total = a->count * a->xfer;
    return ZX_OK;

fail:
    zx_handle_close(a->blk->fifo);
    thrd_join(t, &r);
    return ZX_ERR_IO;
}

///Performs block_fifo_txn's specified by a and request on a random location
/**
 * Loops for a->count number of times performing txn's of request->length blocks at a time,
 * Before each txn, the offset and length are computed for both the vmo and dev
 * Returns ZX_OK upon completion or if block_fifo_txn fails, returns its status
 */
static zx_status_t block_random_io(latency_args_t* a, block_fifo_request_t* request, rand64_t* seed) {
    zx_status_t r = ZX_OK;
    uint64_t num_txn;                                               //current txn we're on
    uint64_t vmo_offset, vmo_length, vmo_size;                      //vmo_offset is in blocks, the others are bytes
    zx_vmo_get_size(a->blk->vmo, &vmo_size);
    size_t block_count = a->blk->info.block_count;                  //number of total blocks
    size_t block_size = a->blk->info.block_size;                    //size of a block in bytes

    request->dev_offset = rand64(seed) % block_count;

    //loop for number of operations calculated in setup_blkdev
    for(num_txn = 0; num_txn < a->count; num_txn++) {   
        
        request->dev_offset += (num_txn * a->xfer) / block_size;    //go to next integer block
        request->dev_offset %= block_count;
        if(request->dev_offset + request->length > block_count) {   //Don't read/write out of device bounds
            request->length = block_count - request->dev_offset;    //use remaining length of device
        }
        vmo_offset = num_txn * a->xfer;                             //go to next chunk in vmo
        vmo_length = request->length * block_size;                  //size of next chunk
        if(vmo_length + vmo_offset > vmo_size) {                    //wrap around if OOB
            vmo_offset = 0;
        }
        request->vmo_offset = vmo_offset / block_size;

        fprintf(stderr, "Devsize: %lu, CurBlock: %lu, CurLength: %lu\n", block_count, request->dev_offset, request->length);

        if ((r = block_fifo_txn(client, request, 1)) != ZX_OK) {    //read/write from/to vmo and dev
            return r;
        }

    }
    return ZX_OK;
}

///Performs block_fifo_txn's specified by a and request
/**
 * Loops for a->count number of times performing txn's of request->length blocks at a time,
 * Before each txn, the offset and length are computed for both the vmo and dev
 * Returns ZX_OK upon completion or if block_fifo_txn fails, returns its status
 */
static zx_status_t block_linear_io(latency_args_t* a, block_fifo_request_t* request) {
    zx_status_t r = ZX_OK;
    uint64_t num_txn;       //current txn we're on
    uint64_t vmo_offset, vmo_length, vmo_size;  //vmo_offset is in blocks, the others are bytes
    zx_vmo_get_size(a->blk->vmo, &vmo_size);
    size_t block_count = a->blk->info.block_count;  //number of total blocks
    size_t block_size = a->blk->info.block_size;    //size of a block in bytes

    //loop for number of operations calculated in setup_blkdev
    for(num_txn = 0; num_txn < a->count; num_txn++) {   
        
        request->dev_offset = (num_txn * a->xfer) / block_size;    //go to next integer block
        if(request->dev_offset + request->length > block_count) {//Don't read/write out of device bounds
            request->length = block_count - request->dev_offset;   //use remaining length of device
        }
        vmo_offset = num_txn * a->xfer;                             //go to next chunk in vmo
        vmo_length = request->length * block_size;                  //size of next chunk
        if(vmo_length + vmo_offset > vmo_size) {                       //wrap around if OOB
            vmo_offset = 0;
        }
        request->vmo_offset = vmo_offset / block_size;

        if ((r = block_fifo_txn(client, request, 1)) != ZX_OK) {    //read/write from/to vmo and dev
            return r;
        }

    }
    return ZX_OK;
}

///Performs read/write requests on a block device determined and with arguments determined by read_cline_args with 
/**
 * Will perform random or linear transactions with arguments in "a" determining length,
 * offset on the device, combinations of read and write, and wait times between operations
 */
zx_status_t run_latency_trials(latency_args_t* a) {
    srand(a->seed);
    zx_duration_t wait = 0;
    zx_status_t r = 0;
    bool is_wait_range = true;
    bool alternate_rw = a->operation == (BLOCKIO_READ | BLOCKIO_WRITE);
    zx_time_t prev = 0;
    block_fifo_request_t request = {
        .txnid = a->blk->txnid,
        .vmoid = a->blk->vmoid,
        .opcode = alternate_rw ? BLOCKIO_READ : a->operation,   //if we are alternating, start on read
        .length = a->xfer / a->blk->info.block_size,    //need an integer number of actual blocks
        .vmo_offset = 0,        //number of blocks offset for vmo
        .dev_offset = 0         //number of blocks offset for dev
    };
    rand64_t seed = RAND63SEED(a->seed);


    if (a->lbound == a->ubound) {   //no wait range specified, take constant wait time
        wait = a->lbound;
        is_wait_range = false;
    }
    uint64_t current_iter;
    for (current_iter = 1; current_iter <= a->iters; current_iter++) {

        if (is_wait_range) {
            wait = (rand64(&seed) % (a->ubound - a->lbound)) + a->lbound;
        }

        fill_buffer(a->blk, bufsz);     //Fill with random numbers

        prev = zx_deadline_after(0);    //for recording time per iteration
        r = a->linear ? block_linear_io(a, &request) : block_random_io(a, &request, &seed);
        if(r != ZX_OK) {
            fprintf(stderr, "Errno: %d\n", r);
        }
        nsec_to_str(zx_deadline_after(0) - prev);
        fprintf(stderr, "\n");

        if(alternate_rw) request.opcode ^= BLOCKIO_READ | BLOCKIO_WRITE; //alternate between read and write

        zx_nanosleep(zx_deadline_after(ZX_NSEC(wait)));
    }
    return ZX_OK;
}

int main(int argc, char** argv) {
    zx_status_t r = ZX_OK;
    blkdev_t blk;
    latency_args_t tmp = {   //default settings
        .blk = &blk,
        .xfer = 32768,
        .iters = 100,
        .lbound = 10,
        .ubound = 100,
        .operation = BLOCKIO_READ,
        .seed = 7891263897612ULL,
        .max_pending = 128,
        .pending = 0,
        .linear = true,
    };
    latency_args_t* bargs = &tmp;
    if(setup_blkdev(argc, argv, bargs) < 0) return 0;
    if((r = run_latency_trials(bargs)) != ZX_OK) {
        fprintf(stderr, "Error #: %d\n", r);
    }
    block_fifo_release_client(client);
    blkdev_close(bargs->blk);
    return 0;
}
