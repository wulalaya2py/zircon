// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#include <zircon/syscalls.h>
#include <zircon/device/block.h>
#include <zircon/misc/xorshiftrand.h>
#include <sync/completion.h>

void usage(void) {
    fprintf(stderr, "usage: blocklatency <args> <device>\n"
        "\n"
        "For all <num> use k/K, m/M, and g/G for ease of use/conversion\n"
        "args:  -wt <num> <num>    wait time range in nanoseconds between operations\n"
        "       -it <num>     number of operations to perform\n"
        "       -bs <num>     transfer block size (multiple of 4K)\n"
        "       -tt <num>     total bytes to transfer\n"
        "       -mo <num>     maximum outstanding ops (1..128)\n"
        "       -linear       transfers in linear order\n"
        "       -random       random transfers across total range\n"
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
        fprintf(stderr, "error: option %s needs a parameter\n", argv[n]); \
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

    atomic_int pending;
    completion_t signal;
} bio_random_args_t;

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
    fprintf(stderr, "%g %s", ftime, unit);
}

static void print_trial_data(zx_time_t res, size_t total, bio_random_args_t* a) {
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

    return ZX_OK;

fail:
    blkdev_close(blk);
    return ZX_ERR_INTERNAL;
}

size_t read_cline_args(int argc, char** argv, bio_random_args_t* bargs, char** dev_name) {
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
            }
        } else if (!strcmp(argv[0], "-tt")) {
            needparam(-1);
            total = str_to_number(argv[0], BYTEMODE);
        } else if (!strcmp(argv[0], "-mo")) {
            needparam(-1);
            size_t n = str_to_number(argv[0], BYTEMODE);
            if ((n < 1) || (n > 128)) {
                error("error: max pending must be between 1 and 128\n");
            }
            bargs->max_pending = n;
        } else if (!strcmp(argv[0], "-linear")) {
            bargs->linear = true;
        } else if (!strcmp(argv[0], "-random")) {
            bargs->linear = false;
        } else if(!(strcmp(argv[0], "-it"))) { //str_to_number of iterations
            needparam(-1);
            bargs->iters = str_to_number(argv[0], NUMMODE);
        } else if(!(strcmp(argv[0], "-wt"))) {  //range of times to wait in between trials
            needparam(-1);
            bargs->lbound = str_to_number(argv[0], NUMMODE);
            needparam(-2);
            bargs->ubound = str_to_number(argv[0], NUMMODE);
            if(bargs->lbound > bargs->ubound) {       //incorrect ordering of bounds
                uint64_t temp = bargs->lbound;
                bargs->lbound = bargs->ubound;
                bargs->ubound = temp;
            }
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

//Sets up blk for use as well as reads other arguments
int setup_blkdev(int argc, char** argv, bio_random_args_t* bargs) {

    nextarg();

    bargs->signal = COMPLETION_INIT;

    char* dev_name[50];       //path to device or device name

    size_t total = read_cline_args(argc, argv, bargs, dev_name);


    if(total == 0) return 0;

    int fd;
    if ((fd = open(*dev_name, O_RDONLY)) < 0) {
        fprintf(stderr, "error: cannot open '%s'\n", argv[3]);
        return -1;
    }
    if (blkdev_open(fd, argv[1], 8*1024*1024, bargs->blk) != ZX_OK) {
        return -1;
    }

    size_t devtotal = bargs->blk->info.block_count * bargs->blk->info.block_size;

    // default to entire device
    if ((total == 0) || (total > devtotal)) {
        total = devtotal;
    }
    bargs->count = total / bargs->xfer;

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
    bio_random_args_t* a = (bio_random_args_t*) arg;

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
            .opcode = BLOCKIO_READ | BLOCKIO_TXN_END,
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
        // zx_status_t r = zx_fifo_write(fifo, sizeof(req), &req, 1, NULL);
        zx_status_t r = zx_fifo_read(fifo, sizeof(req), &req, 1, NULL);
        if (r == ZX_ERR_SHOULD_WAIT) {
            r = zx_object_wait_one(fifo, ZX_FIFO_READABLE | ZX_FIFO_PEER_CLOSED,
                                   ZX_TIME_INFINITE, NULL);
            if (r != ZX_OK) {
                fprintf(stderr, "failed waiting for fifo\n");
                zx_handle_close(fifo);
                return -1;
            }
            continue;
        } else if (r < 0) {
            fprintf(stderr, "error: failed writing fifo\n");
            zx_handle_close(fifo);
            return -1;
        }

        atomic_fetch_add(&a->pending, 1);
        count--;
    }
    return 0;
}


static zx_status_t bio_random(bio_random_args_t* a, uint64_t* _total, zx_time_t* _res) {

    thrd_t t;
    int r;

    size_t count = a->count;
    zx_handle_t fifo = a->blk->fifo;

    zx_time_t t0 = zx_clock_get(ZX_CLOCK_MONOTONIC);
    thrd_create(&t, bio_random_thread, a);

    while (count > 0) {
        block_fifo_response_t resp;
        zx_status_t r = zx_fifo_read(fifo, sizeof(resp), &resp, 1, NULL);
        if (r == ZX_ERR_SHOULD_WAIT) {
            r = zx_object_wait_one(fifo, ZX_FIFO_WRITABLE | ZX_FIFO_PEER_CLOSED,
                                   ZX_TIME_INFINITE, NULL);
            if (r != ZX_OK) {
                fprintf(stderr, "failed waiting for fifo: %d\n", r);
                goto fail;
            }
            continue;
        } else if (r < 0) {
            fprintf(stderr, "error: failed reading fifo: %d\n", r);
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
int run_latency_trials(bio_random_args_t* a) {
    uint64_t current_iter;
    zx_time_t res;
    size_t total;
    zx_duration_t wait = 0;
    bool range = true;
    if (a->lbound == a->ubound) {   //no range specified, take constant wait time
        wait = a->lbound;
        range = false;
    }
    srand(a->seed);
    for (current_iter = 1; current_iter <= a->iters; current_iter++) {
        //TODO read/write
        //TODO trace
        //TODO linear/random
        if (range) {
            wait = (rand() % (a->ubound - a->lbound)) + a->lbound;
        }
        res = 0;
        total = 0;
        if (bio_random(a, &total, &res) != ZX_OK) {
            return -1;
        }

        print_trial_data(res, total, a);
        fprintf(stderr, "^^^Iteration number: %lu complete, waiting ", current_iter);
        nsec_to_str(wait);
        fprintf(stderr, "^^^\n\n");

        zx_nanosleep(zx_deadline_after(ZX_NSEC(wait)));
    }
    return 0;
}

int main(int argc, char** argv) {
    blkdev_t blk;
    bio_random_args_t tmp = {
        .blk = &blk,
        .xfer = 32768,
        .iters = 100,
        .lbound = 10,
        .ubound = 100,
        .seed = 7891263897612ULL,
        .max_pending = 128,
        .pending = 0,
        .linear = true,
    };
    bio_random_args_t* bargs = &tmp;
    setup_blkdev(argc, argv, bargs);
    run_latency_trials(bargs);
    return 0;
}