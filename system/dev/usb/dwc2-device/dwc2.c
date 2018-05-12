// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "dwc2.h"

#define MMIO_INDEX  0
#define IRQ_INDEX   0

/*
static zx_status_t wait_bits(volatile uint32_t* ptr, uint32_t bits, uint32_t expected) {
    for (int i = 0; i < 1000; i++) {
        if ((*ptr & bits) == expected) {
            return ZX_OK;
        }
        usleep(1000);
    }
    return ZX_ERR_TIMED_OUT;
}
*/

static zx_status_t usb_dwc_softreset_core(dwc_usb_t* dwc) {
    dwc_regs_t* regs = dwc->regs;

/* do we need this?
    zx_status_t status = wait_bits(&regs->core_reset.val, DWC_AHB_MASTER_IDLE, DWC_AHB_MASTER_IDLE);
    if (status != ZX_OK) {
        return status;
    }
*/
    dwc_grstctl_t grstctl = {0};
    grstctl.csftrst = 1;
    regs->grstctl = grstctl;

    for (int i = 0; i < 1000; i++) {
        if (regs->grstctl.csftrst == 0) {
            return ZX_OK;
        }
        usleep(1000);
    }
    return ZX_ERR_TIMED_OUT;
}

static zx_status_t usb_dwc_setupcontroller(dwc_usb_t* dwc) {
    dwc_regs_t* regs = dwc->regs;

    const uint32_t rx_words = 1024;
    const uint32_t tx_words = 1024;
//    const uint32_t ptx_words = 1024;

    regs->grxfsiz = rx_words;
    regs->gnptxfsiz = (tx_words << 16) | rx_words;
//??    regs->host_periodic_tx_fifo_size = (ptx_words << 16) | (rx_words + tx_words);

    regs->gahbcfg.hburstlen = 1;
    regs->gahbcfg.dmaenable = 1;

    // device role initialization
    regs->dctl.sftdiscon = 1;
    regs->dctl.sftdiscon = 0;

    regs->diepmsk.val = 0;
    regs->doepmsk.val = 0;
    regs->daint = 0xFFFFFFFF;
    regs->daintmsk = 0;
    for (int i = 0; i < MAX_EPS_CHANNELS; i++) {
        regs->depin[i].diepctl.val = 0;
        regs->depout[i].doepctl.val = 0;
        regs->depin[i].dieptsiz.val = 0;
        regs->depout[i].doeptsiz.val = 0;
    }

    // reset phy clock
    regs->pcgcctl.val = 0;

    dwc_interrupts_t gintmsk = {0};

    gintmsk.rxstsqlvl = 1;
    gintmsk.usbreset = 1;
    gintmsk.enumdone = 1;
    gintmsk.inepintr = 1;
    gintmsk.outepintr = 1;
//    gintmsk.sof_intr = 1;
    gintmsk.usbsuspend = 1;

printf("enabling interrupts %08x\n", gintmsk.val);

    regs->gintmsk = gintmsk;

/*
    union dwc_core_configuration core_configuration;
    core_configuration = regs->core_configuration;
printf("core_configuration: %08x\n", core_configuration.val);
    core_configuration.force_host_mode = 0;
    core_configuration.force_dev_mode = 0;
    regs->core_configuration = core_configuration;
*/

    regs->gahbcfg.glblintrmsk = 1;

    return ZX_OK;
}

static zx_status_t dwc_get_initial_mode(void* ctx, usb_mode_t* out_mode) {
    *out_mode = USB_MODE_DEVICE;
    return ZX_OK;
}

static zx_status_t dwc_set_mode(void* ctx, usb_mode_t mode) {
    return ZX_OK;
}

usb_mode_switch_protocol_ops_t dwc_ums_protocol = {
    .get_initial_mode = dwc_get_initial_mode,
    .set_mode = dwc_set_mode,
};

static zx_status_t dwc_get_protocol(void* ctx, uint32_t proto_id, void* out) {
    switch (proto_id) {
    case ZX_PROTOCOL_USB_DCI: {
        usb_dci_protocol_t* proto = out;
        proto->ops = &dwc_dci_protocol;
        proto->ctx = ctx;
        return ZX_OK;
    }
    case ZX_PROTOCOL_USB_MODE_SWITCH: {
        usb_mode_switch_protocol_t* proto = out;
        proto->ops = &dwc_ums_protocol;
        proto->ctx = ctx;
        return ZX_OK;
    }
    default:
        return ZX_ERR_NOT_SUPPORTED;
    }
}

static void dwc_unbind(void* ctx) {
    zxlogf(ERROR, "dwc_usb: dwc_unbind not implemented\n");
}

static void dwc_release(void* ctx) {
    zxlogf(ERROR, "dwc_usb: dwc_release not implemented\n");
}

static zx_protocol_device_t dwc_device_proto = {
    .version = DEVICE_OPS_VERSION,
    .get_protocol = dwc_get_protocol,
    .unbind = dwc_unbind,
    .release = dwc_release,
};

static void dwc_handle_irq(dwc_usb_t* dwc) {
    dwc_regs_t* regs = dwc->regs;
    dwc_interrupts_t interrupts = regs->gintsts;
    dwc_interrupts_t mask = regs->gintmsk;

printf("XXXXXXXXXXXXXXXXX dwc_handle_irq: %08x\n", interrupts.val);

    interrupts.val &= mask.val;

    // clear interrupt
    uint32_t gotgint = regs->gotgint;
    regs->gotgint = gotgint;
/*
uint32_t reserved0         : 1;
uint32_t modemismatch      : 1;
uint32_t otgintr           : 1;
uint32_t sof_intr          : 1;

uint32_t rxstsqlvl         :1;
uint32_t nptxfempty        :1;
uint32_t ginnakeff         :1;
uint32_t goutnakeff        :1;

uint32_t ulpickint         :1;
uint32_t i2cintr           :1;
uint32_t erlysuspend       :1;
uint32_t usbsuspend        :1;

uint32_t usbreset          :1;
uint32_t enumdone          :1;
uint32_t isooutdrop        :1;
uint32_t eopframe          :1;

uint32_t restoredone       :1;
uint32_t epmismatch        :1;
uint32_t inepintr          :1;
uint32_t outepintr         :1;

uint32_t incomplisoin      :1;
uint32_t incomplisoout     :1;
uint32_t fetsusp           :1;
uint32_t resetdet          :1;

uint32_t port_intr         : 1;
uint32_t host_channel_intr : 1;
uint32_t ptxfempty         :1;
uint32_t lpmtranrcvd       :1;

uint32_t conidstschng      :1;
uint32_t disconnect        :1;
uint32_t sessreqintr       :1;
uint32_t wkupintr          :1;
*/

//04008028  sof_intr nptxfempty outepintr ptxfempty
//  04088028 sof_intr nptxfempty eopframe resetdet ptxfempty
// 54008c20 nptxfempty erlysuspend usbsuspend eopframe ptxfempty conidstschng sessreqintr

    if (interrupts.rxstsqlvl) {
        printf("rxstsqlvl\n");
        dwc_handle_rxstsqlvl_irq(dwc);
    }
    if (interrupts.nptxfempty) {
        dwc_handle_nptxfempty_irq(dwc);
    }
    if (interrupts.usbreset) {
        dwc_handle_reset_irq(dwc);
    }
    if (interrupts.usbsuspend) {
        regs->gintsts.usbsuspend = 1;
    }
    if (interrupts.enumdone) {
        dwc_handle_enumdone_irq(dwc);
    }
    if (interrupts.inepintr) {
        dwc_handle_inepintr_irq(dwc);
    }
    if (interrupts.outepintr) {
        dwc_handle_outepintr_irq(dwc);
    }

    // ????
    regs->gintsts = interrupts;
}

// Thread to handle interrupts.
static int dwc_irq_thread(void* arg) {
    dwc_usb_t* dwc = (dwc_usb_t*)arg;

//sleep(2);

    while (1) {
        zx_status_t wait_res;
        wait_res = zx_interrupt_wait(dwc->irq_handle, NULL);
        if (wait_res != ZX_OK)
            zxlogf(ERROR, "dwc_usb: irq wait failed, retcode = %d\n", wait_res);

        dwc_handle_irq(dwc);
    }

    zxlogf(INFO, "dwc_usb: irq thread finished\n");
    return 0;
}

// Bind is the entry point for this driver.
static zx_status_t usb_dwc_bind(void* ctx, zx_device_t* dev) {
    zxlogf(TRACE, "usb_dwc: bind dev = %p\n", dev);

    platform_device_protocol_t proto;
    zx_status_t status = device_get_protocol(dev, ZX_PROTOCOL_PLATFORM_DEV, &proto);
    if (status != ZX_OK) {
        return status;
    }

    // Allocate a new device object for the bus.
    dwc_usb_t* dwc = calloc(1, sizeof(*dwc));
    if (!dwc) {
        zxlogf(ERROR, "usb_dwc_bind: bind failed to allocate usb_dwc struct\n");
        return ZX_ERR_NO_MEMORY;
    }

    for (unsigned i = 0; i < countof(dwc->eps); i++) {
        dwc_endpoint_t* ep = &dwc->eps[i];
        ep->ep_num = i;
        mtx_init(&ep->lock, mtx_plain);
        list_initialize(&ep->queued_reqs);
    }

    // Carve out some address space for this device.
    size_t mmio_size;
    zx_handle_t mmio_handle = ZX_HANDLE_INVALID;
    status = pdev_map_mmio(&proto, MMIO_INDEX, ZX_CACHE_POLICY_UNCACHED_DEVICE, (void **)&dwc->regs,
                       &mmio_size, &mmio_handle);
    if (status != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: bind failed to pdev_map_mmio.\n");
        goto error_return;
    }

    // Create an IRQ Handle for this device.
    status = pdev_map_interrupt(&proto, IRQ_INDEX, &dwc->irq_handle);
    if (status != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: bind failed to map usb irq.\n");
        goto error_return;
    }

    status = pdev_get_bti(&proto, 0, &dwc->bti_handle);
    if (status != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: bind failed to get bti handle.\n");
        goto error_return;
    }

    dwc->parent = dev;

sleep(10);

    if ((status = usb_dwc_softreset_core(dwc)) != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: failed to reset core.\n");
        goto error_return;
    }

    if ((status = usb_dwc_setupcontroller(dwc)) != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: failed setup controller.\n");
        goto error_return;
    }

    status = io_buffer_init(&dwc->ep0_buffer,  dwc->bti_handle, 65536,
                            IO_BUFFER_RW | IO_BUFFER_CONTIG);
    if (status != ZX_OK) {
        zxlogf(ERROR, "dwc3_bind: io_buffer_init failed\n");
        goto error_return;
    }

   device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "dwc2",
        .ctx = dwc,
        .ops = &dwc_device_proto,
        .proto_id = ZX_PROTOCOL_USB_DCI,
        .proto_ops = &dwc_dci_protocol,
    };

    if ((status = device_add(dev, &args, &dwc->zxdev)) != ZX_OK) {
        free(dwc);
        return status;
    }

    thrd_t irq_thread;
    thrd_create_with_name(&irq_thread, dwc_irq_thread, dwc, "dwc_irq_thread");
    thrd_detach(irq_thread);

    zxlogf(TRACE, "usb_dwc: bind success!\n");
    return ZX_OK;

error_return:
    if (dwc) {
        if (dwc->regs) {
            zx_vmar_unmap(zx_vmar_root_self(), (uintptr_t)dwc->regs, mmio_size);
        }
        zx_handle_close(mmio_handle);
        zx_handle_close(dwc->irq_handle);
        zx_handle_close(dwc->bti_handle);
        free(dwc);
    }

    return status;
}

static zx_driver_ops_t usb_dwc_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = usb_dwc_bind,
};

// The formatter does not play nice with these macros.
// clang-format off
ZIRCON_DRIVER_BEGIN(dwc2, usb_dwc_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_GENERIC),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_GENERIC),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_USB_DWC2_DEVICE),
ZIRCON_DRIVER_END(dwc2)
// clang-format on
