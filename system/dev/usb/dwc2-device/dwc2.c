// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "dwc2.h"

#define PAGE_MASK_4K (0xFFF)
#define USB_PAGE_START (USB_BASE & (~PAGE_MASK_4K))
#define USB_PAGE_SIZE (0x1000)
#define PAGE_REG_DELTA (USB_BASE - USB_PAGE_START)

#define MMIO_INDEX  0
#define IRQ_INDEX   0


volatile struct dwc_regs* regs = NULL;

static zx_status_t wait_bits(volatile uint32_t* ptr, uint32_t bits, uint32_t expected) {
    for (int i = 0; i < 1000; i++) {
        if ((*ptr & bits) == expected) {
            return ZX_OK;
        }
        usleep(1000);
    }
    return ZX_ERR_TIMED_OUT;
}

static zx_status_t usb_dwc_softreset_core(void) {
    zx_status_t status = wait_bits(&regs->core_reset.val, DWC_AHB_MASTER_IDLE, DWC_AHB_MASTER_IDLE);
    if (status != ZX_OK) {
        return status;
    }

    regs->core_reset.val = DWC_SOFT_RESET;

    return wait_bits(&regs->core_reset.val, DWC_SOFT_RESET, 0);
}

static zx_status_t usb_dwc_setupcontroller(void) {
    const uint32_t rx_words = 1024;
    const uint32_t tx_words = 1024;
    const uint32_t ptx_words = 1024;

    regs->rx_fifo_size = rx_words;
    regs->nonperiodic_tx_fifo_size = (tx_words << 16) | rx_words;
    regs->host_periodic_tx_fifo_size = (ptx_words << 16) | (rx_words + tx_words);

    regs->ahb_configuration |= DWC_AHB_DMA_ENABLE | BCM_DWC_AHB_AXI_WAIT;

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
    regs->power = 0;

    union dwc_core_interrupts core_interrupt_mask = {0};

//    core_interrupt_mask.host_channel_intr = 1;
//    core_interrupt_mask.port_intr = 1;
    core_interrupt_mask.rxstsqlvl = 1;
    core_interrupt_mask.usbreset = 1;
    core_interrupt_mask.enumdone = 1;
    core_interrupt_mask.inepintr = 1;
    core_interrupt_mask.outepintr = 1;
//    core_interrupt_mask.sof_intr = 1;
    core_interrupt_mask.usbsuspend = 1;

printf("enabling interrupts %08x\n", core_interrupt_mask.val);

    regs->core_interrupt_mask = core_interrupt_mask;


/*
    union dwc_core_configuration core_configuration;
    core_configuration = regs->core_configuration;
printf("core_configuration: %08x\n", core_configuration.val);
    core_configuration.force_host_mode = 0;
    core_configuration.force_dev_mode = 0;
    regs->core_configuration = core_configuration;
*/

    regs->ahb_configuration |= DWC_AHB_INTERRUPT_ENABLE;

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
    union dwc_core_interrupts interrupts = regs->core_interrupts;
printf("XXXXXXXXXXXXXXXXX dwc_handle_irq: %08x\n", interrupts.val);

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

/* host
    if (interrupts.port_intr) {
        dwc_handle_port_irq(dwc);
    }
    if (interrupts.sof_intr) {
        dwc_handle_sof_irq(dwc);
    }
    if (interrupts.host_channel_intr) {
        dwc_handle_channel_irq(dwc);
    }
*/
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
        regs->core_interrupts.usbsuspend = 1;
    }
    if (interrupts.enumdone) {
        dwc_handle_enumdone_irq(dwc);
    }
    if (interrupts.inepintr) {
        printf("inepintr\n");
        dwc_handle_inepintr_irq(dwc);
    }
    if (interrupts.outepintr) {
        printf("outepintr\n");
        dwc_handle_outepintr_irq(dwc);
    }

//    regs->core_interrupts = interrupts;
}

// Thread to handle interrupts.
static int dwc_irq_thread(void* arg) {
    dwc_usb_t* dwc = (dwc_usb_t*)arg;

sleep(2);

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

    dwc_usb_t* usb_dwc = NULL;

    platform_device_protocol_t proto;
    zx_status_t status = device_get_protocol(dev, ZX_PROTOCOL_PLATFORM_DEV, &proto);
    if (status != ZX_OK) {
        return status;
    }

    // Allocate a new device object for the bus.
    usb_dwc = calloc(1, sizeof(*usb_dwc));
    if (!usb_dwc) {
        zxlogf(ERROR, "usb_dwc: bind failed to allocate usb_dwc struct\n");
        return ZX_ERR_NO_MEMORY;
    }

    usb_dwc->free_channel_completion = COMPLETION_INIT;
    usb_dwc->free_channels = ALL_CHANNELS_FREE;
    usb_dwc->next_device_address = 1;
    usb_dwc->DBG_reqid = 0x1;
    usb_request_pool_init(&usb_dwc->free_usb_reqs);

    // Carve out some address space for this device.
    size_t mmio_size;
    zx_handle_t mmio_handle = ZX_HANDLE_INVALID;
    status = pdev_map_mmio(&proto, MMIO_INDEX, ZX_CACHE_POLICY_UNCACHED_DEVICE, (void **)&regs,
                       &mmio_size, &mmio_handle);
    if (status != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: bind failed to pdev_map_mmio.\n");
        goto error_return;
    }

    // Create an IRQ Handle for this device.
    status = pdev_map_interrupt(&proto, IRQ_INDEX, &usb_dwc->irq_handle);
    if (status != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: bind failed to map usb irq.\n");
        goto error_return;
    }

    status = pdev_get_bti(&proto, 0, &usb_dwc->bti_handle);
    if (status != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: bind failed to get bti handle.\n");
        goto error_return;
    }

    usb_dwc->parent = dev;
    list_initialize(&usb_dwc->rh_req_head);

    // Initialize the free list.
    mtx_lock(&usb_dwc->free_req_mtx);
    list_initialize(&usb_dwc->free_reqs);
    mtx_unlock(&usb_dwc->free_req_mtx);

    if ((status = usb_dwc_softreset_core()) != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: failed to reset core.\n");
        goto error_return;
    }

    if ((status = usb_dwc_setupcontroller()) != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: failed setup controller.\n");
        goto error_return;
    }

    // Initialize all the channel completions.
    for (size_t i = 0; i < NUM_HOST_CHANNELS; i++) {
        usb_dwc->channel_complete[i] = COMPLETION_INIT;
        usb_dwc->sof_waiters[i] = COMPLETION_INIT;
    }

    // We create a mock device at device_id = 0 for enumeration purposes.
    // Any new device that connects to the bus is assigned this ID until we
    // set its address.
    if ((status = create_default_device(usb_dwc)) != ZX_OK) {
        zxlogf(ERROR, "usb_dwc: failed to create default device. "
                "retcode = %d\n", status);
        goto error_return;
    }

    status = io_buffer_init(&usb_dwc->ep0_buffer,  usb_dwc->bti_handle, 65536,
                            IO_BUFFER_RW | IO_BUFFER_CONTIG);
    if (status != ZX_OK) {
        zxlogf(ERROR, "dwc3_bind: io_buffer_init failed\n");
        goto error_return;
    }

/*
    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "dwc2",
        .ctx = usb_dwc,
        .ops = &dwc_device_proto,
        .proto_id = ZX_PROTOCOL_USB_HCI,
        .proto_ops = &dwc_hci_protocol,
    };
*/
   device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "dwc2",
        .ctx = usb_dwc,
        .ops = &dwc_device_proto,
        .proto_id = ZX_PROTOCOL_USB_DCI,
        .proto_ops = &dwc_dci_protocol,
    };

    if ((status = device_add(dev, &args, &usb_dwc->zxdev)) != ZX_OK) {
        free(usb_dwc);
        return status;
    }

    // Thread that responds to requests for the root hub.
    thrd_t root_hub_req_worker;
    thrd_create_with_name(&root_hub_req_worker, dwc_root_hub_req_worker,
                          usb_dwc, "dwc_root_hub_req_worker");
    thrd_detach(root_hub_req_worker);

    thrd_t irq_thread;
    thrd_create_with_name(&irq_thread, dwc_irq_thread, usb_dwc,
                          "dwc_irq_thread");
    thrd_detach(irq_thread);

    zxlogf(TRACE, "usb_dwc: bind success!\n");
    return ZX_OK;

error_return:
    if (regs) {
        zx_vmar_unmap(zx_vmar_root_self(), (uintptr_t)regs, mmio_size);
    }
    zx_handle_close(mmio_handle);
    if (usb_dwc) {
        zx_handle_close(usb_dwc->irq_handle);
        zx_handle_close(usb_dwc->bti_handle);
        free(usb_dwc);
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
