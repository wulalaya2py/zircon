// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

// Standard Includes
#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

// DDK includes
#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/io-buffer.h>
#include <ddk/protocol/platform-defs.h>
#include <ddk/protocol/platform-device.h>
#include <ddk/protocol/usb-bus.h>
#include <ddk/protocol/usb-dci.h>
#include <ddk/protocol/usb-hci.h>
#include <ddk/protocol/usb-mode-switch.h>
#include <ddk/protocol/usb.h>

// Zircon USB includes
#include <zircon/hw/usb-hub.h>
#include <zircon/hw/usb.h>
#include <sync/completion.h>

#include "usb_dwc_regs.h"

#include <zircon/listnode.h>
#include <zircon/process.h>

// TODO - move to dwc_usb_t
extern volatile struct dwc_regs* regs;

typedef enum dwc_ep0_state {
    EP0_STATE_DISCONNECT,
    EP0_STATE_IDLE,
    EP0_STATE_DATA_OUT,
    EP0_STATE_DATA_IN,
    EP0_STATE_STATUS,
    EP0_STATE_STALL,
} dwc_ep0_state_t;

typedef struct dwc_usb_device {
    mtx_t devmtx;

    usb_speed_t speed;
    uint32_t hub_address;
    int port;
    uint32_t device_id;

    list_node_t endpoints;
} dwc_usb_device_t;

typedef struct dwc_usb {
    zx_device_t* zxdev;
    usb_bus_interface_t bus;
    zx_handle_t irq_handle;
    zx_handle_t bti_handle;
    thrd_t irq_thread;
    zx_device_t* parent;

    // device stuff
    io_buffer_t ep0_buffer;
    usb_dci_interface_t dci_intf;
    dwc_ep0_state_t ep0_state;
    usb_setup_t cur_setup;
    bool configured;
} dwc_usb_t;

// dwc2-device.c
extern usb_dci_protocol_ops_t dwc_dci_protocol;

void dwc_handle_reset_irq(dwc_usb_t* dwc);
void dwc_handle_enumdone_irq(dwc_usb_t* dwc);
void dwc_handle_rxstsqlvl_irq(dwc_usb_t* dwc);
void dwc_handle_inepintr_irq(dwc_usb_t* dwc);
void dwc_handle_outepintr_irq(dwc_usb_t* dwc);
void dwc_handle_nptxfempty_irq(dwc_usb_t* dwc);
