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

#include <dwc2/usb_dwc_regs.h>

#include <zircon/listnode.h>
#include <zircon/process.h>

#define MAX_DEVICE_COUNT 65
#define ROOT_HUB_DEVICE_ID (MAX_DEVICE_COUNT - 1)
#define NUM_HOST_CHANNELS 8

#define ALL_CHANNELS_FREE 0xff

// TODO - move to dwc_usb_t
extern volatile struct dwc_regs* regs;

typedef struct dwc_usb_device dwc_usb_device_t;

typedef enum dwc_endpoint_direction {
    DWC_EP_OUT = 0,
    DWC_EP_IN = 1,
} dwc_endpoint_direction_t;

typedef enum dwc_usb_data_toggle {
    DWC_TOGGLE_DATA0 = 0,
    DWC_TOGGLE_DATA1 = 2,
    DWC_TOGGLE_DATA2 = 1,
    DWC_TOGGLE_SETUP = 3,
} dwc_usb_data_toggle_t;

typedef enum dwc_ctrl_phase {
    CTRL_PHASE_SETUP = 1,
    CTRL_PHASE_DATA = 2,
    CTRL_PHASE_STATUS = 3,
} dwc_ctrl_phase_t;

typedef enum dwc_ep0_state {
    EP0_STATE_DISCONNECT,
    EP0_STATE_IDLE,
    EP0_STATE_DATA_OUT,
    EP0_STATE_DATA_IN,
    EP0_STATE_STATUS,
    EP0_STATE_STALL,
} dwc_ep0_state_t;

typedef struct dwc_usb_transfer_request {
    list_node_t node;

    dwc_ctrl_phase_t ctrl_phase;
    usb_request_t* setup_req;

    size_t bytes_transferred;
    dwc_usb_data_toggle_t next_data_toggle;
    bool complete_split;

    // Number of packets queued for transfer before programming the channel.
    uint32_t packets_queued;

    // Number of bytes queued for transfer before programming the channel.
    uint32_t bytes_queued;

    // Total number of bytes in this transaction.
    uint32_t total_bytes_queued;

    bool short_attempt;

    usb_request_t* usb_req;

    uint32_t cspit_retries;

    // DEBUG
    uint32_t request_id;
} dwc_usb_transfer_request_t;

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

    // Pertaining to root hub transactions.
    mtx_t rh_req_mtx;
    completion_t rh_req_completion;
    list_node_t rh_req_head;

    // Pertaining to a free list of request structures.
    mtx_t free_req_mtx;
    list_node_t free_reqs;
    size_t free_req_count;  // Number of free requests on this list.

    // List of devices attached to this controller.
    dwc_usb_device_t usb_devices[MAX_DEVICE_COUNT];

    // Pertaining to requests scheduled against the mock root hub.
    mtx_t rh_status_mtx;
    dwc_usb_transfer_request_t* rh_intr_req;
    usb_port_status_t root_port_status;

    // Pertaining to the availability of channels on this device.
    mtx_t free_channel_mtx;
    completion_t free_channel_completion;
    uint8_t free_channels;
    uint32_t next_device_address;

    // Assign a new request ID to each request so that we know when it's scheduled
    // and when it's executed.
    uint32_t DBG_reqid;

    union dwc_host_channel_interrupts channel_interrupts[NUM_HOST_CHANNELS];
    completion_t channel_complete[NUM_HOST_CHANNELS];

    // Pertaining to threads waiting to schedule a packet on the next start of
    // frame on this device.
    mtx_t sof_waiters_mtx;
    uint n_sof_waiters;
    completion_t sof_waiters[NUM_HOST_CHANNELS];

    // Pool of free requests to reuse.
    usb_request_pool_t free_usb_reqs;

    // device stuff
    io_buffer_t ep0_buffer;
    usb_dci_interface_t dci_intf;
    dwc_ep0_state_t ep0_state;
    usb_setup_t cur_setup;
    bool configured;
} dwc_usb_t;

typedef struct dwc_usb_endpoint {
    list_node_t node;
    uint8_t ep_address;

    mtx_t pending_request_mtx;
    list_node_t pending_requests; // List of requests pending for this endpoint.

    // Pointer to the device that owns this endpoint.
    dwc_usb_device_t* parent;

    usb_endpoint_descriptor_t desc;

    thrd_t request_scheduler_thread;
    completion_t request_pending_completion;
} dwc_usb_endpoint_t;

typedef struct dwc_usb_scheduler_thread_ctx {
    dwc_usb_endpoint_t* ep;
    dwc_usb_t* dwc;
} dwc_usb_scheduler_thread_ctx_t;

// dwc2-device.c
extern usb_dci_protocol_ops_t dwc_dci_protocol;

void dwc_handle_reset_irq(dwc_usb_t* dwc);
void dwc_handle_enumdone_irq(dwc_usb_t* dwc);
void dwc_handle_rxstsqlvl_irq(dwc_usb_t* dwc);
void dwc_handle_inepintr_irq(dwc_usb_t* dwc);
void dwc_handle_outepintr_irq(dwc_usb_t* dwc);
void dwc_handle_nptxfempty_irq(dwc_usb_t* dwc);

// dwc2-host.c
extern usb_hci_protocol_ops_t dwc_hci_protocol;

zx_status_t create_default_device(dwc_usb_t* dwc);
int dwc_root_hub_req_worker(void* arg);
void dwc_handle_port_irq(dwc_usb_t* dwc);
void dwc_handle_sof_irq(dwc_usb_t* dwc);
void dwc_handle_channel_irq(dwc_usb_t* dwc);
