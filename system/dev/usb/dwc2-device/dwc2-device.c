// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>

#include "dwc2.h"

static void dwc2_ep0_out_start(dwc_usb_t* dwc)
{
    printf("dwc2_ep0_out_start\n");

	deptsiz0_t doeptsize0 = {0};
	depctl_t doepctl = {0};

	doeptsize0.supcnt = 3;
	doeptsize0.pktcnt = 1;
	doeptsize0.xfersize = 8*3;
    regs->depout[0].doeptsiz.val = doeptsize0.val;

	doepctl.epena = 1;
    regs->depout[0].doepctl = doepctl;

//	flush_cpu_cache();
}

static void dwc_ep0_complete_request(dwc_usb_t* dwc) {
    printf("dwc_ep0_complete_request\n");

}

static zx_status_t dwc_handle_setup(dwc_usb_t* dwc, usb_setup_t* setup, void* buffer, size_t length,
                                     size_t* out_actual) {
printf("dwc_handle_setup\n");
    zx_status_t status;

    if (setup->bmRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE)) {
        // handle some special setup requests in this driver
        switch (setup->bRequest) {
        case USB_REQ_SET_ADDRESS:
            zxlogf(TRACE, "SET_ADDRESS %d\n", setup->wValue);
//            dwc_set_address(dwc, setup->wValue);
            *out_actual = 0;
            return ZX_OK;
        case USB_REQ_SET_CONFIGURATION:
            zxlogf(TRACE, "SET_CONFIGURATION %d\n", setup->wValue);
//            dwc3_reset_configuration(dwc);
            dwc->configured = false;
            status = usb_dci_control(&dwc->dci_intf, setup, buffer, length, out_actual);
            if (status == ZX_OK && setup->wValue) {
                dwc->configured = true;
//                dwc3_start_eps(dwc);
            }
            return status;
        default:
            // fall through to usb_dci_control()
            break;
        }
    } else if (setup->bmRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) &&
               setup->bRequest == USB_REQ_SET_INTERFACE) {
        zxlogf(TRACE, "SET_INTERFACE %d\n", setup->wValue);
//        dwc3_reset_configuration(dwc);
        dwc->configured = false;
        status = usb_dci_control(&dwc->dci_intf, setup, buffer, length, out_actual);
        if (status == ZX_OK) {
            dwc->configured = true;
//            dwc3_start_eps(dwc);
        }
        return status;
    }

    return usb_dci_control(&dwc->dci_intf, setup, buffer, length, out_actual);
}

static void dwc_ep_start_transfer(dwc_usb_t* dwc, unsigned ep_num, bool is_in, zx_paddr_t buffer,
                                  size_t length, bool send_zlp) {
printf("ZZZZZZZ dwc_ep_start_transfer\n");

	depctl_t depctl;
	volatile deptsiz_t* deptsiz;
//	uint32_t ep_mps = 64; // _ep->maxpacket;

//    _ep->total_len = _ep->xfer_len;

	if (is_in) {
		depctl = regs->depin[ep_num].diepctl;
		deptsiz = &regs->depin[ep_num].dieptsiz;
	} else {
		depctl = regs->depout[ep_num].doepctl;
		deptsiz = &regs->depout[ep_num].doeptsiz;
	}

#if 0
	/* Zero Length Packet? */
	if (0 == _ep->xfer_len) {
		deptsiz.b.xfersize = is_in ? 0 : ep_mps;
		deptsiz.b.pktcnt = 1;
	} else {
		deptsiz.b.pktcnt =
		(_ep->xfer_len + (ep_mps - 1)) /ep_mps;
		if (is_in && _ep->xfer_len < ep_mps)
			deptsiz.b.xfersize = _ep->xfer_len;
		else
			deptsiz.b.xfersize = _ep->xfer_len - _ep->xfer_count;
	}

	/* Fill size and count */
	dwc_write_reg32(epctltsize_reg, deptsiz.d32);

	/* IN endpoint */
	if (is_in) {
		gintmsk_data_t intr_mask = {0};

		/* First clear it from GINTSTS */
		intr_mask.b.nptxfempty = 1;
		dwc_modify_reg32(DWC_REG_GINTSTS, intr_mask.d32, 0);
		dwc_modify_reg32(DWC_REG_GINTMSK, intr_mask.d32, intr_mask.d32);
	}
#else
    deptsiz->pktcnt = 1;
	deptsiz->xfersize = length;
	
	if (is_in) {
		regs->gintsts.nptxfempty = 0;
		regs->gintmsk.nptxfempty = 1;
	}
#endif

#if DOEY_THE_DMA
	if (is_in) {
		regs->depin[ep_num].diepdma = buffer;
		regs->depin[ep_num].dieptsiz.pktcnt = 0;     // FIXME
		regs->depin[ep_num].dieptsiz.xfersize = length;     // FIXME
	} else {
		regs->depout[ep_num].doepdma = buffer;
	}
#endif

	/* EP enable */
	depctl.cnak = 1;
	depctl.epena = 1;

	if (is_in) {
		regs->depin[ep_num].diepctl = depctl;
	} else {
		regs->depout[ep_num].doepctl = depctl;
	}
}

static void dwc_handle_ep0(dwc_usb_t* dwc) {
    printf("dwc_handle_ep0\n");

/*    
  	pcd_struct_t * _pcd = &gadget_wrapper.pcd;
	dwc_ep_t * ep0 = &_pcd->dwc_eps[0].dwc_ep;
	struct usb_req_flag *req_flag = &gadget_wrapper.req_flag;
*/
	switch (dwc->ep0_state) {
	case EP0_STATE_IDLE: {
printf("dwc_handle_ep0 EP0_STATE_IDLE\n");
//		req_flag->request_config = 0;
	    usb_setup_t* setup = &dwc->cur_setup;

        bool is_out = ((setup->bmRequestType & USB_DIR_MASK) == USB_DIR_OUT);
        if (setup->wLength > 0 && is_out) {
            // queue a read for the data phase
            dwc_ep_start_transfer(dwc, 0, false, io_buffer_phys(&dwc->ep0_buffer), setup->wLength, false);
            dwc->ep0_state = EP0_STATE_DATA_OUT;
        } else {
            size_t actual;
            zx_status_t status = dwc_handle_setup(dwc, setup, io_buffer_virt(&dwc->ep0_buffer),
                                                  dwc->ep0_buffer.size, &actual);
            zxlogf(INFO, "dwc_handle_setup returned %d actual %zu\n", status, actual);
//            if (status != ZX_OK) {
//                dwc3_cmd_ep_set_stall(dwc, EP0_OUT);
//                dwc3_queue_setup_locked(dwc);
//                break;
//            }

            if (setup->wLength > 0) {
                // queue a write for the data phase
printf("queue write\n");
                io_buffer_cache_flush(&dwc->ep0_buffer, 0, actual);
                dwc_ep_start_transfer(dwc, 0, true, io_buffer_phys(&dwc->ep0_buffer), actual, false);
                dwc->ep0_state = EP0_STATE_DATA_IN;
            } else {
//                dwc->ep0_state = EP0_STATE_WAIT_NRDY_IN;
            }
        }    	
	    break;
    }
	case EP0_STATE_DATA_IN:
    printf("dwc_handle_ep0 EP0_STATE_DATA_IN\n");
//		if (ep0->xfer_count < ep0->total_len)
//			printf("FIX ME!! dwc_otg_ep0_continue_transfer!\n");
//		else
			dwc_ep0_complete_request(dwc);
		break;
	case EP0_STATE_DATA_OUT:
    printf("dwc_handle_ep0 EP0_STATE_DATA_OUT\n");
		dwc_ep0_complete_request(dwc);
		break;
	case EP0_STATE_STATUS:
    printf("dwc_handle_ep0 EP0_STATE_STATUS\n");
		dwc_ep0_complete_request(dwc);
		/* OUT for next SETUP */
		dwc->ep0_state = EP0_STATE_IDLE;
//		ep0->stopped = 1;
//		ep0->is_in = 0;
		break;

	case EP0_STATE_STALL:
	case EP0_STATE_DISCONNECT:
	default:
		printf("EP0 state is %d, should not get here pcd_setup()\n", dwc->ep0_state);
		break;
    }
}

static void dwc_complete_ep(dwc_usb_t* dwc, uint32_t ep_num, int is_in) {
    printf("dwc_complete_ep\n");

#if 0
	deptsiz_data_t deptsiz;
	pcd_struct_t *pcd = &gadget_wrapper.pcd;
	dwc_ep_t * ep;
	u32 epnum = ep_num;

	if (ep_num) {
		if (!is_in)
			epnum = ep_num + 1;
	}

	ep = &pcd->dwc_eps[epnum].dwc_ep;

	if (is_in) {
		pcd->dwc_eps[epnum].req->actual = ep->xfer_len;
		deptsiz.d32 = dwc_read_reg32(DWC_REG_IN_EP_TSIZE(ep_num));
		if (deptsiz.b.xfersize == 0 && deptsiz.b.pktcnt == 0 &&
                    ep->xfer_count == ep->xfer_len) {
			ep->start_xfer_buff = 0;
			ep->xfer_buff = 0;
			ep->xfer_len = 0;
		}
		pcd->dwc_eps[epnum].req->status = 0;
	} else {
		deptsiz.d32 = dwc_read_reg32(DWC_REG_OUT_EP_TSIZE(ep_num));
		pcd->dwc_eps[epnum].req->actual = ep->xfer_count;
		ep->start_xfer_buff = 0;
		ep->xfer_buff = 0;
		ep->xfer_len = 0;
		pcd->dwc_eps[epnum].req->status = 0;
	}

	if (pcd->dwc_eps[epnum].req->complete) {
		pcd->dwc_eps[epnum].req->complete((struct usb_ep *)(pcd->dwc_eps[epnum].priv), pcd->dwc_eps[epnum].req);
	}
#endif
}


static void dwc_flush_fifo(dwc_usb_t* dwc, const int num) {
    dwc_grstctl_t grstctl = {0};

	grstctl.txfflsh = 1;
	grstctl.txfnum = num;
	regs->grstctl = grstctl;
	
    uint32_t count = 0;
	do {
	    grstctl = regs->grstctl;
		if (++count > 10000)
			break;
	} while (grstctl.txfflsh == 1);

    zx_nanosleep(zx_deadline_after(ZX_USEC(1)));

	if (num == 0) {
		return;
    }

    grstctl.val = 0;
	grstctl.rxfflsh = 1;
	regs->grstctl = grstctl;

	count = 0;
	do {
	    grstctl = regs->grstctl;
		if (++count > 10000)
			break;
	} while (grstctl.rxfflsh == 1);

    zx_nanosleep(zx_deadline_after(ZX_USEC(1)));
}

void dwc_handle_reset_irq(dwc_usb_t* dwc) {
	zxlogf(INFO, "USB RESET\n");

	/* Clear the Remote Wakeup Signalling */
	regs->dctl.rmtwkupsig = 1;

	for (int i = 0; i < MAX_EPS_CHANNELS; i++) {
	    depctl_t diepctl = regs->depin[i].diepctl;

        if (diepctl.epena) {
            // disable all active IN EPs
            diepctl.snak = 1;
            diepctl.epdis = 1;
    	    regs->depin[i].diepctl = diepctl;
        }

        regs->depout[i].doepctl.snak = 1;
	}

	/* Flush the NP Tx FIFO */
	dwc_flush_fifo(dwc, 0);

	/* Flush the Learning Queue */
	regs->grstctl.intknqflsh = 1;

    // EPO IN and OUT
	regs->daintmsk = (1 < DWC_EP_IN_SHIFT) | (1 < DWC_EP_OUT_SHIFT);

    dwc_doepmsk_t doepmsk = {0};
	doepmsk.setup = 1;
	doepmsk.xfercompl = 1;
	doepmsk.ahberr = 1;
	doepmsk.epdisabled = 1;
	regs->doepmsk = doepmsk;

    dwc_diepmsk_t diepmsk = {0};
	diepmsk.xfercompl = 1;
	diepmsk.timeout = 1;
	diepmsk.epdisabled = 1;
	diepmsk.ahberr = 1;
	regs->diepmsk = diepmsk;

	/* Reset Device Address */
	regs->dcfg.devaddr = 0;

	/* setup EP0 to receive SETUP packets */
	dwc2_ep0_out_start(dwc);

	/* Clear interrupt */
    dwc_interrupts_t gintsts = {0};
	gintsts.usbreset = 1;
	regs->gintsts = gintsts;

//	flush_cpu_cache();
}

void dwc_handle_enumdone_irq(dwc_usb_t* dwc) {
	zxlogf(INFO, "SPEED ENUM\n");

    dwc->ep0_state = EP0_STATE_IDLE;

    depctl_t diepctl = regs->depin[0].diepctl;
    diepctl.mps = DWC_DEP0CTL_MPS_64;
    regs->depin[0].diepctl = diepctl;

    depctl_t doepctl = regs->depout[0].doepctl;
    doepctl.epena = 1;
    regs->depout[0].doepctl = doepctl;

    dwc_dctl_t dctl = {0};
    dctl.cgnpinnak = 1;
    regs->dctl = dctl;

	/* high speed */
	regs->gusbcfg.usbtrdtim = 5;

	/* Clear interrupt */
	dwc_interrupts_t gintsts = {0};
	gintsts.enumdone = 1;
	regs->gintsts = gintsts;
}

void dwc_handle_rxstsqlvl_irq(dwc_usb_t* dwc) {
printf("dwc_handle_rxstsqlvl_irq\n");
/*
	gintmsk_data_t gintmask = {0};
	gintsts_data_t gintsts = {0};
	dwc_ep_t *ep;
	pcd_struct_t *pcd = &gadget_wrapper.pcd;
	struct usb_req_flag *req_flag = &gadget_wrapper.req_flag;
	device_grxsts_data_t status;
*/

	/* Disable the Rx Status Queue Level interrupt */
    regs->gintmsk.rxstsqlvl = 0;

	/* Get the Status from the top of the FIFO */
	 dwc_grxstsp_t grxstsp = regs->grxstsp;
	if (grxstsp.epnum != 0)
		grxstsp.epnum = 2;
	/* Get pointer to EP structure */
//	ep = &pcd->dwc_eps[status.b.epnum].dwc_ep;

	switch (grxstsp.pktsts) {
	case DWC_STS_DATA_UPDT:
printf("DWC_STS_DATA_UPDT\n");
/*
		if (status.b.bcnt && ep->xfer_buff) {
			dwc_otg_read_packet(ep->xfer_buff, status.b.bcnt);
			ep->xfer_count += status.b.bcnt;
			ep->xfer_buff += status.b.bcnt;
		}
*/
		break;

	case DWC_DSTS_SETUP_UPDT:
printf("DWC_STS_DATA_UPDT\n");
{
    volatile uint32_t* fifo = (uint32_t *)((uint8_t *)regs + 0x1000);
    uint32_t* dest = (uint32_t*)&dwc->cur_setup;
    dest[0] = *fifo;
    dest[1] = *fifo;
printf("SETUP bmRequestType: 0x%02x bRequest: %u wValue: %u wIndex: %u wLength: %u\n",
       dwc->cur_setup.bmRequestType, dwc->cur_setup.bRequest, dwc->cur_setup.wValue,
       dwc->cur_setup.wIndex, dwc->cur_setup.wLength); 
}
//		req_flag->request_enable = 1;
//		ep->xfer_count += status.b.bcnt;
		break;

	case DWC_DSTS_GOUT_NAK:
printf("dwc_handle_rxstsqlvl_irq DWC_DSTS_GOUT_NAK\n");
        break;
	case DWC_STS_XFER_COMP:
printf("dwc_handle_rxstsqlvl_irq DWC_STS_XFER_COMP\n");
        break;
	case DWC_DSTS_SETUP_COMP:
printf("dwc_handle_rxstsqlvl_irq DWC_DSTS_SETUP_COMP\n");
        break;
	default:
printf("dwc_handle_rxstsqlvl_irq default\n");
		break;
	}


	/* Enable the Rx Status Queue Level interrupt */
    regs->gintmsk.rxstsqlvl = 1;

	/* Clear interrupt */
	dwc_interrupts_t gintsts = {0};
	gintsts.rxstsqlvl = 1;
	regs->gintsts = gintsts;
}

void dwc_handle_inepintr_irq(dwc_usb_t* dwc) {
printf("dwc_handle_inepintr_irq\n");

#if 0
	diepint_data_t diepint = {0};
	gintmsk_data_t intr_mask = {0};
	gintsts_data_t gintsts = {0};
	u32 ep_intr;
	u32 epnum = 0;

	/* Read in the device interrupt bits */
	ep_intr = dwc_read_reg32(DWC_REG_DAINT);

	ep_intr = (dwc_read_reg32(DWC_REG_DAINT) &
		dwc_read_reg32(DWC_REG_DAINTMSK));
	ep_intr =(ep_intr & 0xffff);

	/* Clear the INEPINT in GINTSTS */
	/* Clear all the interrupt bits for all IN endpoints in DAINT */
	gintsts.b.inepint = 1;
	dwc_write_reg32(DWC_REG_GINTSTS, gintsts.d32);
	dwc_write_reg32(DWC_REG_DAINT, 0xFFFF);
	flush_cpu_cache();

	/* Service the Device IN interrupts for each endpoint */
	while (ep_intr) {
		if (ep_intr & 0x1) {
			diepint.d32 = (dwc_read_reg32(DWC_REG_IN_EP_INTR(epnum)) &
				dwc_read_reg32(DWC_REG_DAINTMSK));

			/* Transfer complete */
			if (diepint.b.xfercompl) {
				/* Disable the NP Tx FIFO Empty Interrrupt  */
				intr_mask.b.nptxfempty = 1;
				dwc_modify_reg32(DWC_REG_GINTMSK, intr_mask.d32, 0);
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(epnum, xfercompl);
				/* Complete the transfer */
				if (0 == epnum) {
					handle_ep0();
				} else {
					dwc_complete_ep(dwc, epnum, 1);
					if (diepint.b.nak)
						CLEAR_IN_EP_INTR(epnum, nak);
				}
			}
			/* Endpoint disable  */
			if (diepint.b.epdisabled) {
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(epnum, epdisabled);
			}
			/* AHB Error */
			if (diepint.b.ahberr) {
				/* Clear the bit in DIEPINTn for this interrupt */
				CLEAR_IN_EP_INTR(epnum, ahberr);
			}
			/* TimeOUT Handshake (non-ISOC IN EPs) */
			if (diepint.b.timeout) {
				handle_in_ep_timeout_intr(epnum);
				CLEAR_IN_EP_INTR(epnum, timeout);
			}
			/** IN Token received with TxF Empty */
			if (diepint.b.intktxfemp) {
				CLEAR_IN_EP_INTR(epnum, intktxfemp);
			}
			/** IN Token Received with EP mismatch */
			if (diepint.b.intknepmis) {
				CLEAR_IN_EP_INTR(epnum, intknepmis);
			}
			/** IN Endpoint NAK Effective */
			if (diepint.b.inepnakeff) {
				CLEAR_IN_EP_INTR(epnum, inepnakeff);
			}
		}
		epnum++;
		ep_intr >>= 1;
	}
#endif
	/* Clear interrupt */
	dwc_interrupts_t gintsts = {0};
	gintsts.inepintr = 1;
	regs->gintsts = gintsts;
}

/*
static void dwc_otg_ep_write_packet(dwc_ep_t *_ep, u32 byte_count, u32 dword_count) {
	u32 i;
	u32 fifo;
	u32 temp_data;
	u8 *data_buff = _ep->xfer_buff;
	u64 t_64 = (u64)_ep->xfer_buff;

	if (_ep->xfer_count >= _ep->xfer_len) {
		ERR("%s() No data for EP%d!!!\n", "dwc_otg_ep_write_packet", _ep->num);
		return;
	}

	fifo = DWC_REG_DATA_FIFO(_ep->num);

	if (t_64 & 0x3) {
		for (i = 0; i < dword_count; i++) {
			temp_data = get_unaligned(data_buff);
			dwc_write_reg32(fifo, temp_data);
			data_buff += 4;
		}
	} else {
		for (i = 0; i < dword_count; i++) {
			temp_data = *(u32*)t_64;
			dwc_write_reg32(fifo, temp_data);
			t_64 += 4;
		}
	}

	_ep->xfer_count += byte_count;
    _ep->xfer_buff += byte_count;

	flush_cpu_cache();

	return;
}
*/

void dwc_handle_outepintr_irq(dwc_usb_t* dwc) {
printf("dwc_handle_outepintr_irq\n");

	uint32_t epnum = 0;

	/* Read in the device interrupt bits */
	uint32_t ep_intr = regs->daint & 0xFFFF0000;
	ep_intr >>= DWC_EP_OUT_SHIFT;

	/* Clear the interrupt */
	dwc_interrupts_t gintsts = {0};
	gintsts.outepintr = 1;
	regs->gintsts = gintsts;
	regs->daint = 0xFFFF0000;

	while (ep_intr) {
		if (ep_intr & 1) {
printf("dwc_handle_outepintr_irq epnum %u\n", epnum);
		    doepint_t doepint = regs->depout[epnum].doepint;

			/* Transfer complete */
			if (doepint.xfercompl) {
printf("dwc_handle_outepintr_irq xfercompl\n");
				/* Clear the bit in DOEPINTn for this interrupt */
			    doepint_t clear = {0};
			    clear.xfercompl = 1;
			    regs->depout[epnum].doepint = clear;

				if (epnum == 0) {
				    clear.val = 0;
				    clear.setup = 1;
    			    regs->depout[epnum].doepint = clear;
					dwc_handle_ep0(dwc);
				} else {
					dwc_complete_ep(dwc, epnum, 0);
				}
			}
			/* Endpoint disable  */
			if (doepint.epdisabled) {
printf("dwc_handle_outepintr_irq epdisabled\n");
				/* Clear the bit in DOEPINTn for this interrupt */
			    doepint_t clear = {0};
                clear.epdisabled = 1;
			    regs->depout[epnum].doepint = clear;
			}
			/* AHB Error */
			if (doepint.ahberr) {
printf("dwc_handle_outepintr_irq ahberr\n");
			    doepint_t clear = {0};
                clear.ahberr = 1;
			    regs->depout[epnum].doepint = clear;
			}
			/* Setup Phase Done (contr0l EPs) */
			if (doepint.setup) {
printf("dwc_handle_outepintr_irq setup\n");
			    doepint_t clear = {0};
                clear.setup = 1;
			    regs->depout[epnum].doepint = clear;
			}
		}
		epnum++;
		ep_intr >>= 1;
	}
}

void dwc_handle_nptxfempty_irq(dwc_usb_t* dwc) {
printf("dwc_handle_nptxfempty_irq\n");

#if 0
	gnptxsts_data_t txstatus = {0};
	gintsts_data_t gintsts = {0};
	dwc_ep_t *ep = 0;
	depctl_data_t depctl;
	u32 len = 0;
	u32 dwords;
	u32 epnum = 0;
	pcd_struct_t *pcd = &gadget_wrapper.pcd;

    /* Get the epnum from the IN Token Learning Queue. */
	for (epnum = 0; epnum < NUM_EP; epnum++) {
		ep = &pcd->dwc_eps[epnum].dwc_ep;

		/* IN endpoint ? */
		if (epnum && !ep->is_in)
			continue;

		if (ep->type == DWC_OTG_EP_TYPE_INTR && ep->xfer_len == 0)
			continue;

		depctl.d32 = dwc_read_reg32(DWC_REG_IN_EP_REG(epnum));
		if (depctl.b.epena != 1)
			continue;

		flush_cpu_cache();

		/* While there is space in the queue and space in the FIFO and
		 * More data to tranfer, Write packets to the Tx FIFO */
		txstatus.d32 = dwc_read_reg32(DWC_REG_GNPTXSTS);
		while  (/*txstatus.b.nptxqspcavail > 0 &&
			txstatus.b.nptxfspcavail > dwords &&*/
			ep->xfer_count < ep->xfer_len) {
				u32 retry = 1000000;

				len = ep->xfer_len - ep->xfer_count;
				if (len > ep->maxpacket)
					len = ep->maxpacket;

				dwords = (len + 3) >> 2;

				while (retry--) {
					txstatus.d32 = dwc_read_reg32(DWC_REG_GNPTXSTS);
					if (txstatus.b.nptxqspcavail > 0 && txstatus.b.nptxfspcavail > dwords)
						break;
					else
						flush_cpu_cache();
				}
				if (0 == retry) {
					printf("TxFIFO FULL: Can't trans data to HOST !\n");
					break;
				}
				/* Write the FIFO */
				dwc_otg_ep_write_packet(ep, len, dwords);

				flush_cpu_cache();
			}

	}
#endif

	/* Clear interrupt */
	dwc_interrupts_t gintsts = {0};
	gintsts.nptxfempty = 1;
	regs->gintsts = gintsts;
}

static void dwc_request_queue(void* ctx, usb_request_t* req) {
/*    dwc_t* dwc = ctx;

    zxlogf(LTRACE, "dwc_request_queue ep: %u\n", req->header.ep_address);
    unsigned ep_num = dwc_ep_num(req->header.ep_address);
    if (ep_num < 2 || ep_num >= countof(dwc->eps)) {
        zxlogf(ERROR, "dwc_request_queue: bad ep address 0x%02X\n", req->header.ep_address);
        usb_request_complete(req, ZX_ERR_INVALID_ARGS, 0);
        return;
    }

    dwc_ep_queue(dwc, ep_num, req);
*/
}

static zx_status_t dwc_set_interface(void* ctx, usb_dci_interface_t* dci_intf) {
    dwc_usb_t* dwc = ctx;
    memcpy(&dwc->dci_intf, dci_intf, sizeof(dwc->dci_intf));
    return ZX_OK;
}

static zx_status_t dwc_config_ep(void* ctx, usb_endpoint_descriptor_t* ep_desc,
                                  usb_ss_ep_comp_descriptor_t* ss_comp_desc) {
//    dwc_usb_t* dwc = ctx;
//    return dwc_ep_config(dwc, ep_desc, ss_comp_desc);
return -1;
}

static zx_status_t dwc_disable_ep(void* ctx, uint8_t ep_addr) {
//    dwc_usb_t* dwc = ctx;
//    return dwc_ep_disable(dwc, ep_addr);
return -1;
}

static zx_status_t dwc_set_stall(void* ctx, uint8_t ep_address) {
//    dwc_usb_t* dwc = ctx;
//    return dwc_ep_set_stall(dwc, dwc_ep_num(ep_address), true);
return -1;
}

static zx_status_t dwc_clear_stall(void* ctx, uint8_t ep_address) {
//    dwc_usb_t* dwc = ctx;
//    return dwc_ep_set_stall(dwc, dwc_ep_num(ep_address), false);
return -1;
}

static zx_status_t dwc_get_bti(void* ctx, zx_handle_t* out_handle) {
    dwc_usb_t* dwc = ctx;
    *out_handle = dwc->bti_handle;
    return ZX_OK;
}

usb_dci_protocol_ops_t dwc_dci_protocol = {
    .request_queue = dwc_request_queue,
    .set_interface = dwc_set_interface,
    .config_ep = dwc_config_ep,
    .disable_ep = dwc_disable_ep,
    .ep_set_stall = dwc_set_stall,
    .ep_clear_stall = dwc_clear_stall,
    .get_bti = dwc_get_bti,
};
