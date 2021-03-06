# Copyright 2017 The Fuchsia Authors
#
# Use of this source code is governed by a MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT

LOCAL_DIR := $(GET_LOCAL_DIR)

PLATFORM_BOARD_NAME := odroidc2
PLATFORM_USE_SHIM := true

include make/board.mk

# prepend Linux header to kernel image

HEADER_TOOL := $(LOCAL_DIR)/prepend-header.py
KERNEL_LOAD_OFFSET := 0x10280000
ODROID_KERNEL_IMAGE := $(BUILDDIR)/odroidc2-zircon.bin
ODROID_BOOT_SHIM := $(BUILDDIR)/odroidc2-boot-shim.bin

$(ODROID_KERNEL_IMAGE): $(HEADER_TOOL) $(KERNEL_ZBI) $(ODROID_BOOT_SHIM)
	$(NOECHO)$(HEADER_TOOL) --kernel $(KERNEL_ZBI) --shim $(ODROID_BOOT_SHIM) --load_offset $(KERNEL_LOAD_OFFSET) --output $@

EXTRA_KERNELDEPS += $(ODROID_KERNEL_IMAGE)
