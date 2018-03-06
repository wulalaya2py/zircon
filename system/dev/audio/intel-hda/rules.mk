# Copyright 2017 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULES += \
    $(LOCAL_DIR)/controller \
    $(LOCAL_DIR)/codecs/qemu \
    $(LOCAL_DIR)/codecs/realtek \

ADSP_FIRMWARE_NAME := dsp_fw_kbl_v3266.bin
ADSP_FIRMWARE_BIN := $(LOCAL_DIR)/firmware/$(ADSP_FIRMWARE_NAME)

ADSP_TPLG_NAME := 9d71-GOOGLE-EVEMAX-0-tplg.bin
ADSP_TPLG_BIN := $(LOCAL_DIR)/firmware/$(ADSP_TPLG_NAME)

ADSP_I2S_CFG_NAME := max98927-render-2ch-48khz-24b.bin
ADSP_I2S_CFG_BIN := $(LOCAL_DIR)/firmware/$(ADSP_I2S_CFG_NAME)

USER_MANIFEST_LINES += {core}lib/firmware/$(ADSP_FIRMWARE_NAME)=$(ADSP_FIRMWARE_BIN)
USER_MANIFEST_LINES += {core}lib/firmware/$(ADSP_TPLG_NAME)=$(ADSP_TPLG_BIN)
USER_MANIFEST_LINES += {core}lib/firmware/$(ADSP_I2S_CFG_NAME)=$(ADSP_I2S_CFG_BIN)
