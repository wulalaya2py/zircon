# Copyright 2018 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_TYPE := userapp
MODULE_GROUP := misc

MODULE_SRCS += \
    $(LOCAL_DIR)/sstest.cpp

MODULE_NAME := sstest

MODULE_STATIC_LIBS := \
    system/ulib/trace \
    system/ulib/async \
    system/ulib/async-loop \
    system/ulib/zxcpp \
    system/ulib/fbl \
    system/ulib/zx

MODULE_LIBS := \
    system/ulib/async.default \
    system/ulib/c \
    system/ulib/fdio \
    system/ulib/zircon \
    system/ulib/launchpad

include make/module.mk

