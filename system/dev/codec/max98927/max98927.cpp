// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>
#include <zircon/device/i2c.h>
#include <zircon/assert.h>

#include <fbl/alloc_checker.h>

#include "max98927.h"
#include "max98927-registers.h"

namespace audio {
namespace max98927 {

uint8_t MAX98927Device::ReadReg(uint16_t addr) {
    uint8_t val = 0;

    // segments followed by write data (address)
    size_t segments_size = sizeof(i2c_slave_ioctl_segment_t) * 3;
    uint8_t buf[segments_size + sizeof(addr)];
    buf[segments_size] = (uint8_t)(addr >> 8);
    buf[segments_size + 1] = (uint8_t)(addr & 0xFF);

    i2c_slave_ioctl_segment_t* segments = (i2c_slave_ioctl_segment_t*)buf;
    segments[0].type = I2C_SEGMENT_TYPE_WRITE;
    segments[0].len = sizeof(addr);
    segments[1].type = I2C_SEGMENT_TYPE_READ;
    segments[1].len = sizeof(val);
    segments[2].type = I2C_SEGMENT_TYPE_END;
    segments[2].len = 0;

    size_t actual = 0;
    zx_status_t st = device_ioctl(parent(), IOCTL_I2C_SLAVE_TRANSFER, buf, sizeof(buf),
                                  &val, sizeof(val), &actual);
    if (st != ZX_OK) {
        zxlogf(ERROR, "max98927: register 0x%04x read failed (err %d)\n", addr, st);
        ZX_DEBUG_ASSERT(false);
        return 0;
    }
    if (actual != sizeof(val)) {
        zxlogf(ERROR, "max98927: register 0x%04x read unexpected length (got %zu, expected %zu)\n",
                      addr, actual, sizeof(val));
        ZX_DEBUG_ASSERT(false);
        return 0;
    }

    zxlogf(SPEW, "max98927: register 0x%04x read 0x%02x\n", addr, val);

    return val;
}

void MAX98927Device::WriteReg(uint16_t addr, uint8_t val) {
    // segments followed by write data (address and val)
    size_t segments_size = sizeof(i2c_slave_ioctl_segment_t) * 2;
    uint8_t buf[segments_size + sizeof(addr) + sizeof(val)];
    buf[segments_size] = (uint8_t)(addr >> 8);
    buf[segments_size + 1] = (uint8_t)(addr & 0xFF);
    buf[segments_size + 2] = val;

    i2c_slave_ioctl_segment_t* segments = (i2c_slave_ioctl_segment_t*)buf;
    segments[0].type = I2C_SEGMENT_TYPE_WRITE;
    segments[0].len = sizeof(addr) + sizeof(val);
    segments[1].type = I2C_SEGMENT_TYPE_END;
    segments[1].len = 0;

    size_t actual = 0;
    zx_status_t st = device_ioctl(parent(), IOCTL_I2C_SLAVE_TRANSFER, buf, sizeof(buf),
                                  NULL, 0, &actual);
    if (st != ZX_OK) {
        zxlogf(ERROR, "max98927: register 0x%04x write failed (err %d)\n", addr, st);
        ZX_DEBUG_ASSERT(false);
    }
    ZX_DEBUG_ASSERT(actual == 0);

    zxlogf(SPEW, "max98927: register 0x%04x write0x%02x\n", addr, val);
}

void MAX98927Device::DdkUnbind() {
}

void MAX98927Device::DdkRelease() {
    delete this;
}

void MAX98927Device::Test() {
    // PCM config - slave mode
    WriteReg(PCM_MASTER_MODE, 0);

    // PCM config - 48kHz 16-bits
    WriteReg(PCM_SAMPLE_RATE_SETUP_1, PCM_SAMPLE_RATE_SETUP_1_DIG_IF_SR(0x8));
    WriteReg(PCM_SAMPLE_RATE_SETUP_2, PCM_SAMPLE_RATE_SETUP_2_SPK_SR(0x8) |
                                      PCM_SAMPLE_RATE_SETUP_2_IVADC_SR(0x8));
    WriteReg(PCM_MODE_CFG, PCM_MODE_CFG_CHANSZ_16BITS | 0x3);
    WriteReg(PCM_CLOCK_SETUP, 0x2);

    // Enable TX channels
    WriteReg(PCM_RX_EN_A, 0x3);

    // Set speaker source to tone generator
    WriteReg(SPK_SRC_SEL, SPK_SRC_SEL_TONE_GEN);

    // Generate a tone. Must do after AMP_ENABLE_EN and AMP_DSP_EN.
    WriteReg(TONE_GEN_DC_CFG, 0x6); // fs/64 @ 48kHz = 750Hz

    zxlogf(INFO, "max98927: playing test tone...\n");

    // Enable for 2 secs. Gloabl first then amplifier for click-and-pop performance.
    WriteReg(AMP_ENABLE, AMP_ENABLE_EN);
    WriteReg(GLOBAL_ENABLE, GLOBAL_ENABLE_EN);

    zx_nanosleep(zx_deadline_after(ZX_SEC(2)));

    WriteReg(GLOBAL_ENABLE, 0);
    WriteReg(AMP_ENABLE, 0);

    zxlogf(INFO, "max98927: test tone done\n");
}

zx_status_t MAX98927Device::Initialize() {
    // Reset device
    WriteReg(SOFTWARE_RESET, SOFTWARE_RESET_RST);

    // Set outputs to HiZ
    WriteReg(PCM_TX_HIZ_CTRL_A, 0xFF);
    WriteReg(PCM_TX_HIZ_CTRL_B, 0xFF);

    // Default monomix output is (channel 0 + channel 1) / 2
    // Default monomix input channel 0 is PCM RX channel 0
    WriteReg(PCM_SPK_MONOMIX_A, PCM_SPK_MONOMIX_A_CFG_OUTPUT_0_1 |
                                PCM_SPK_MONOMIX_B_CFG_CH0_SRC(0));
    // Default monomix input channel 1 is PCM RX channel 1
    WriteReg(PCM_SPK_MONOMIX_B, PCM_SPK_MONOMIX_B_CFG_CH1_SRC(1));

    // Default volume (+13dB)
    WriteReg(AMP_VOL_CTRL, 0x38);
    WriteReg(SPK_GAIN, SPK_GAIN_PCM(SPK_GAIN_15DB));

    // Enable DC blocking filter
    WriteReg(AMP_DSP_CFG, AMP_DSP_CFG_DCBLK_EN);

    // Enable IMON/VMON DC blocker
    WriteReg(MEAS_DSP_CFG, MEAS_DSP_CFG_I_DCBLK(MEAS_DSP_CFG_FREQ_3_7HZ) |
                           MEAS_DSP_CFG_V_DCBLK(MEAS_DSP_CFG_FREQ_3_7HZ) |
                           MEAS_DSP_CFG_DITH_EN |
                           MEAS_DSP_CFG_I_DCBLK_EN |
                           MEAS_DSP_CFG_V_DCBLK_EN);

    // Boost output voltage & current limit
    WriteReg(BOOST_CTRL_0, 0x1C); // 10.00V
    WriteReg(BOOST_CTRL_1, 0x3E); // 4.00A

    // Measurement ADC config
    WriteReg(MEAS_ADC_CFG, MEAS_ADC_CFG_CH2_EN);
    WriteReg(MEAS_ADC_BASE_DIV_MSB, 0);
    WriteReg(MEAS_ADC_BASE_DIV_LSB, 0x24);

    // Brownout level
    WriteReg(BROWNOUT_LVL4_AMP1_CTRL1, 0x06); // -6dBFS

    // Envelope tracker configuration
    WriteReg(ENV_TRACKER_VOUT_HEADROOM, 0x08); // 1.000V
    WriteReg(ENV_TRACKER_CTRL, ENV_TRACKER_CTRL_EN);
    WriteReg(ENV_TRACKER_BOOST_VOUT_RB, 0x10); // 8.500V

    // TODO: figure out vmon-slot-no and imon-slot-no

    // Set interleave mode
    WriteReg(PCM_TX_CH_SRC_B, PCM_TX_CH_SRC_B_INTERLEAVE);

    return ZX_OK;
}

zx_status_t MAX98927Device::Bind() {
    zx_status_t st = Initialize();
    if (st != ZX_OK) {
        return st;
    }

    return DdkAdd("max98927");
}

zx_status_t MAX98927Device::Create(zx_device_t* parent, fbl::unique_ptr<MAX98927Device>* out) {
    fbl::AllocChecker ac;
    fbl::unique_ptr<MAX98927Device> ret(new (&ac) MAX98927Device(parent));
    if (!ac.check()) {
        zxlogf(ERROR, "max98927: out of memory attempting to allocate device\n");
        return ZX_ERR_NO_MEMORY;
    }

    *out = fbl::move(ret);
    return ZX_OK;
}

}  // namespace max98927
}  // namespace audio

extern "C" {
zx_status_t max98927_bind_hook(void* ctx, zx_device_t* parent) {
    fbl::unique_ptr<audio::max98927::MAX98927Device> dev;
    zx_status_t st = audio::max98927::MAX98927Device::Create(parent, &dev);
    if (st != ZX_OK) {
        return st;
    }

    st = dev->Bind();
    if (st == ZX_OK) {
        // devmgr is now in charge of the memory for dev
        __UNUSED auto ptr = dev.release();
        return st;
    }

    return ZX_OK;
}
}  // extern "C"
