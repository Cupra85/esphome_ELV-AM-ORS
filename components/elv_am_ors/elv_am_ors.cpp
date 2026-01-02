#include "elv_am_ors.h"
#include "esphome/core/log.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace esphome {
namespace elv_am_ors {

static const char *const TAG = "elv_am_ors";

// ================= AS7331 =================
static const uint8_t AS7331_REG_OSR    = 0x00;
static const uint8_t AS7331_REG_MRES1  = 0x02;  // 0x02..0x07 = 3x16-bit
static const uint8_t AS7331_REG_CREG1  = 0x06;
static const uint8_t AS7331_REG_CREG2  = 0x07;
static const uint8_t AS7331_REG_CREG3  = 0x08;

static const uint8_t AS7331_MODE_CFG  = 0x02;
static const uint8_t AS7331_MODE_MEAS = 0x03;

// ================= OPT3001 =================
static const uint8_t OPT3001_REG_RESULT = 0x00;
static const uint8_t OPT3001_REG_CONFIG = 0x01;

// =====================================================================
// AS7331 LSB helper (datasheet-based; LSB in nW/cm² per count)
// TIME scaling: datasheet defines LSB for TIME=10 (1024ms) and scales
// ~proportional with integration time. We scale by 2^(10-time_code).
// =====================================================================
float ELVAMORS::as7331_lsb_nwcm2_(uint8_t gain_code, uint8_t time_code, uint8_t channel_index) const {
  // TIME=10 reference LSB table (nW/cm²) for channels UVA/UVB/UVC
  // Rows: gain_code 0..11 representing 2048x..1x
  static const float lsb_time10[12][3] = {
      {0.16213f, 0.18001f, 0.03849f},  // 2048x
      {0.32425f, 0.36002f, 0.07698f},  // 1024x
      {0.64851f, 0.72004f, 0.15396f},  //  512x
      {1.2970f,  1.4401f,  0.30791f},  //  256x
      {2.5940f,  2.8802f,  0.61582f},  //  128x
      {5.1881f,  5.7604f,  1.2316f },  //   64x
      {10.376f,  11.521f,  2.4633f },  //   32x
      {20.752f,  23.041f,  4.9266f },  //   16x
      {41.505f,  46.083f,  9.8530f },  //    8x
      {83.010f,  92.167f,  19.706f },  //    4x
      {166.02f,  184.33f,  39.412f },  //    2x
      {332.04f,  368.67f,  78.824f },  //    1x
  };

  if (gain_code > 11 || channel_index > 2) return NAN;

  uint8_t t = time_code & 0x0F;
  if (t == 15) t = 0;  // per datasheet: TIME=15 maps to 1ms

  // Scale relative to TIME=10 (1024ms)
  // For TIME < 10 -> integration shorter -> LSB larger -> multiply by 2^(10-t)
  // For TIME > 10 -> integration longer  -> LSB smaller -> divide by 2^(t-10)
  int32_t exp = 10 - static_cast<int32_t>(t);
  float scale = 1.0f;
  if (exp >= 0) {
    scale = static_cast<float>(1UL << exp);
  } else {
    scale = 1.0f / static_cast<float>(1UL << (-exp));
  }

  return lsb_time10[gain_code][channel_index] * scale;
}

// =====================================================================
// SETUP
// =====================================================================
void ELVAMORS::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ELV-AM-ORS");

  this->as7331_ok_ = this->as7331_configure_();
  this->opt3001_ok_ = this->opt3001_configure_();

  if (!this->as7331_ok_) {
    ESP_LOGE(TAG, "AS7331 initialization failed (addr=0x%02X)", this->as7331_address_);
  }
  if (!this->opt3001_ok_) {
    ESP_LOGE(TAG, "OPT3001 initialization failed (addr=0x%02X)", this->opt3001_address_);
  }
}

// =====================================================================
// UPDATE LOOP
// =====================================================================
void ELVAMORS::update() {
  // ---------- UV (AS7331) ----------
  float uva = NAN, uvb = NAN, uvc = NAN;
  if (this->as7331_ok_ && this->as7331_read_uv_(uva, uvb, uvc)) {
    if (this->uva_sensor_) this->uva_sensor_->publish_state(uva);
    if (this->uvb_sensor_) this->uvb_sensor_->publish_state(uvb);
    if (this->uvc_sensor_) this->uvc_sensor_->publish_state(uvc);

    // UV-Index: (vereinfachte, sinnvolle Näherung) -> nur UVB / 0.025
    // (UVI ist erythem-gewichtet; ohne ELV-Faktoren bleibt dies die beste robuste Approx.)
    if (this->uvi_sensor_) {
      float uvi = NAN;
      if (!isnan(uvb)) uvi = uvb / 0.025f;
      this->uvi_sensor_->publish_state(uvi);
    }
  }

  // ---------- Illuminance (OPT3001) ----------
  if (this->opt3001_ok_ && this->illuminance_sensor_) {
    float lux = NAN;
    if (this->opt3001_read_lux_(lux)) {
      this->illuminance_sensor_->publish_state(lux);
    }
  }

  // ---------- Irradiance (analog OUT) ----------
  if (this->irradiance_sensor_) {
    float irr = NAN;
    if (this->read_irradiance_(irr)) {
      this->irradiance_sensor_->publish_state(irr);
    }
  }
}

// =====================================================================
// AS7331 CONFIGURATION
// =====================================================================
bool ELVAMORS::as7331_configure_() {
  this->set_i2c_address(this->as7331_address_);

  // Put device into CFG mode
  uint8_t osr = AS7331_MODE_CFG;
  if (!this->write_register(AS7331_REG_OSR, &osr, 1))
    return false;

  // CREG1: [GAIN(7:4) | TIME(3:0)]
  uint8_t creg1 = (this->as7331_gain_code_ << 4) | (this->as7331_time_code_ & 0x0F);
  if (!this->write_register(AS7331_REG_CREG1, &creg1, 1))
    return false;

  // CREG2: divider disabled, defaults
  uint8_t creg2 = 0x00;
  if (!this->write_register(AS7331_REG_CREG2, &creg2, 1))
    return false;

  // CREG3: CMD measurement mode (bit pattern per datasheet; keep other bits 0)
  uint8_t creg3 = 0x40;
  if (!this->write_register(AS7331_REG_CREG3, &creg3, 1))
    return false;

  return true;
}

// =====================================================================
// AS7331 READ UV (robust measurement sequence)
// =====================================================================
bool ELVAMORS::as7331_read_uv_(float &uva, float &uvb, float &uvc) {
  this->set_i2c_address(this->as7331_address_);

  // Ensure CFG mode before starting a new single-shot
  uint8_t osr_cfg = AS7331_MODE_CFG;
  if (!this->write_register(AS7331_REG_OSR, &osr_cfg, 1))
    return false;

  // Start single-shot measurement: MEAS mode + SS bit
  uint8_t osr_meas = AS7331_MODE_MEAS | 0x80;
  if (!this->write_register(AS7331_REG_OSR, &osr_meas, 1))
    return false;

  // Wait integration time (TIME code -> 1ms * 2^TIME)
  uint8_t t = this->as7331_time_code_ & 0x0F;
  if (t == 15) t = 0;
  uint32_t t_ms = 1UL << t;
  delay(t_ms + 5);

  // Read measurement registers (3x16-bit)
  uint8_t raw[6];
  if (!this->read_register(AS7331_REG_MRES1, raw, 6))
    return false;

  uint16_t r1 = (static_cast<uint16_t>(raw[1]) << 8) | raw[0];
  uint16_t r2 = (static_cast<uint16_t>(raw[3]) << 8) | raw[2];
  uint16_t r3 = (static_cast<uint16_t>(raw[5]) << 8) | raw[4];

  // Convert counts -> nW/cm² using LSB, then to W/m²:
  // 1 nW/cm² = 1e-5 W/m²
  float lsb_uva = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 0);
  float lsb_uvb = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 1);
  float lsb_uvc = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 2);

  uva = static_cast<float>(r1) * lsb_uva * 1e-5f;
  uvb = static_cast<float>(r2) * lsb_uvb * 1e-5f;
  uvc = static_cast<float>(r3) * lsb_uvc * 1e-5f;

  // Return to CFG mode to avoid stale/partial channel behaviour
  if (!this->write_register(AS7331_REG_OSR, &osr_cfg, 1))
    return false;

  return true;
}

// =====================================================================
// OPT3001
// =====================================================================
bool ELVAMORS::opt3001_configure_() {
  this->set_i2c_address(this->opt3001_address_);

  // OPT3001 config: continuous + auto-range + 800ms conversion time (datasheet standard pattern)
  const uint16_t cfg = 0xC610;
  uint8_t buf[3] = {
      OPT3001_REG_CONFIG,
      uint8_t(cfg >> 8),
      uint8_t(cfg & 0xFF),
  };

  if (!this->write(buf, 3))
    return false;

  // One-time warm-up so first result register is valid
  delay(1000);
  return true;
}

bool ELVAMORS::opt3001_read_lux_(float &lux) {
  this->set_i2c_address(this->opt3001_address_);

  uint8_t raw[2];
  if (!this->read_register(OPT3001_REG_RESULT, raw, 2))
    return false;

  uint16_t v = (static_cast<uint16_t>(raw[0]) << 8) | raw[1];
  uint16_t e = (v >> 12) & 0x0F;
  uint16_t m = v & 0x0FFF;

  lux = static_cast<float>(m) * (0.01f * static_cast<float>(1UL << e));
  return true;
}

// =====================================================================
// IRRADIANCE (ANALOG OUT)
// =====================================================================
bool ELVAMORS::read_irradiance_(float &irr) {
#ifdef ARDUINO
  const int raw = analogRead(this->irra_adc_pin_);
  const float v = (static_cast<float>(raw) / static_cast<float>(this->irra_adc_resolution_)) * this->irra_adc_ref_voltage_;

  float val = (v - this->irra_offset_v_) * this->irra_slope_wm2_per_v_;
  if (val < 0.0f) val = 0.0f;

  irr = val;
  return true;
#else
  irr = NAN;
  return false;
#endif
}

}  // namespace elv_am_ors
}  // namespace esphome
