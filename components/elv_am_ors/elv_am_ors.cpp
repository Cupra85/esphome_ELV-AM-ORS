#include "elv_am_ors.h"
#include "esphome/core/log.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace esphome {
namespace elv_am_ors {

static const char *const TAG = "elv_am_ors";

// ================= AS7331 REGISTERS =================
static const uint8_t AS7331_REG_OSR    = 0x00;
static const uint8_t AS7331_REG_MRES1  = 0x02;  // UVA LSB
static const uint8_t AS7331_REG_CREG1  = 0x06;
static const uint8_t AS7331_REG_CREG2  = 0x07;
static const uint8_t AS7331_REG_CREG3  = 0x08;

static const uint8_t AS7331_MODE_CFG  = 0x02;
static const uint8_t AS7331_MODE_MEAS = 0x03;

// ================= OPT3001 REGISTERS =================
static const uint8_t OPT3001_REG_RESULT = 0x00;
static const uint8_t OPT3001_REG_CONFIG = 0x01;

// =====================================================================
// AS7331 LSB TABLE (from datasheet, TIME=10 reference, nW/cm²)
// =====================================================================
static const float AS7331_LSB_TIME10[12][3] = {
    {0.16213f, 0.18001f, 0.03849f},  // 2048x
    {0.32425f, 0.36002f, 0.07698f},
    {0.64851f, 0.72004f, 0.15396f},
    {1.2970f,  1.4401f,  0.30791f},
    {2.5940f,  2.8802f,  0.61582f},
    {5.1881f,  5.7604f,  1.2316f},
    {10.376f,  11.521f,  2.4633f},
    {20.752f,  23.041f,  4.9266f},
    {41.505f,  46.083f,  9.8530f},
    {83.010f,  92.167f,  19.706f},
    {166.02f,  184.33f,  39.412f},
    {332.04f,  368.67f,  78.824f},
};

float ELVAMORS::as7331_lsb_nwcm2_(uint8_t gain, uint8_t time, uint8_t ch) const {
  if (gain > 11 || ch > 2) return NAN;
  uint8_t t = time & 0x0F;
  if (t == 15) t = 0;

  int exp = 10 - t;
  float scale = (exp >= 0) ? float(1UL << exp) : 1.0f / float(1UL << (-exp));
  return AS7331_LSB_TIME10[gain][ch] * scale;
}

// =====================================================================
// SETUP
// =====================================================================
void ELVAMORS::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ELV-AM-ORS (continuous mode)");

  this->set_i2c_address(this->as7331_address_);

  // --- AS7331 CONFIGURATION ---
  uint8_t osr = AS7331_MODE_CFG;
  this->write_register(AS7331_REG_OSR, &osr, 1);

  uint8_t creg1 = (this->as7331_gain_code_ << 4) | (this->as7331_time_code_ & 0x0F);
  this->write_register(AS7331_REG_CREG1, &creg1, 1);

  uint8_t creg2 = 0x00;
  this->write_register(AS7331_REG_CREG2, &creg2, 1);

  // CONTINUOUS MODE (no CMD, no SYN, no READY)
  uint8_t creg3 = 0x00;
  this->write_register(AS7331_REG_CREG3, &creg3, 1);

  // Start continuous measurement
  osr = AS7331_MODE_MEAS;
  this->write_register(AS7331_REG_OSR, &osr, 1);

  this->as7331_ok_ = true;

  // --- OPT3001 CONFIGURATION ---
  this->set_i2c_address(this->opt3001_address_);
  const uint16_t cfg = 0xC610;  // continuous, auto-range, 800ms
  uint8_t buf[3] = {
      OPT3001_REG_CONFIG,
      uint8_t(cfg >> 8),
      uint8_t(cfg & 0xFF),
  };

  if (this->write(buf, 3)) {
    delay(1000);  // warm-up
    this->opt3001_ok_ = true;
  } else {
    this->opt3001_ok_ = false;
  }
}

// =====================================================================
// UPDATE LOOP
// =====================================================================
void ELVAMORS::update() {
  // ================= UV (AS7331) =================
  if (this->as7331_ok_) {
    this->set_i2c_address(this->as7331_address_);

    uint8_t raw[6];
    if (this->read_register(AS7331_REG_MRES1, raw, 6)) {
      uint16_t r_uva = (raw[1] << 8) | raw[0];
      uint16_t r_uvb = (raw[3] << 8) | raw[2];
      uint16_t r_uvc = (raw[5] << 8) | raw[4];

      float uva = r_uva * this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 0) * 1e-5f;
      float uvb = r_uvb * this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 1) * 1e-5f;
      float uvc = r_uvc * this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 2) * 1e-5f;

      if (this->uva_sensor_) this->uva_sensor_->publish_state(uva);
      if (this->uvb_sensor_) this->uvb_sensor_->publish_state(uvb);
      if (this->uvc_sensor_) this->uvc_sensor_->publish_state(uvc);

      // UV-Index per ELV-Handbuch: UVB / 25 mW/m²
      if (this->uvi_sensor_) {
        this->uvi_sensor_->publish_state(uvb / 0.025f);
      }
    }
  }

  // ================= LUX (OPT3001) =================
  if (this->opt3001_ok_ && this->illuminance_sensor_) {
    this->set_i2c_address(this->opt3001_address_);

    uint8_t raw[2];
    if (this->read_register(OPT3001_REG_RESULT, raw, 2)) {
      uint16_t v = (raw[0] << 8) | raw[1];
      uint16_t e = (v >> 12) & 0x0F;
      uint16_t m = v & 0x0FFF;
      float lux = m * (0.01f * (1UL << e));
      this->illuminance_sensor_->publish_state(lux);
    }
  }

  // ================= IRRADIANCE (ADC) =================
#ifdef ARDUINO
  if (this->irradiance_sensor_) {
    int raw = analogRead(this->irra_adc_pin_);
    float v = (float(raw) / this->irra_adc_resolution_) * this->irra_adc_ref_voltage_;
    float irr = (v - this->irra_offset_v_) * this->irra_slope_wm2_per_v_;
    if (irr < 0) irr = 0;
    this->irradiance_sensor_->publish_state(irr);
  }
#endif
}

}  // namespace elv_am_ors
}  // namespace esphome
