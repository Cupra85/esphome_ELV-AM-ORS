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
static const uint8_t AS7331_REG_MRES1  = 0x02;
static const uint8_t AS7331_REG_CREG1  = 0x06;
static const uint8_t AS7331_REG_CREG2  = 0x07;
static const uint8_t AS7331_REG_CREG3  = 0x08;

static const uint8_t AS7331_MODE_CFG  = 0x02;
static const uint8_t AS7331_MODE_MEAS = 0x03;

// ================= OPT3001 =================
static const uint8_t OPT3001_REG_RESULT = 0x00;
static const uint8_t OPT3001_REG_CONFIG = 0x01;

void ELVAMORS::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ELV-AM-ORS");

  this->as7331_ok_ = this->as7331_configure_();
  this->opt3001_ok_ = this->opt3001_configure_();

  if (!this->as7331_ok_) {
    ESP_LOGW(TAG, "AS7331 init failed (0x%02X)", this->as7331_address_);
  }
  if (!this->opt3001_ok_) {
    ESP_LOGW(TAG, "OPT3001 init failed (0x%02X)", this->opt3001_address_);
  }
}

void ELVAMORS::dump_config() {
  ESP_LOGCONFIG(TAG, "ELV-AM-ORS:");
  ESP_LOGCONFIG(TAG, "  AS7331 address: 0x%02X", this->as7331_address_);
  ESP_LOGCONFIG(TAG, "  OPT3001 address: 0x%02X", this->opt3001_address_);
  LOG_I2C_DEVICE(this);
}

void ELVAMORS::update() {
  // ---------- UV (AS7331) ----------
  if (this->as7331_ok_) {
    float uva, uvb, uvc;
    if (this->as7331_read_uv_(uva, uvb, uvc)) {
      if (this->uva_sensor_) this->uva_sensor_->publish_state(uva);
      if (this->uvb_sensor_) this->uvb_sensor_->publish_state(uvb);
      if (this->uvc_sensor_) this->uvc_sensor_->publish_state(uvc);

      if (this->uvi_sensor_) {
        this->uvi_sensor_->publish_state((uva + uvb + uvc) / 0.025f);
      }
    }
  }

  // ---------- Illuminance ----------
  if (this->opt3001_ok_ && this->illuminance_sensor_) {
    float lux;
    if (this->opt3001_read_lux_(lux)) {
      this->illuminance_sensor_->publish_state(lux);
    }
  }

  // ---------- Irradiance ----------
  if (this->irradiance_sensor_) {
    float irr;
    if (this->read_irradiance_(irr)) {
      this->irradiance_sensor_->publish_state(irr);
    }
  }
}

bool ELVAMORS::as7331_configure_() {
  this->set_i2c_address(this->as7331_address_);

  uint8_t osr = AS7331_MODE_CFG;
  if (!this->write_register(AS7331_REG_OSR, &osr, 1))
    return false;

  uint8_t creg1 = (this->as7331_gain_code_ << 4) | (this->as7331_time_code_ & 0x0F);
  if (!this->write_register(AS7331_REG_CREG1, &creg1, 1))
    return false;

  uint8_t creg2 = 0x00;
  if (!this->write_register(AS7331_REG_CREG2, &creg2, 1))
    return false;

  uint8_t creg3 = 0x40;  // CMD mode
  if (!this->write_register(AS7331_REG_CREG3, &creg3, 1))
    return false;

  return true;
}

bool ELVAMORS::as7331_read_uv_(float &uva, float &uvb, float &uvc) {
  this->set_i2c_address(this->as7331_address_);

  uint8_t start = AS7331_MODE_MEAS | 0x80;
  if (!this->write_register(AS7331_REG_OSR, &start, 1))
    return false;

  delay((1UL << this->as7331_time_code_) + 5);

  uint8_t raw[6];
  if (!this->read_register(AS7331_REG_MRES1, raw, 6))
    return false;

  uint16_t r1 = (raw[1] << 8) | raw[0];
  uint16_t r2 = (raw[3] << 8) | raw[2];
  uint16_t r3 = (raw[5] << 8) | raw[4];

  // *** KORREKTE SKALIERUNG ***
  uva = r1 * 1e-5f;
  uvb = r2 * 1e-5f;
  uvc = r3 * 1e-5f;

  return true;
}

bool ELVAMORS::opt3001_configure_() {
  this->set_i2c_address(this->opt3001_address_);

  const uint16_t cfg = 0xC610;  // continuous, auto-range, 800 ms
  uint8_t buf[3] = {
    OPT3001_REG_CONFIG,
    uint8_t(cfg >> 8),
    uint8_t(cfg & 0xFF),
  };

  return this->write(buf, 3);
}

bool ELVAMORS::opt3001_read_lux_(float &lux) {
  this->set_i2c_address(this->opt3001_address_);

  uint8_t raw[2];
  if (!this->read_register(OPT3001_REG_RESULT, raw, 2))
    return false;

  uint16_t v = (raw[0] << 8) | raw[1];
  uint16_t e = (v >> 12) & 0x0F;
  uint16_t m = v & 0x0FFF;

  lux = m * (0.01f * (1 << e));
  return true;
}

bool ELVAMORS::read_irradiance_(float &irr) {
#ifdef ARDUINO
  int raw = analogRead(this->irra_adc_pin_);
  float v = (float(raw) / this->irra_adc_resolution_) * this->irra_adc_ref_voltage_;

  irr = (v - this->irra_offset_v_) * this->irra_slope_wm2_per_v_;
  if (irr < 0) irr = 0;
  return true;
#else
  return false;
#endif
}

}  // namespace elv_am_ors
}  // namespace esphome
