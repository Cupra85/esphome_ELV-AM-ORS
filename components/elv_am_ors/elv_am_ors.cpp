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

    // UV-Index: physikalisch korrekt -> nur UVB
    if (this->uvi_sensor_) {
      float uvi = NAN;
      if (!isnan(uvb)) {
        uvi = uvb / 0.025f;  // 25 mW/m²
      }
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

  // CFG mode
  uint8_t osr = AS7331_MODE_CFG;
  if (!this->write_register(AS7331_REG_OSR, &osr, 1))
    return false;

  // Gain + Integration Time
  uint8_t creg1 = (this->as7331_gain_code_ << 4) |
                  (this->as7331_time_code_ & 0x0F);
  if (!this->write_register(AS7331_REG_CREG1, &creg1, 1))
    return false;

  // Divider disabled
  uint8_t creg2 = 0x00;
  if (!this->write_register(AS7331_REG_CREG2, &creg2, 1))
    return false;

  // CMD measurement mode
  uint8_t creg3 = 0x40;
  if (!this->write_register(AS7331_REG_CREG3, &creg3, 1))
    return false;

  return true;
}

// =====================================================================
// AS7331 READ UV (CORRECT MEASUREMENT SEQUENCE)
// =====================================================================
bool ELVAMORS::as7331_read_uv_(float &uva, float &uvb, float &uvc) {
  this->set_i2c_address(this->as7331_address_);

  // 1. Back to CFG
  uint8_t osr_cfg = AS7331_MODE_CFG;
  if (!this->write_register(AS7331_REG_OSR, &osr_cfg, 1))
    return false;

  // 2. Start single-shot measurement
  uint8_t osr_meas = AS7331_MODE_MEAS | 0x80;  // SS=1
  if (!this->write_register(AS7331_REG_OSR, &osr_meas, 1))
    return false;

  // 3. Wait integration time
  uint8_t t = this->as7331_time_code_ & 0x0F;
  if (t == 15) t = 0;
  delay((1UL << t) + 5);

  // 4. Read raw values
  uint8_t raw[6];
  if (!this->read_register(AS7331_REG_MRES1, raw, 6))
    return false;

  uint16_t r1 = (raw[1] << 8) | raw[0];
  uint16_t r2 = (raw[3] << 8) | raw[2];
  uint16_t r3 = (raw[5] << 8) | raw[4];

  // 5. Convert using datasheet LSB (nW/cm² → W/m²)
  float lsb_uva = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 0);
  float lsb_uvb = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 1);
  float lsb_uvc = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 2);

  uva = r1 * lsb_uva * 1e-5f;
  uvb = r2 * lsb_uvb * 1e-5f;
  uvc = r3 * lsb_uvc * 1e-5f;

  // 6. Back to CFG (IMPORTANT)
  if (!this->write_register(AS7331_REG_OSR, &osr_cfg, 1))
    return false;

  return true;
}

// =====================================================================
// OPT3001
// =====================================================================
bool ELVAMORS::opt3001_configure_() {
  this->set_i2c_address(this->opt3001_address_);

  const uint16_t cfg = 0xC610;  // Continuous, auto-range, 800 ms
  uint8_t buf[3] = {
    OPT3001_REG_CONFIG,
    uint8_t(cfg >> 8),
    uint8_t(cfg & 0xFF),
  };

  if (!this->write(buf, 3))
    return false;

  // One-time warm-up
  delay(1000);
  return true;
}

bool ELVAMORS::opt3001_read_lux_(float &lux) {
  this->set_i2c_address(this->opt3001_address_);

  uint8_t raw[2];
  if (!this->read_register(OPT3001_REG_RESULT, raw, 2))
    return false;

  uint16_t v = (raw[0] << 8) | raw[1];
  uint16_t e = (v >> 12) & 0x0F;
  uint16_t m = v & 0x0FFF;

  lux = m * (0.01f * (1UL << e));
  return true;
}

// =====================================================================
// IRRADIANCE (ANALOG OUT)
// =====================================================================
bool ELVAMORS::read_irradiance_(float &irr) {
#ifdef ARDUINO
  int raw = analogRead(this->irra_adc_pin_);
  float v = (float(raw) / this->irra_adc_resolution_) * this->irra_adc_ref_voltage_;

  irr = (v - this->irra_offset_v_) * this->irra_slope_wm2_per_v_;
  if (irr < 0.0f) irr = 0.0f;

  return true;
#else
  return false;
#endif
}

}  // namespace elv_am_ors
}  // namespace esphome
