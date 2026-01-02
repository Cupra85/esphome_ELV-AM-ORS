#include "elv_am_ors.h"
#include "esphome/core/log.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace esphome {
namespace elv_am_ors {

static const char *const TAG = "elv_am_ors";

// AS7331 register addresses
static const uint8_t AS7331_REG_OSR = 0x00;
static const uint8_t AS7331_REG_CREG1 = 0x06;
static const uint8_t AS7331_REG_CREG2 = 0x07;
static const uint8_t AS7331_REG_CREG3 = 0x08;

// AS7331 measurement register base
static const uint8_t AS7331_REG_MRES1 = 0x02;

// AS7331 op state (OSR.DOS)
static const uint8_t AS7331_DEVICE_MODE_CFG = 0x02;
static const uint8_t AS7331_DEVICE_MODE_MEAS = 0x03;

// AS7331 measurement mode (CREG3.MMODE): CONT=0, CMD=1, SYNS=2, SYND=3
static const uint8_t AS7331_MEAS_MODE_CMD = 0x01;

// OPT3001 register addresses
static const uint8_t OPT3001_REG_RESULT = 0x00;
static const uint8_t OPT3001_REG_CONFIG = 0x01;

void ELVAMORS::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ELV-AM-ORS ...");

  this->as7331_ok_ = this->as7331_configure_();
  this->opt3001_ok_ = this->opt3001_configure_();

  if (!this->as7331_ok_) {
    ESP_LOGW(TAG, "AS7331 init failed (I2C addr 0x%02X). Check wiring/address jumpers.", this->as7331_address_);
  }
  if (!this->opt3001_ok_) {
    ESP_LOGW(TAG, "OPT3001 init failed (I2C addr 0x%02X). Check wiring/address jumpers.", this->opt3001_address_);
  }
}

void ELVAMORS::dump_config() {
  ESP_LOGCONFIG(TAG, "ELV-AM-ORS:");
  ESP_LOGCONFIG(TAG, "  I2C addresses: AS7331=0x%02X, OPT3001=0x%02X", this->as7331_address_, this->opt3001_address_);
  ESP_LOGCONFIG(TAG, "  AS7331 gain_code=%u (0=2048x..11=1x), time_code=%u (0=1ms..10=1024ms)",
                this->as7331_gain_code_, this->as7331_time_code_);
  ESP_LOGCONFIG(TAG, "  IRRA ADC pin=%u, ref=%.3f V, resolution=%u, slope=%.3f W/m²/V, offset=%.3f V",
                this->irra_adc_pin_, this->irra_adc_ref_voltage_, this->irra_adc_resolution_,
                this->irra_slope_wm2_per_v_, this->irra_offset_v_);
  LOG_I2C_DEVICE(this);
}

void ELVAMORS::update() {
  // UV (AS7331)
  if (this->as7331_ok_) {
    float uva = NAN, uvb = NAN, uvc = NAN;
    if (this->as7331_read_uv_(uva, uvb, uvc)) {
      if (this->uva_sensor_ != nullptr) this->uva_sensor_->publish_state(uva);
      if (this->uvb_sensor_ != nullptr) this->uvb_sensor_->publish_state(uvb);
      if (this->uvc_sensor_ != nullptr) this->uvc_sensor_->publish_state(uvc);

      // UVI per ELV: UVI = E_UV / 0.025 W/m²
      if (this->uvi_sensor_ != nullptr) {
        const float e_uv = uva + uvb + uvc;
        float uvi = NAN;
        if (!isnan(e_uv)) uvi = e_uv / 0.025f;
        this->uvi_sensor_->publish_state(uvi);
      }
    } else {
      ESP_LOGW(TAG, "AS7331 read failed");
    }
  }

  // Illuminance (OPT3001)
  if (this->opt3001_ok_ && this->illuminance_sensor_ != nullptr) {
    float lux = NAN;
    if (this->opt3001_read_lux_(lux)) {
      this->illuminance_sensor_->publish_state(lux);
    } else {
      ESP_LOGW(TAG, "OPT3001 read failed");
    }
  }

  // Irradiance (analog OUT)
  if (this->irradiance_sensor_ != nullptr) {
    float irr = NAN;
    if (this->read_irradiance_(irr)) {
      this->irradiance_sensor_->publish_state(irr);
    } else {
      ESP_LOGW(TAG, "Irradiance ADC read failed");
    }
  }
}

bool ELVAMORS::as7331_configure_() {
  this->set_address(this->as7331_address_);

  uint8_t osr_cfg = (AS7331_DEVICE_MODE_CFG & 0x07);
  if (!this->write_register(AS7331_REG_OSR, &osr_cfg, 1)) return false;

  uint8_t time_code = this->as7331_time_code_ & 0x0F;
  uint8_t gain_code = this->as7331_gain_code_ & 0x0F;
  uint8_t creg1 = (gain_code << 4) | time_code;
  if (!this->write_register(AS7331_REG_CREG1, &creg1, 1)) return false;

  uint8_t creg2 = 0x00;  // divider disabled
  if (!this->write_register(AS7331_REG_CREG2, &creg2, 1)) return false;

  uint8_t creg3 = 0x00;
  creg3 |= (0x0 & 0x03);                 // CCLK=0
  creg3 |= (AS7331_MEAS_MODE_CMD << 6);  // MMODE=CMD
  if (!this->write_register(AS7331_REG_CREG3, &creg3, 1)) return false;

  return true;
}

float ELVAMORS::as7331_lsb_nwcm2_(uint8_t gain_code, uint8_t time_code, uint8_t channel_index) const {
  // LSB values for TIME=10 (1024ms), CCLK=0, divider disabled.
  // From AS7331 datasheet Figure 48 (LSB in nW/cm²).
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
  if (t == 15) t = 0;

  int32_t exp = 10 - static_cast<int32_t>(t);
  float scale = 1.0f;
  if (exp >= 0) scale = static_cast<float>(1UL << exp);
  else scale = 1.0f / static_cast<float>(1UL << (-exp));

  return lsb_time10[gain_code][channel_index] * scale;
}

bool ELVAMORS::as7331_read_uv_(float &uva_w_m2, float &uvb_w_m2, float &uvc_w_m2) {
  this->set_address(this->as7331_address_);

  if (!this->as7331_configure_()) return false;

  uint8_t osr_meas_start = (AS7331_DEVICE_MODE_MEAS & 0x07) | 0x80;  // SS=1
  if (!this->write_register(AS7331_REG_OSR, &osr_meas_start, 1)) return false;

  uint8_t t = this->as7331_time_code_ & 0x0F;
  if (t == 15) t = 0;
  uint32_t t_ms = 1UL << t;
  delay(static_cast<uint32_t>(t_ms + 5));

  uint8_t raw[6];
  if (!this->read_register(AS7331_REG_MRES1, raw, sizeof(raw))) return false;

  const uint16_t mres1 = (static_cast<uint16_t>(raw[1]) << 8) | raw[0];
  const uint16_t mres2 = (static_cast<uint16_t>(raw[3]) << 8) | raw[2];
  const uint16_t mres3 = (static_cast<uint16_t>(raw[5]) << 8) | raw[4];

  const float lsb_uva = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 0);
  const float lsb_uvb = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 1);
  const float lsb_uvc = this->as7331_lsb_nwcm2_(this->as7331_gain_code_, this->as7331_time_code_, 2);

  uva_w_m2 = static_cast<float>(mres1) * lsb_uva * 1e-5f;
  uvb_w_m2 = static_cast<float>(mres2) * lsb_uvb * 1e-5f;
  uvc_w_m2 = static_cast<float>(mres3) * lsb_uvc * 1e-5f;

  return true;
}

bool ELVAMORS::opt3001_configure_() {
  this->set_address(this->opt3001_address_);

  const uint16_t config = 0xC610;  // continuous + auto-range + 800ms
  uint8_t buf[3];
  buf[0] = OPT3001_REG_CONFIG;
  buf[1] = (config >> 8) & 0xFF;
  buf[2] = config & 0xFF;
  return this->write(buf, 3);
}

bool ELVAMORS::opt3001_read_lux_(float &lux) {
  this->set_address(this->opt3001_address_);

  uint8_t raw[2];
  if (!this->read_register(OPT3001_REG_RESULT, raw, 2)) return false;

  uint16_t v = (static_cast<uint16_t>(raw[0]) << 8) | raw[1];
  uint16_t exponent = (v >> 12) & 0x0F;
  uint16_t mantissa = v & 0x0FFF;

  lux = static_cast<float>(mantissa) * (0.01f * static_cast<float>(1UL << exponent));
  return true;
}

bool ELVAMORS::read_irradiance_(float &irradiance_w_m2) {
#ifdef ARDUINO
  const int raw = analogRead(this->irra_adc_pin_);
  const float v = (static_cast<float>(raw) / static_cast<float>(this->irra_adc_resolution_)) * this->irra_adc_ref_voltage_;

  float irr = (v - this->irra_offset_v_) * this->irra_slope_wm2_per_v_;
  if (irr < 0.0f) irr = 0.0f;

  irradiance_w_m2 = irr;
  return true;
#else
  irradiance_w_m2 = NAN;
  return false;
#endif
}

}  // namespace elv_am_ors
}  // namespace esphome
