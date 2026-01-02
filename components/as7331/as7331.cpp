#include "as7331.h"
#include "esphome/core/log.h"

namespace esphome {
namespace as7331 {

static const char *const TAG = "as7331";

// AS7331 registers
static const uint8_t REG_OSR   = 0x00;
static const uint8_t REG_MRES  = 0x02;  // UVA LSB
static const uint8_t REG_CREG1 = 0x06;
static const uint8_t REG_CREG2 = 0x07;
static const uint8_t REG_CREG3 = 0x08;

// Modes
static const uint8_t MODE_CFG  = 0x02;
static const uint8_t MODE_MEAS = 0x03;

// LSB table @ TIME = 10 (datasheet, nW/cm²)
static const float LSB_TIME10[12][3] = {
  {0.16213f,0.18001f,0.03849f},{0.32425f,0.36002f,0.07698f},
  {0.64851f,0.72004f,0.15396f},{1.2970f,1.4401f,0.30791f},
  {2.5940f,2.8802f,0.61582f},{5.1881f,5.7604f,1.2316f},
  {10.376f,11.521f,2.4633f},{20.752f,23.041f,4.9266f},
  {41.505f,46.083f,9.8530f},{83.010f,92.167f,19.706f},
  {166.02f,184.33f,39.412f},{332.04f,368.67f,78.824f}
};

float AS7331Component::lsb_(uint8_t ch) const {
  int exp = 10 - (time_ & 0x0F);
  float scale = (exp >= 0) ? float(1UL << exp) : 1.0f / float(1UL << (-exp));
  return LSB_TIME10[gain_][ch] * scale * 1e-5f; // -> W/m²
}

void AS7331Component::setup() {
  ESP_LOGI(TAG, "Setting up AS7331 (continuous mode)");

  uint8_t v;

  // CFG
  v = MODE_CFG;
  this->write_register(REG_OSR, &v, 1);

  // Gain + integration time
  v = (gain_ << 4) | (time_ & 0x0F);
  this->write_register(REG_CREG1, &v, 1);

  // Divider off
  v = 0x00;
  this->write_register(REG_CREG2, &v, 1);

  // Continuous mode
  v = 0x00;
  this->write_register(REG_CREG3, &v, 1);

  // Start measurement
  v = MODE_MEAS;
  this->write_register(REG_OSR, &v, 1);
}

void AS7331Component::update() {
  uint8_t raw[6];
  if (!this->read_register(REG_MRES, raw, 6)) {
    ESP_LOGW(TAG, "AS7331 read failed");
    return;
  }

  uint16_t r_uva = (raw[1] << 8) | raw[0];
  uint16_t r_uvb = (raw[3] << 8) | raw[2];
  uint16_t r_uvc = (raw[5] << 8) | raw[4];

  if (uva_) uva_->publish_state(r_uva * lsb_(0));
  if (uvb_) uvb_->publish_state(r_uvb * lsb_(1));
  if (uvc_) uvc_->publish_state(r_uvc * lsb_(2));
}

}  // namespace as7331
}  // namespace esphome
