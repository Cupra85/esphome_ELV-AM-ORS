#include "elv_am_ors.h"
#include "esphome/core/log.h"

namespace esphome {
namespace elv_am_ors {

static const char *TAG = "elv_am_ors";

// AS7331
static constexpr uint8_t AS7331_REG_MODE   = 0x00;
static constexpr uint8_t AS7331_REG_CONFIG = 0x01;

// OPT3001
static constexpr uint8_t OPT3001_ADDR       = 0x45;
static constexpr uint8_t OPT3001_REG_RESULT = 0x00;
static constexpr uint8_t OPT3001_REG_CONFIG = 0x01;

void ELVAMORS::setup() {
  ESP_LOGI(TAG, "ELV-AM-ORS Setup");

  // --- AS7331: Gain 1x, Integration Time ~100 ms ---
  {
    uint8_t cfg[2] = {AS7331_REG_CONFIG, 0x04};
    this->write(cfg, 2);
  }

  // --- OPT3001: Continuous, Auto-Range ---
  this->set_i2c_address(OPT3001_ADDR);
  uint8_t opt_cfg[2] = {OPT3001_REG_CONFIG, 0b11000010};
  this->write(opt_cfg, 2);

  // --- ADC Setup ---
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(
      static_cast<adc1_channel_t>(irra_adc_channel_),
      ADC_ATTEN_DB_12);

  auto cal = esp_adc_cal_characterize(
      ADC_UNIT_1,
      ADC_ATTEN_DB_12,
      ADC_WIDTH_BIT_12,
      1100,
      &adc_chars_);

  adc_calibrated_ = (cal != ESP_ADC_CAL_VAL_DEFAULT_VREF);
}

void ELVAMORS::update() {
  float uva, uvb, uvc;

  if (read_uv_(uva, uvb, uvc)) {
    if (uva_) uva_->publish_state(uva);
    if (uvb_) uvb_->publish_state(uvb);
    if (uvc_) uvc_->publish_state(uvc);

    if (uv_index_) {
      uv_index_->publish_state((uva + uvb + uvc) / 0.025f);
    }
  }

  if (illuminance_) {
    float lux;
    if (read_illuminance_(lux))
      illuminance_->publish_state(lux);
  }

  if (irradiance_) {
    irradiance_->publish_state(read_irradiance_());
  }
}

bool ELVAMORS::read_uv_(float &uva, float &uvb, float &uvc) {
  uint8_t start[2] = {AS7331_REG_MODE, 0x01};
  if (!write(start, 2))
    return false;

  delay(50);

  uint8_t data[6];
  if (!read_register(0x10, data, 6))
    return false;

  uva = ((data[0] << 8) | data[1]) * 0.001f;
  uvb = ((data[2] << 8) | data[3]) * 0.001f;
  uvc = ((data[4] << 8) | data[5]) * 0.001f;

  return true;
}

bool ELVAMORS::read_illuminance_(float &lux) {
  this->set_i2c_address(OPT3001_ADDR);

  uint8_t data[2];
  if (!read_register(OPT3001_REG_RESULT, data, 2))
    return false;

  uint16_t raw = (data[0] << 8) | data[1];
  uint16_t exp = (raw >> 12) & 0x0F;
  uint16_t man = raw & 0x0FFF;

  lux = man * (0.01f * (1 << exp));
  return true;
}

float ELVAMORS::read_irradiance_() {
  int raw = adc1_get_raw(static_cast<adc1_channel_t>(irra_adc_channel_));

  uint32_t mv = adc_calibrated_
      ? esp_adc_cal_raw_to_voltage(raw, &adc_chars_)
      : raw * 3300 / 4095;

  float voltage = mv / 1000.0f;
  return voltage * (1286.0f / 3.3f);
}

}  // namespace elv_am_ors
}  // namespace esphome
