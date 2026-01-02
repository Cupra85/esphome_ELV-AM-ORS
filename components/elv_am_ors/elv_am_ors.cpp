#include "elv_am_ors.h"
#include "esphome/core/log.h"

namespace esphome {
namespace elv_am_ors {

static const char *TAG = "elv_am_ors";

// OPT3001
static constexpr uint8_t OPT3001_ADDR = 0x45;
static constexpr uint8_t OPT3001_REG_RESULT = 0x00;
static constexpr uint8_t OPT3001_REG_CONFIG = 0x01;

// GPIO → ADC1 mapping (ESP32-S3)
static bool gpio_to_adc1_channel(uint8_t gpio, adc1_channel_t &channel) {
  switch (gpio) {
    case 1: channel = ADC1_CHANNEL_0; break;
    case 2: channel = ADC1_CHANNEL_1; break;
    case 3: channel = ADC1_CHANNEL_2; break;
    case 4: channel = ADC1_CHANNEL_3; break;
    case 5: channel = ADC1_CHANNEL_4; break;
    case 6: channel = ADC1_CHANNEL_5; break;
    case 7: channel = ADC1_CHANNEL_6; break;
    case 8: channel = ADC1_CHANNEL_7; break;
    default: return false;
  }
  return true;
}

void ELVAMORS::setup() {
  ESP_LOGI(TAG, "ELV-AM-ORS initialisiert");

  // OPT3001 konfigurieren (Continuous, Auto-Range)
  this->write_address(OPT3001_ADDR);
  uint8_t cfg[3] = {
      OPT3001_REG_CONFIG,
      0b11000010,
      0b00000000,
  };
  this->write(cfg, 3);

  // ADC initialisieren
  if (irradiance_ && irra_adc_pin_) {
    uint8_t gpio = irra_adc_pin_->get_pin();

    if (!gpio_to_adc1_channel(gpio, irra_adc_channel_)) {
      ESP_LOGE(TAG, "GPIO %u ist kein gültiger ADC1-Pin", gpio);
      return;
    }

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(irra_adc_channel_, ADC_ATTEN_DB_11);

    ESP_LOGI(TAG, "IRRA-ADC auf GPIO %u konfiguriert", gpio);
  }
}

void ELVAMORS::update() {
  float uva, uvb, uvc;

  if (read_uv_(uva, uvb, uvc)) {
    if (uva_) uva_->publish_state(uva);
    if (uvb_) uvb_->publish_state(uvb);
    if (uvc_) uvc_->publish_state(uvc);

    if (uv_index_) {
      float uv_index = (uva + uvb + uvc) / 0.025f;
      uv_index_->publish_state(uv_index);
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
  uint8_t start[2] = {0x00, 0x01};
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
  this->write_address(OPT3001_ADDR);

  uint8_t data[2];
  if (!read_register(OPT3001_REG_RESULT, data, 2))
    return false;

  uint16_t raw = (data[0] << 8) | data[1];
  uint16_t exponent = (raw >> 12) & 0x0F;
  uint16_t mantissa = raw & 0x0FFF;

  lux = mantissa * (0.01f * (1 << exponent));
  return true;
}

float ELVAMORS::read_irradiance_() {
  int raw = adc1_get_raw(irra_adc_channel_);
  float voltage = raw * (3.3f / 4095.0f);
  return voltage * (1286.0f / 3.3f);
}

}  // namespace elv_am_ors
}  // namespace esphome
