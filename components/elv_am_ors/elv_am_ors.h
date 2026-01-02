#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace elv_am_ors {

class ELVAMORS : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_as7331_address(uint8_t address) { this->as7331_address_ = address; }
  void set_opt3001_address(uint8_t address) { this->opt3001_address_ = address; }

  void set_as7331_gain_code(uint8_t gain_code) { this->as7331_gain_code_ = gain_code; }
  void set_as7331_time_code(uint8_t time_code) { this->as7331_time_code_ = time_code; }

  void set_irra_adc_pin(uint8_t pin) { this->irra_adc_pin_ = pin; }
  void set_irra_adc_ref_voltage(float v) { this->irra_adc_ref_voltage_ = v; }
  void set_irra_adc_resolution(uint16_t res) { this->irra_adc_resolution_ = res; }
  void set_irra_calibration(float slope_wm2_per_v, float offset_v) {
    this->irra_slope_wm2_per_v_ = slope_wm2_per_v;
    this->irra_offset_v_ = offset_v;
  }

  void set_uva_sensor(sensor::Sensor *s) { this->uva_sensor_ = s; }
  void set_uvb_sensor(sensor::Sensor *s) { this->uvb_sensor_ = s; }
  void set_uvc_sensor(sensor::Sensor *s) { this->uvc_sensor_ = s; }
  void set_uvi_sensor(sensor::Sensor *s) { this->uvi_sensor_ = s; }
  void set_illuminance_sensor(sensor::Sensor *s) { this->illuminance_sensor_ = s; }
  void set_irradiance_sensor(sensor::Sensor *s) { this->irradiance_sensor_ = s; }

 protected:
  bool as7331_configure_();
  bool as7331_read_uv_(float &uva_w_m2, float &uvb_w_m2, float &uvc_w_m2);
  float as7331_lsb_nwcm2_(uint8_t gain_code, uint8_t time_code, uint8_t channel_index) const;

  bool opt3001_configure_();
  bool opt3001_read_lux_(float &lux);

  bool read_irradiance_(float &irradiance_w_m2);

  uint8_t as7331_address_{0x77};
  uint8_t opt3001_address_{0x45};

  uint8_t as7331_gain_code_{0};  // 0=2048x ... 11=1x
  uint8_t as7331_time_code_{6};  // 0=1ms ... 10=1024ms

  uint8_t irra_adc_pin_{4};  // default GPIO4
  float irra_adc_ref_voltage_{3.3};
  uint16_t irra_adc_resolution_{4095};

  float irra_slope_wm2_per_v_{400.0};
  float irra_offset_v_{0.115};

  sensor::Sensor *uva_sensor_{nullptr};
  sensor::Sensor *uvb_sensor_{nullptr};
  sensor::Sensor *uvc_sensor_{nullptr};
  sensor::Sensor *uvi_sensor_{nullptr};
  sensor::Sensor *illuminance_sensor_{nullptr};
  sensor::Sensor *irradiance_sensor_{nullptr};

  bool as7331_ok_{false};
  bool opt3001_ok_{false};
};

}  // namespace elv_am_ors
}  // namespace esphome
