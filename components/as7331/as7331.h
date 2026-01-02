#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace as7331 {

class AS7331Component : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_uva_sensor(sensor::Sensor *s) { uva_ = s; }
  void set_uvb_sensor(sensor::Sensor *s) { uvb_ = s; }
  void set_uvc_sensor(sensor::Sensor *s) { uvc_ = s; }

  void set_gain(uint8_t g) { gain_ = g; }
  void set_integration_time(uint8_t t) { time_ = t; }

 protected:
  sensor::Sensor *uva_{nullptr};
  sensor::Sensor *uvb_{nullptr};
  sensor::Sensor *uvc_{nullptr};

  uint8_t gain_{5};   // default: 16x
  uint8_t time_{8};   // default: ~256 ms

  float lsb_(uint8_t channel) const;
};

}  // namespace as7331
}  // namespace esphome
