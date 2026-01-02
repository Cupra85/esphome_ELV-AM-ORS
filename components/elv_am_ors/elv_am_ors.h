#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace elv_am_ors {

class ELVAMORS : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_uva_sensor(sensor::Sensor *s) { uva_ = s; }
  void set_uvb_sensor(sensor::Sensor *s) { uvb_ = s; }
  void set_uvc_sensor(sensor::Sensor *s) { uvc_ = s; }
  void set_uv_index_sensor(sensor::Sensor *s) { uv_index_ = s; }
  void set_illuminance_sensor(sensor::Sensor *s) { illuminance_ = s; }
  void set_irradiance_sensor(sensor::Sensor *s) { irradiance_ = s; }

  void setup() override;
  void update() override;

 protected:
  sensor::Sensor *uva_{nullptr};
  sensor::Sensor *uvb_{nullptr};
  sensor::Sensor *uvc_{nullptr};
  sensor::Sensor *uv_index_{nullptr};
  sensor::Sensor *illuminance_{nullptr};
  sensor::Sensor *irradiance_{nullptr};

  bool read_uv_(float &uva, float &uvb, float &uvc);
  bool read_illuminance_(float &lux);
  float read_irradiance_();
};

}  // namespace elv_am_ors
}  // namespace esphome
