#ifndef ESPHOMELIB_ZH03_COMPONENT_H
#define ESPHOMELIB_ZH03_COMPONENT_H

#include "esphomelib/defines.h"

// #ifdef USE_ZH03

#include "esphomelib/component.h"
#include "esphomelib/uart_component.h"
#include "esphomelib/sensor/sensor.h"

ESPHOMELIB_NAMESPACE_BEGIN

namespace sensor {

// using PMSensorTemperatureSensor = sensor::EmptyPollingParentSensor<0, ICON_EMPTY, UNIT_C>;
// using PMSensorCO2Sensor = sensor::EmptyPollingParentSensor<0, ICON_PERIODIC_TABLE_CO2, UNIT_PPM>;

using PMSensor25 = sensor::EmptyPollingParentSensor<0, ICON_CHEMICAL_WEAPON, UNIT_MICROGRAMS_PER_CUBIC_METER>;
using PMSensor10 = sensor::EmptyPollingParentSensor<0, ICON_CHEMICAL_WEAPON, UNIT_MICROGRAMS_PER_CUBIC_METER>;
using PMSensor1 = sensor::EmptyPollingParentSensor<0, ICON_CHEMICAL_WEAPON, UNIT_MICROGRAMS_PER_CUBIC_METER>;

class ZH03Component : public PollingComponent, public UARTDevice {
 public:
  ZH03Component(UARTComponent *parent, const std::string &pm25_name, const std::string &pm10_name, const std::string &pm1_name,
                 uint32_t update_interval = 15000);
  
  float get_setup_priority() const override;

  void setup() override;

  void update() override;

  bool set_fan(bool on);

  bool set_qa_mode();

  bool read_qa_sample();

  // PMSensorTemperatureSensor *make_temperature_sensor(const std::string &name);
  // PMSensorCO2Sensor *get_co2_sensor() const;

  PMSensor25 *get_pm25_sensor() const {
    return this->pm25_sensor_;
  }
  PMSensor10 *get_pm10_sensor() const {
    return this->pm10_sensor_;
  }
  PMSensor1 *get_pm1_sensor() const {
    return this->pm1_sensor_;
  }

 protected:
  bool pm_write_command_(const uint8_t *command, uint8_t *response);

  // PMSensorTemperatureSensor *temperature_sensor_{nullptr};
  // PMSensorCO2Sensor *co2_sensor_;

  PMSensor25 *pm25_sensor_;
  PMSensor10 *pm10_sensor_;
  PMSensor1 *pm1_sensor_;

};

} // namespace sensor

ESPHOMELIB_NAMESPACE_END


//#endif //USE_ZH03

#endif //ESPHOMELIB_ZH03_COMPONENT_H
