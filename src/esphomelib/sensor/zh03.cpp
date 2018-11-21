#include "esphomelib/defines.h"

// #ifdef USE_ZH03

#include "esphomelib/sensor/zh03.h"
#include "esphomelib/log.h"


ESPHOMELIB_NAMESPACE_BEGIN

namespace sensor {

static const char *TAG = "sensor.pmsensor";
// static const uint8_t MHZ19_REQUEST_LENGTH = 8;
// static const uint8_t MHZ19_RESPONSE_LENGTH = 9;
// static const uint8_t MHZ19_COMMAND_GET_PPM[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00};

static const uint8_t PM_REQUEST_LENGTH = 9;
static const uint8_t PM_RESPONSE_LENGTH = 9;
// Set QA
static const uint8_t PM_COMMAND_ENTER_QA[] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
// b"\xFF\x01\x78\x41\x00\x00\x00\x00\x46"

static const uint8_t PM_COMMAND_REQUEST_QA_SAMPLE[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
// \xFF\x01\x86\x00\x00\x00\x00\x00\x79

// Fan on
static const uint8_t PM_COMMAND_FAN_ON[] = {0xFF, 0x01, 0xA7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58};
static const uint8_t PM_COMMAND_FAN_OFF[] = {0xFF, 0x01, 0xA7, 0x01, 0x00, 0x00, 0x00, 0x00, 0x57};




ZH03Component::ZH03Component(UARTComponent *parent, const std::string &pm25_name, const std::string &pm10_name, const std::string &pm1_name,
                 uint32_t update_interval)
    : PollingComponent(update_interval), UARTDevice(parent),
      pm25_sensor_(new PMSensor25(pm25_name, this)), pm10_sensor_(new PMSensor10(pm10_name, this)), pm1_sensor_(new PMSensor1(pm1_name, this)) {

}
// uint8_t mhz19_checksum_(const uint8_t *command) {
//   uint8_t sum = 0;
//   for (uint8_t i = 1; i < MHZ19_REQUEST_LENGTH; i++) {
//     sum += command[i];
//   }
//   return 0xFF - sum + 0x01;
// }

bool ZH03Component::set_fan(bool on) {
  ESP_LOGD(TAG, "Trying to set fan=%zi", on);
  uint8_t response[PM_RESPONSE_LENGTH];
  const uint8_t* command = on ? PM_COMMAND_FAN_ON : PM_COMMAND_FAN_OFF;
  if (!this->pm_write_command_(command, response)) {
    ESP_LOGW(TAG, "Fan Reading data from PM sensor failed!");
    // this->status_set_warning();
    return false;
  }

  if (response[0] != 0xFF || response[1] != 0xA7) {
    ESP_LOGW(TAG, "Invalid preamble from PM sensor for set fan command.");
    this->status_set_warning();
    return false;
  }

  if (response[2] == 0x01) {
    ESP_LOGD(TAG, "Succeded setting the fan mode.");
    return true;
  }
  else if (response[2] == 0x00) {
    ESP_LOGW(TAG, "Set command failed!");
    return false;

  }
  else {
    ESP_LOGW(TAG, "Unknown response for set command. Failure?");
    return false;
  }
  return true;
}

bool ZH03Component::set_qa_mode() {
    // uint8_t response[PM_RESPONSE_LENGTH];
    ESP_LOGD(TAG, "Trying to set QA.");
  if (!this->pm_write_command_(PM_COMMAND_ENTER_QA, nullptr)) {
    ESP_LOGW(TAG, "QA Reading data from PM sensor failed!");
    // this->status_set_warning();
    return false;
  }
  ESP_LOGD(TAG, "QA mode set!");
  return true;
}


bool ZH03Component::read_qa_sample() {
  uint8_t response[PM_RESPONSE_LENGTH];
  ESP_LOGD(TAG, "Sending QA request!");
  if (!this->pm_write_command_(PM_COMMAND_REQUEST_QA_SAMPLE, response)) {
    ESP_LOGW(TAG, "Sample. Reading data from PM sensor failed!");
    // this->status_set_warning();
    return false;
  }

  if (response[0] != 0xFF || response[1] != 0x86) {
    ESP_LOGW(TAG, "Invalid preamble from PM sensor for QA sample.");
    this->status_set_warning();
    return false;
  }

  const uint16_t pm25_conc = (uint16_t(response[2]) << 8) | response[3];
  const uint16_t pm10_conc = (uint16_t(response[4]) << 8) | response[5];
  const uint16_t pm1_conc = (uint16_t(response[6]) << 8) | response[7];

  ESP_LOGD(TAG, "Sensor read values PM2.5=%d, PM10=%d, PM1=%d", pm25_conc, pm10_conc, pm1_conc);

  // Publish values
  this->pm25_sensor_->publish_state(pm25_conc);
  ESP_LOGD(TAG, "Received PM2.5 concentration=%u ug/m3", pm25_conc);
  this->pm10_sensor_->publish_state(pm10_conc);
  ESP_LOGD(TAG, "Received PM10 concentration=%u ug/m3", pm10_conc);
  this->pm1_sensor_->publish_state(pm1_conc);
  ESP_LOGD(TAG, "Received PM1 concentration=%u ug/m3", pm1_conc);

  return true;
}

void ZH03Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ZH03...");
  if (!this->set_qa_mode()) {
    this->mark_failed();
  }
}

void ZH03Component::update() {

  ESP_LOGD(TAG, "Sensor updating from sensor component! enrique.");
  read_qa_sample();
}

bool ZH03Component::pm_write_command_(const uint8_t *command, uint8_t *response) {
  this->flush();
  this->write_array(command, PM_REQUEST_LENGTH);
  // this->write_byte(mhz19_checksum_(command));

  if (response == nullptr)
    return true;

  bool ret = this->read_array(response, PM_RESPONSE_LENGTH);
  this->flush();
  return ret;
}


// bool ZH03Component::mhz19_write_command_(const uint8_t *command, uint8_t *response) {
//   this->flush();
//   this->write_array(command, MHZ19_REQUEST_LENGTH);
//   this->write_byte(mhz19_checksum_(command));

//   if (response == nullptr)
//     return true;

//   bool ret = this->read_array(response, MHZ19_RESPONSE_LENGTH);
//   this->flush();
//   return ret;
// }
// MHZ19TemperatureSensor *ZH03Component::make_temperature_sensor(const std::string &name) {
//   return this->temperature_sensor_ = new MHZ19TemperatureSensor(name, this);
// }
// MHZ19CO2Sensor *ZH03Component::get_co2_sensor() const {
//   return this->co2_sensor_;
// }


float ZH03Component::get_setup_priority() const {
  return setup_priority::HARDWARE_LATE;
}

} // namespace sensor

ESPHOMELIB_NAMESPACE_END

//#endif //USE_ZH03

