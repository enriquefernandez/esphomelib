#include "esphomelib/defines.h"

#ifdef USE_SENSOR

#include <utility>
#include "esphomelib/sensor/sensor.h"

#include "esphomelib/log.h"

ESPHOMELIB_NAMESPACE_BEGIN

namespace sensor {

static const char *TAG = "sensor.sensor";

void Sensor::publish_state(float state) {
  this->raw_state = state;
  this->raw_callback_.call(state);

  ESP_LOGV(TAG, "'%s': Received new state %f", this->name_.c_str(), state);

  if (this->filter_list_ == nullptr) {
    this->send_state_to_frontend_internal_(state);
  } else {
    this->filter_list_->input(state);
  }
}
void Sensor::push_new_value(float state) {
  this->publish_state(state);
}
std::string Sensor::unit_of_measurement() {
  return "";
}
std::string Sensor::icon() {
  return "";
}
uint32_t Sensor::update_interval() {
  return 0;
}
int8_t Sensor::accuracy_decimals() {
  return 0;
}
Sensor::Sensor(const std::string &name)
    : Nameable(name), state(NAN), raw_state(NAN) {
  // By default, apply a smoothing over the last 15 values
  this->add_filter(new SlidingWindowMovingAverageFilter(15, 15));
}
Sensor::Sensor()
    : Sensor("") {

}

void Sensor::set_unit_of_measurement(const std::string &unit_of_measurement) {
  this->unit_of_measurement_ = unit_of_measurement;
}
void Sensor::set_icon(const std::string &icon) {
  this->icon_ = icon;
}
void Sensor::set_accuracy_decimals(int8_t accuracy_decimals) {
  this->accuracy_decimals_ = accuracy_decimals;
}
void Sensor::add_on_state_callback(std::function<void(float)> &&callback) {
  this->callback_.add(std::move(callback));
}
void Sensor::add_on_raw_state_callback(std::function<void(float)> &&callback) {
  this->raw_callback_.add(std::move(callback));
}
std::string Sensor::get_icon() {
  if (this->icon_.has_value())
    return *this->icon_;
  return this->icon();
}
std::string Sensor::get_unit_of_measurement() {
  if (this->unit_of_measurement_.has_value())
    return *this->unit_of_measurement_;
  return this->unit_of_measurement();
}
int8_t Sensor::get_accuracy_decimals() {
  if (this->accuracy_decimals_.has_value())
    return *this->accuracy_decimals_;
  return this->accuracy_decimals();
}
void Sensor::add_filter(Filter *filter) {
  // inefficient, but only happens once on every sensor setup and nobody's going to have massive amounts of
  // filters
  if (this->filter_list_ == nullptr) {
    this->filter_list_ = filter;
  } else {
    Filter *last_filter = this->filter_list_;
    while (last_filter->next_ != nullptr)
      last_filter = last_filter->next_;
    last_filter->next_ = filter;
    last_filter->output_ = [filter] (float value) {
      filter->input(value);
    };
  }
  filter->initialize([this](float value) {
    this->send_state_to_frontend_internal_(value);
  });
}
void Sensor::add_filters(const std::list<Filter *> &filters) {
  for (Filter *filter : filters) {
    this->add_filter(filter);
  }
}
void Sensor::set_filters(const std::list<Filter *> &filters) {
  this->clear_filters();
  this->add_filters(filters);
}
void Sensor::clear_filters() {
  Filter *filter = this->filter_list_;
  while (filter != nullptr) {
    Filter *next_filter = filter->next_;
    delete filter;
    filter = next_filter;
  }

  this->filter_list_ = nullptr;
}
float Sensor::get_value() const {
  return this->state;
}
float Sensor::get_state() const {
  return this->state;
}
float Sensor::get_raw_value() const {
  return this->raw_state;
}
float Sensor::get_raw_state() const {
  return this->raw_state;
}
std::string Sensor::unique_id() { return ""; }

void Sensor::send_state_to_frontend_internal_(float state) {
  this->has_state_ = true;
  this->state = state;
  ESP_LOGD(TAG, "'%s': Sending state %.5f with %d decimals of accuracy",
           this->get_name().c_str(), state, this->get_accuracy_decimals());
  this->callback_.call(state);
}
SensorStateTrigger *Sensor::make_state_trigger() {
  return new SensorStateTrigger(this);
}
SensorRawStateTrigger *Sensor::make_raw_state_trigger() {
  return new SensorRawStateTrigger(this);
}
ValueRangeTrigger *Sensor::make_value_range_trigger() {
  return new ValueRangeTrigger(this);
}
bool Sensor::has_state() const {
  return this->has_state_;
}
uint32_t Sensor::calculate_expected_filter_update_interval() {
  uint32_t interval = this->update_interval();
  if (interval == 4294967295UL)
    // update_interval: never
    return 0;
  Filter *filter = this->filter_list_;
  while (filter != nullptr) {
    interval = filter->expected_interval(interval);
    filter = filter->next_;
  }
  return interval;
}

PollingSensorComponent::PollingSensorComponent(const std::string &name, uint32_t update_interval)
    : PollingComponent(update_interval), Sensor(name) {}

uint32_t PollingSensorComponent::update_interval() {
  return this->get_update_interval();
}

const char ICON_FLASH[] = "mdi:flash";
const char UNIT_V[] = "V";
const char ICON_EMPTY[] = "";
const char UNIT_C[] = "°C";
const char ICON_GAUGE[] = "mdi:gauge";
const char UNIT_HPA[] = "hPa";
const char ICON_WATER_PERCENT[] = "mdi:water-percent";
const char UNIT_PERCENT[] = "%";
const char ICON_SCREEN_ROTATION[] = "mdi:screen-rotation";
const char ICON_BRIEFCASE_DOWNLOAD[] = "mdi:briefcase-download";
const char UNIT_DEGREES_PER_SECOND[] = "°/s";
const char UNIT_M_PER_S_SQUARED[] = "m/s²";
const char ICON_BRIGHTNESS_5[] = "mdi:brightness-5";
const char UNIT_LX[] = "lx";
const char UNIT_OHM[] = "Ω";
const char ICON_GAS_CYLINDER[] = "mdi:gas-cylinder";
const char ICON_PERIODIC_TABLE_CO2[] = "mdi:periodic-table-co2";
const char UNIT_PPM[] = "ppm";
const char UNIT_A[] = "A";
const char UNIT_W[] = "W";
const char ICON_MAGNET[] = "mdi:magnet";
const char ICON_THERMOMETER[] = "mdi:thermometer";
const char ICON_LIGHTBULB[] = "mdi:lightbulb";
const char ICON_BATTERY[] = "mdi:battery";
const char ICON_FLOWER[] = "mdi:flower";
const char UNIT_UT[] = "µT";
const char UNIT_DEGREES[] = "°";
const char UNIT_K[] = "K";
const char UNIT_MICROSIEMENS_PER_CENTIMETER[] = "µS/cm";
const char UNIT_MICROGRAMS_PER_CUBIC_METER[] = "µg/m^3";
const char ICON_CHEMICAL_WEAPON[] = "mdi:chemical-weapon";

SensorStateTrigger::SensorStateTrigger(Sensor *parent) {
  parent->add_on_state_callback([this](float value) {
    this->trigger(value);
  });
}

SensorRawStateTrigger::SensorRawStateTrigger(Sensor *parent) {
  parent->add_on_raw_state_callback([this](float value) {
    this->trigger(value);
  });
}

ValueRangeTrigger::ValueRangeTrigger(Sensor *parent) {
  parent->add_on_state_callback([this](float value) {
    if (isnan(value))
      return;

    float local_min = this->min_.value(value);
    float local_max = this->max_.value(value);

    bool in_range = (isnan(local_min) && value <= local_max) || (isnan(local_max) && value >= local_min)
        || (!isnan(local_min) && !isnan(local_max) && local_min <= value && value <= local_max);

    if (in_range != this->previous_in_range_ && in_range) {
      this->trigger(value);
    }

    this->previous_in_range_ = in_range;
  });
}
void ValueRangeTrigger::set_min(std::function<float(float)> &&min) {
  this->min_ = std::move(min);
}
void ValueRangeTrigger::set_min(float min) {
  this->min_ = min;
}
void ValueRangeTrigger::set_max(std::function<float(float)> &&max) {
  this->max_ = std::move(max);
}
void ValueRangeTrigger::set_max(float max) {
  this->max_ = max;
}

} // namespace sensor

ESPHOMELIB_NAMESPACE_END

#endif //USE_SENSOR
