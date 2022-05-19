#pragma once

#ifdef USE_ARDUINO

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/optional.h"
#include "esphome/core/log.h"

#include "CEC_Device.h"

#include <functional>

#ifdef USE_ESP32
#include "driver/timer.h"
#endif

typedef struct hw_timer_s
{
  uint8_t group;
  uint8_t num;
} hw_timer_t;

namespace esphome {
namespace hdmi_cec {

class HdmiCecTrigger;
template<typename... Ts> class HdmiCecSendAction;

static const uint8_t HDMI_CEC_MAX_DATA_LENGTH = 16;

struct HdmiCecStore {
  CEC_Device cec_device_;
  ISRInternalGPIOPin pin_;
  uint8_t pin_number_;
#ifdef USE_ESP32
  timer_group_t timer_group;
  timer_idx_t timer_idx;
#endif

  bool desired_line_state_;
  unsigned long low_count_ = 0;
  unsigned long pin_interrupt_count_;
  unsigned long timer_interrupt_count_;

  static void pin_interrupt(HdmiCecStore *arg);
  static void timer_interrupt(HdmiCecStore *arg);
  void interrupt_();
};

class HdmiCec : public Component {
 public:
  HdmiCec(){};
  // Component overrides
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void loop() override;

  void send_data(uint8_t source, uint8_t destination, const std::vector<uint8_t> &data);
  void set_address(uint8_t address) { this->address_ = address; }
  void set_physical_address(uint16_t physical_address) { this->physical_address_ = physical_address; }
  void set_promiscuous_mode(uint16_t promiscuous_mode) { this->promiscuous_mode_ = promiscuous_mode; }
  void set_pin(InternalGPIOPin *pin, uint8_t pin_number) {
    this->pin_ = pin;
    this->pin_->pin_mode(gpio::FLAG_INPUT);
    this->store_.pin_ = this->pin_->to_isr();
    this->store_.pin_number_ = pin_number;
  }
  void add_trigger(HdmiCecTrigger *trigger);

 protected:
  void send_data_internal_(uint8_t source, uint8_t destination, unsigned char *buffer, int count);

  template<typename... Ts> friend class HdmiCecSendAction;

  void on_ready_(int logical_address);
  void on_transmit_complete_(unsigned char *buffer, int count, bool ack);
  void on_receive_complete_(unsigned char *buffer, int count, bool ack);

  InternalGPIOPin *pin_;
  std::vector<HdmiCecTrigger *> triggers_{};
  uint8_t address_;
  uint16_t physical_address_;
  bool promiscuous_mode_;
  bool ready_ = false;
  HdmiCecStore store_{};
};

template<typename... Ts> class HdmiCecSendAction : public Action<Ts...>, public Parented<HdmiCec> {
 public:
  void set_data_template(const std::function<std::vector<uint8_t>(Ts...)> func) {
    this->data_func_ = func;
    this->static_ = false;
  }
  void set_data_static(const std::vector<uint8_t> &data) {
    this->data_static_ = data;
    this->static_ = true;
  }
  void set_source(uint8_t source) { this->source_ = source; }
  void set_destination(uint8_t destination) { this->destination_ = destination; }

  void play(Ts... x) override {
    auto source = this->source_.has_value() ? *this->source_ : this->parent_->address_;
    if (this->static_) {
      this->parent_->send_data(source, this->destination_, this->data_static_);
    } else {
      auto val = this->data_func_(x...);
      this->parent_->send_data(source, this->destination_, val);
    }
  }

 protected:
  optional<uint8_t> source_{};
  uint8_t destination_;
  bool static_{false};
  std::function<std::vector<uint8_t>(Ts...)> data_func_{};
  std::vector<uint8_t> data_static_{};
};

class HdmiCecTrigger : public Trigger<uint8_t, uint8_t, std::vector<uint8_t>, bool>, public Component {
  friend class HdmiCec;

 public:
  explicit HdmiCecTrigger(HdmiCec *parent) : parent_(parent){};
  void setup() override { this->parent_->add_trigger(this); }

  void set_source(uint8_t source) { this->source_ = source; }
  void set_destination(uint8_t destination) { this->destination_ = destination; }
  void set_opcode(uint8_t opcode) { this->opcode_ = opcode; }
  void set_data(const std::vector<uint8_t> &data) { this->data_ = data; }

 protected:
  HdmiCec *parent_;
  optional<uint8_t> source_{};
  optional<uint8_t> destination_{};
  optional<uint8_t> opcode_{};
  optional<std::vector<uint8_t>> data_{};
};

}  // namespace hdmi_cec
}  // namespace esphome

#endif  // USE_ARDUINO
