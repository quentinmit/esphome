#ifdef USE_ARDUINO

#include "hdmi_cec.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace hdmi_cec {

static const char *const TAG = "hdmi_cec";

void message_to_debug_string(char *message, const unsigned char *buffer, int count) {
  for (int i = 0; i < count; i++) {
    sprintf(&(message[i * 3]), "%02X", buffer[i]);
    if (i < count - 1) {
      sprintf(&(message[i * 3 + 2]), ":");
    }
  }
  message[count*3] = 0;
}

void HdmiCec::on_ready_(int logical_address) {
  // This is called after the logical address has been allocated
  ESP_LOGD(TAG, "Device ready, Logical address assigned: %d", logical_address);
  this->ready_ = true;
  this->address_ = logical_address;
  // Report physical address
  unsigned char buf[4] = {0x84, (unsigned char) (physical_address_ >> 8), (unsigned char) (physical_address_ & 0xff),
                          address_};
  this->send_data_internal_(this->address_, 0xF, buf, 4);
}

void HdmiCec::on_receive_complete_(unsigned char *buffer, int count, bool ack) {
  // No command received?
  if (count < 1)
    return;

  auto source = (buffer[0] & 0xF0) >> 4;
  auto destination = (buffer[0] & 0x0F);

  // If we're not in promiscuous mode and the message isn't for us, ignore it.
  if (!this->promiscuous_mode_ && destination != this->address_ && destination != 0xF) {
    return;
  }

  // Pop the source/destination byte from buffer
  buffer = &buffer[1];
  count = count - 1;

  char debug_message[HDMI_CEC_MAX_DATA_LENGTH * 3];
  message_to_debug_string(debug_message, buffer, count);
  ESP_LOGD(TAG, "RX: (%d->%d) %02X:%s", source, destination, ((source & 0x0f) << 4) | (destination & 0x0f),
           debug_message);

  uint8_t opcode = 0;

  if (count > 0) {
    // Ping message has empty data.
    opcode = buffer[0];
  }

  // Handling the physical address response in code instead of yaml since I think it always
  // needs to happen for other devices to be able to talk to this device.
  if (count > 0 && opcode == 0x83 && destination == address_) {
    // Report physical address
    unsigned char buf[4] = {0x84, (unsigned char) (physical_address_ >> 8), (unsigned char) (physical_address_ & 0xff),
                            address_};
    this->send_data_internal_(this->address_, 0xF, buf, 4);
  }

  for (auto *trigger : this->triggers_) {
    if ((!trigger->opcode_.has_value() || (count > 0 && *trigger->opcode_ == opcode)) &&
        (!trigger->source_.has_value() || (*trigger->source_ == source)) &&
        (!trigger->destination_.has_value() || (*trigger->destination_ == destination)) &&
        (!trigger->data_.has_value() ||
         (count == trigger->data_->size() && std::equal(trigger->data_->begin(), trigger->data_->end(), buffer)))) {
      auto data_vec = std::vector<uint8_t>(buffer, buffer + count);
      trigger->trigger(source, destination, data_vec);
    }
  }
}

void IRAM_ATTR HOT HdmiCecStore::pin_interrupt(HdmiCecStore *arg) {
  arg->pin_interrupt_count_++;
  bool currentLineState = arg->pin_.digital_read();
  unsigned long time = micros();
  arg->cec_device_.Run(time, currentLineState);
  arg->_desired_line_state = arg->cec_device_.DesiredLineState();
  return;
  if (arg->cec_device_.DesiredLineState()) {
    arg->pin_.pin_mode(gpio::FLAG_INPUT);
  } else {
    arg->pin_.digital_write(false);
    arg->pin_.pin_mode(gpio::FLAG_OUTPUT);
  }
  //if (arg->cec_device_._waitTime > 0) {
    // schedule another call to Run in waitTime
  //}
}

void HdmiCec::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HDMI-CEC...");
  this->store_.cec_device_.Initialize(0x2000, CEC_Device::CDT_AUDIO_SYSTEM, true);

  this->pin_->attach_interrupt(HdmiCecStore::pin_interrupt, &this->store_, gpio::INTERRUPT_ANY_EDGE);
}

void HdmiCec::dump_config() {
  ESP_LOGCONFIG(TAG, "HDMI-CEC:");
  ESP_LOGCONFIG(TAG, "  address: %d", this->address_);
  LOG_PIN("  Pin: ", this->pin_);
}

void HdmiCec::send_data(uint8_t source, uint8_t destination, const std::vector<uint8_t> &data) {
  const uint8_t *buffer = reinterpret_cast<const uint8_t *>(data.data());
  auto *char_buffer = const_cast<unsigned char *>(buffer);

  this->send_data_internal_(source, destination, char_buffer, data.size());
}

void HdmiCec::send_data_internal_(uint8_t source, uint8_t destination, unsigned char *buffer, int count) {
  char debug_message[HDMI_CEC_MAX_DATA_LENGTH * 3];
  message_to_debug_string(debug_message, buffer, count);
  ESP_LOGD(TAG, "TX: (%d->%d) %02X:%s", source, destination, ((source & 0x0f) << 4) | (destination & 0x0f),
           debug_message);

  this->store_.cec_device_.TransmitFrame(destination, buffer, count);
}

void HdmiCec::add_trigger(HdmiCecTrigger *trigger) { this->triggers_.push_back(trigger); };

void HdmiCec::loop() {
  // All the work is done by the ISRs, but we need to process the packets here.

  // if (1||this->store_.cec_device_._state != 0) {
  //   ESP_LOGD(TAG, "Current state: %d interrupts %ld", this->store_.cec_device_._state, this->store_.pin_interrupt_count_);
  // }

  if (!this->ready_ && this->store_.cec_device_.LogicalAddress() > 0) {
    this->on_ready_(this->store_.cec_device_.LogicalAddress());
  }

  unsigned char buffer[16];
  int count = 16;
  bool ack;
  if (this->store_.cec_device_.ReceivedPacket(buffer, &count, &ack)) {
    this->on_receive_complete_(buffer, count, ack);
  }

  // The current implementation of CEC is inefficient and relies on polling to
  // identify signal changes at just the right time. Experimentally it needs to
  // run faster than every ~0.04ms to be reliable. This can be solved by creating
  // an interrupt-driven CEC driver.
  static int counter = 0;
  static int timer = 0;
  if (millis() - timer > 10000) {
    ESP_LOGD(TAG, "Ran %d times in 10000ms (every %fms)", counter, 10000.0f / (float) counter);
    ESP_LOGD(TAG, "Current state: %d interrupts %ld desired line state %d", this->store_.cec_device_._state, this->store_.pin_interrupt_count_, this->store_._desired_line_state);
    counter = 0;
    timer = millis();
  }
  counter++;
}

}  // namespace hdmi_cec
}  // namespace esphome

#endif  // USE_ARDUINO
