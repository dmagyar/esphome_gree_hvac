#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"

namespace esphome {
namespace greeuart {

// enum SwingMode : uint8_t { SWING_OFF = 0, SWING_VERTICAL = 1, SWING_HORIZONTAL = 2, SWING_BOTH = 3 };

enum ac_mode: uint8_t {
  AC_MODE_OFF = 0x10,
  // auto 0-1-2-3
  AC_MODE_AUTO = 0x80,
  // cool 0-1-2-3
  AC_MODE_COOL = 0x90,
  // dry 1 (only) but set it to AUTO (0) for setting it to 1 later
  AC_MODE_DRY = 0xA0,
  // fanonly 0-1-2-3
  AC_MODE_FANONLY = 0xB0,
  // heat 0-1-2-3
  AC_MODE_HEAT = 0xC0
};

enum ac_basic_fan: uint8_t {
  AC_FAN_AUTO = 0x00,
  AC_FAN_LOW = 0x01,
  AC_FAN_MEDIUM = 0x02,
  AC_FAN_HIGH = 0x03,
  AC_FAN_SLEEP = 0x0b
}

enum ac_extra_fan: uint8_t {
  AC_EXTRAFAN_AUTO = 0x00,
  AC_EXTRAFAN_LOW = 0x01,
  AC_EXTRAFAN_MEDIUM_LOW = 0x02,
  AC_EXTRAFAN_MEDIUM = 0x03,
  AC_EXTRAFAN_MEDIUM_HIGH = 0x04,
  AC_EXTRAFAN_HIGH = 0x05
};

// ESPHOME only understands full swings :(
enum ac_swing: uint8_t {
  AC_SWING_OFF = 0x00,
  AC_SWING_VERTICAL = 0x10,
  AC_SWING_HORIZONTAL = 0x01,
  AC_SWING_BOTH = 0x11
};

// not implemented yet missing ESPHOME API
enum ac_updown_swing_H: uint8_t {
  AC_UPDOWN_SWING_OFF = 0x00,
  AC_UPDOWN_SWING_FULL = 0x10,
  AC_UPDOWN_FIXED_UPMOST = 0x20,
  AC_UPDOWN_FIXED_MIDDLE_UP = 0x30,
  AC_UPDOWN_FIXED_MIDDLE = 0x40,
  AC_UPDOWN_FIXED_MIDDLE_LOW = 0x50,
  AC_UPDOWN_FIXED_DOWNMOST = 0x60,
  AC_UPDOWN_SWING_DOWNMOST = 0x70,
  AC_UPDOWN_SWING_MIDDLE_LOW = 0x80,
  AC_UPDOWN_SWING_MIDDLE = 0x90,
  AC_UPDOWN_SWING_MIDDLE_UP = 0xA0,
  AC_UPDOWN_SWING_UPMOST = 0xB0
};

// not implemented yet
enum ac_leftright_swing_H: uint8_t {
  AC_LEFTRIGHT_SWING_OFF = 0x00,
  AC_LEFTRIGHT_SWING_FULL = 0x01,
  AC_LEFTRIGHT_FIXED_LEFTMOST = 0x02,
  AC_LEFTRIGHT_FIXED_LEFTMIDDLE = 0x03,
  AC_LEFTRIGHT_FIXED_MIDDLE = 0x04,
  AC_LEFTRIGHT_FIXED_RIGHTMIDDLE = 0x05,
  AC_LEFTRIGHT_FIXED_RIGHT = 0x06
};


#define GREE_START_BYTE 0x7E
#define GREE_RX_BUFFER_SIZE 52

union gree_start_bytes_t {
//     uint16_t u16;
    uint8_t u8x2[2];
};

struct gree_header_t
{
  gree_start_bytes_t start_bytes;
  uint8_t data_length;
};

struct gree_raw_packet_t
{
  gree_header_t header;
  uint8_t data[1]; // first data byte
};


/*
class Constants {
  public:
    // ac update interval in ms
    static const uint32_t AC_STATE_REQUEST_INTERVAL;
};
const uint32_t Constants::AC_STATE_REQUEST_INTERVAL = 300;
*/

class GreeUARTClimate : public climate::Climate, public uart::UARTDevice, public PollingComponent {
 public:
  // void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;
  void set_supported_presets(const std::set<climate::ClimatePreset> &presets) { this->supported_presets_ = presets; }
  void set_supported_swing_modes(const std::set<climate::ClimateSwingMode> &modes) {
     this->supported_swing_modes_ = modes;
  }

 protected:
  climate::ClimateTraits traits() override;
  void read_state_(uint8_t *data, uint8_t size);
  void send_data_(const uint8_t *message, uint8_t size);
  void dump_message_(const char *title, const uint8_t *message, uint8_t size);
  uint8_t get_checksum_(const uint8_t *message, size_t size);

 private:
  // uint32_t _update_period = Constants::AC_STATE_REQUEST_INTERVAL;

  // Parts of the message that must have specific values for "send" command.
  // These are not 0x00 and the meaning of those values is unknown at the moment.
  // Others set to 0x00
  // data_write_[41] = 12; // unknown but not 0x00. TODO
  uint8_t data_write_[47] = {0x7E, 0x7E, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t data_read_[GREE_RX_BUFFER_SIZE] = {0};

  bool receiving_packet_ = false;

  std::set<climate::ClimatePreset> supported_presets_{};
  std::set<climate::ClimateSwingMode> supported_swing_modes_{};
};

}  // namespace greeuart
}  // namespace esphome
