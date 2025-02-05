#include <cmath>
#include "gree.h"
#include "esphome/core/macros.h"

namespace esphome {
namespace greeuart {

static const char *const TAG = "greeuart";

// block of byte positions in requests/answers
// GrKoR: I recommend change this approach (byte positions) to structures like gree_raw_packet_t.
// You can use pointers (and type casting) for assignment structure to the data buffer.
// It will be much much easier to work with structures instead of array and byte positions
static const uint8_t FORCE_UPDATE = 7;
static const uint8_t MODE = 8;
static const uint8_t MODE_MASK = 0b11110000;
static const uint8_t FAN_MASK = 0b00001111;
static const uint8_t SWING_UPDOWN_MASK = 0b11110000;
static const uint8_t SWING_LEFTRIGHT_MASK = 0b00001111;

static const uint8_t SWING = 12;
static const uint8_t QUIET = 20;
static const uint8_t EXTRAFAN = 20;

static const uint8_t CRC_WRITE = 46;
//CRC_READ moved to read_state_ with last bytes because of different length of incoming packets
//static const uint8_t CRC_READ = 49;

static const uint8_t TEMPERATURE = 9;
static const uint8_t INDOOR_TEMPERATURE = 46;

// component settings
static const uint8_t MIN_VALID_TEMPERATURE = 16;
static const uint8_t MAX_VALID_TEMPERATURE = 30;
static const uint8_t TEMPERATURE_STEP = 1;

// prints user configuration
void GreeUARTClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "GreeUART:");
  ESP_LOGCONFIG(TAG, "  Update interval: %u", this->get_update_interval());
  this->dump_traits_(TAG);
  this->check_uart_settings(4800, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
}

void GreeUARTClimate::loop() {
  gree_raw_packet_t *raw_packet = (gree_raw_packet_t *)this->data_read_;

  while (!receiving_packet_ && this->available() >= sizeof(gree_header_t)) {
    if (this->peek() != GREE_START_BYTE) {
      this->read(); // read the byte "to nowhere"
      continue;
    }

    this->read_array(this->data_read_, sizeof(gree_start_bytes_t));
    receiving_packet_ = (raw_packet->header.start_bytes.u8x2[1] == GREE_START_BYTE);
    
    if (receiving_packet_) {
      this->read_byte( &raw_packet->header.data_length );

      if (raw_packet->header.data_length + sizeof(gree_header_t) > GREE_RX_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Incoming packet is too big! header.data_length = %d, maximum is %d", raw_packet->header.data_length, GREE_RX_BUFFER_SIZE - sizeof(gree_header_t));
        receiving_packet_ = false;
        memset(this->data_read_, 0, GREE_RX_BUFFER_SIZE);
      }
    }
  }

  if (receiving_packet_ && this->available() >= raw_packet->header.data_length) {
    this->read_array(raw_packet->data, raw_packet->header.data_length);

    dump_message_("Read array", this->data_read_, raw_packet->header.data_length + sizeof(gree_header_t));
    read_state_(this->data_read_, raw_packet->header.data_length + sizeof(gree_header_t));
    
    receiving_packet_ = false;
    memset(this->data_read_, 0, GREE_RX_BUFFER_SIZE);
  }
}

/*
void GreeUARTClimate::setup() {
  this->set_update_interval(300);
}
*/

void GreeUARTClimate::update() {
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));
}

climate::ClimateTraits GreeUARTClimate::traits() {
  auto traits = climate::ClimateTraits();

  traits.set_visual_min_temperature(MIN_VALID_TEMPERATURE);
  traits.set_visual_max_temperature(MAX_VALID_TEMPERATURE);
  traits.set_visual_temperature_step(TEMPERATURE_STEP);

  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_AUTO,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY,
    climate::CLIMATE_MODE_HEAT
  });

  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
//     climate::CLIMATE_FAN_MIDDLE,
//      climate::CLIMATE_FAN_FOCUS,
      climate::CLIMATE_FAN_HIGH,
      climate::CLIMATE_FAN_QUIET
  });

  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL
  });

  traits.set_supports_current_temperature(true);
  traits.set_supports_two_point_target_temperature(false);

  traits.set_supported_presets(this->supported_presets_);
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
  traits.add_supported_preset(climate::CLIMATE_PRESET_BOOST);
  traits.add_supported_preset(climate::CLIMATE_PRESET_SLEEP);
  traits.add_supported_preset(climate::CLIMATE_PRESET_COMFORT);
  traits.add_supported_preset(climate::CLIMATE_PRESET_AWAY);

  return traits;
}

void GreeUARTClimate::read_state_(uint8_t *data, uint8_t size) {
  // get checksum byte from received data (using the last byte)
  uint8_t data_crc = data[size-1];
  // get checksum byte based on received data (calculating)
  uint8_t get_crc = get_checksum_(data, size);

  if (data_crc != get_crc) {
    ESP_LOGW(TAG, "Invalid checksum.");
    return;
  }

// now we are using only packets with 0x31 as first data byte
  if (data[3] != 49) {
    ESP_LOGW(TAG, "Invalid packet type.");
    return;
  }

  this->target_temperature = data[TEMPERATURE] / 16 + MIN_VALID_TEMPERATURE;
  this->current_temperature = data[INDOOR_TEMPERATURE] - 40; // check later?

  // partially saving current state to previous request
  data_write_[MODE] = data[MODE];
  // add target temperature state too? ok
  data_write_[TEMPERATURE] = data[TEMPERATURE];

  // update CLIMATE state according AC response
  switch (data[MODE] & MODE_MASK) {
    case AC_MODE_OFF:
      this->mode = climate::CLIMATE_MODE_OFF;
      break;
    case AC_MODE_AUTO:
      this->mode = climate::CLIMATE_MODE_AUTO;
      break;
    case AC_MODE_COOL:
      this->mode = climate::CLIMATE_MODE_COOL;
      break;
    case AC_MODE_DRY:
      this->mode = climate::CLIMATE_MODE_DRY;
      break;
    case AC_MODE_FANONLY:
      this->mode = climate::CLIMATE_MODE_FAN_ONLY;
      break;
    case AC_MODE_HEAT:
      this->mode = climate::CLIMATE_MODE_HEAT;
      break;
    default:
      ESP_LOGW(TAG, "Unknown AC MODE&fan: %s", data[MODE]);
  }

  // by default, no preset
  this->preset = climate::CLIMATE_PRESET_NONE;

  if ((data[MODE] & FAN_MASK) > 7) {
    // SLEEP + fanmode, need to deduct 8
    data[MODE] = (uint8_t)data[MODE] - 8;
    this->preset = climate::CLIMATE_PRESET_SLEEP;
  }

  // get current AC FAN SPEED from its response
  switch (data[MODE] & FAN_MASK) {
    case AC_FAN_AUTO:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case AC_FAN_LOW:
      if (data[QUIET] == 8) {
          this->fan_mode = climate::CLIMATE_FAN_QUIET;
          break;
      }
        this->fan_mode = climate::CLIMATE_FAN_LOW;
        break;
    case AC_FAN_MEDIUM:
      if (data[EXTRAFAN] == AC_EXTRAFAN_MEDIUM_LOW) {
        this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
        break;
      }
      if (data[EXTRAFAN] == AC_EXTRAFAN_MEDIUM) {
        this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
        break;
      }
      // todo: more extrafan values? let's report MEDIUM
      this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      break;
    case AC_FAN_HIGH:
      if (data[EXTRAFAN] == AC_EXTRAFAN_MEDIUM_HIGH) {
        this->fan_mode = climate::CLIMATE_FAN_FOCUS;
        break;
      }
      if (data[EXTRAFAN] == AC_EXTRAFAN_HIGH) {
        this->fan_mode = climate::CLIMATE_FAN_HIGH;
        break;
      }
      // todo: more extrafan values? let's report HIGH
      this->fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    
    default:
      ESP_LOGW(TAG, "Unknown AC mode&FAN: %s", data[MODE]);
  }

  
  switch (data[SWING]) {
    case AC_SWING_OFF:
      this->swing_mode = climate::CLIMATE_SWING_OFF;
      break;

    case AC_SWING_VERTICAL:
      this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      break;

    case AC_SWING_HORIZONTAL:
      this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
      break;

    case AC_SWING_BOTH:
      this->swing_mode = climate::CLIMATE_SWING_BOTH;
      break;
  }
  

  switch (data[10]) {
    case 1:
      // when COOL TURBO
      this->preset = climate::CLIMATE_PRESET_BOOST;
      break;
    case 9:
      // when HEAT TURBO
      this->preset = climate::CLIMATE_PRESET_BOOST;
      break;
    default:
      break;
  }

  if ((data[SWING] = 0x50) && (data[QUIET] == 0X08)) {
      this->preset = climate::CLIMATE_PRESET_COMFORT;
  }

  this->publish_state();
}

void GreeUARTClimate::control(const climate::ClimateCall &call) {
  data_write_[FORCE_UPDATE] = 175;
  // show current temperature on display every time when sending new command. TEST!
  data_write_[13] = 0x20;
  

  // logging of saved mode&fan vars
/*
  char str[250] = {0};
  char *pstr = str;
  for (int i = 0; i < sizeof(data_save_); i++) {
    pstr += sprintf(pstr, "%02X ", data_save_[i]);
  }
  ESP_LOGV(TAG, "SAVED: %s", str);
*/

  // saving mode&fan values from previous 
  uint8_t new_mode = data_write_[MODE] & MODE_MASK;
  uint8_t new_fan_speed = data_write_[MODE] & FAN_MASK;

  if (call.get_mode().has_value()) {
    switch (call.get_mode().value()) {
      case climate::CLIMATE_MODE_OFF:
        new_mode = AC_MODE_OFF;
        break;
      case climate::CLIMATE_MODE_AUTO:
        new_mode = AC_MODE_AUTO;
        break;
      case climate::CLIMATE_MODE_COOL:
        new_mode = AC_MODE_COOL;
        break;
      case climate::CLIMATE_MODE_DRY:
        new_mode = AC_MODE_DRY;
        new_fan_speed = AC_FAN_LOW;
        break;
      case climate::CLIMATE_MODE_FAN_ONLY:
        new_mode = AC_MODE_FANONLY;
        break;
      case climate::CLIMATE_MODE_HEAT:
        new_mode = AC_MODE_HEAT;
        break;
      default:
        ESP_LOGW(TAG, "Setting of unsupported MODE: %s", call.get_mode().value());
        break;
    }
  }

  // set fan speed only if MODE != DRY (only LOW available)
  if (call.get_fan_mode().has_value()) {
    switch (call.get_fan_mode().value()) {
      case climate::CLIMATE_FAN_AUTO:
        new_fan_speed = AC_FAN_AUTO;
        data_write_[QUIET] = 0x00;
        break;
      case climate::CLIMATE_FAN_LOW:
        new_fan_speed = AC_FAN_LOW;
        data_write_[QUIET] = 0x00;
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        new_fan_speed = AC_FAN_MEDIUM;
        data_write_[EXTRAFAN] = AC_EXTRAFAN_MEDIUM_LOW;
        data_write_[QUIET] = 0x00;
        break;
      case climate::CLIMATE_FAN_MIDDLE:
        new_fan_speed = AC_FAN_MEDIUM;
        data_write_[EXTRAFAN] = AC_EXTRAFAN_MEDIUM;
        data_write_[QUIET] = 0x00;
        break;
      case climate::CLIMATE_FAN_FOCUS:
        new_fan_speed = AC_FAN_HIGH;
        data_write_[EXTRAFAN] = AC_EXTRAFAN_MEDIUM_HIGH;
        data_write_[QUIET] = 0x00;
        break;
      case climate::CLIMATE_FAN_HIGH:
        new_fan_speed = AC_FAN_HIGH;
        data_write_[EXTRAFAN] = AC_EXTRAFAN_HIGH;
        data_write_[QUIET] = 0x00;
        break;
      case climate::CLIMATE_FAN_QUIET:
        new_fan_speed = AC_FAN_LOW;
        data_write_[QUIET] = 0x08;
        break;
      default:
        ESP_LOGW(TAG, "Setting of unsupported FANSPEED: %s", call.get_fan_mode().value());
        break;
    }
  }
  
  // set low speed when DRY mode because other speeds are not available
  if (new_mode == AC_MODE_DRY && new_fan_speed != AC_FAN_LOW) {
    // ESP_LOGW(TAG, "DRY mode exception: %s", new_fan_speed);
    new_fan_speed = AC_FAN_LOW;
  }

  if (call.get_preset().has_value()) {
    switch (call.get_preset().value()) {
      case climate::CLIMATE_PRESET_NONE:
        if (new_mode == AC_MODE_COOL) {
          data_write_[10] = 0;
        } else if (new_mode == AC_MODE_HEAT) {
          data_write_[10] = 8;
        }
        data_write_[SWING] = 0x50;
        if (new_fan_speed > 7) {
            new_fan_speed = new_fan_speed - 8;
        }
        break;
      case climate::CLIMATE_PRESET_BOOST:
        if (new_mode == AC_MODE_COOL) {
          data_write_[10] = 1;
        } else if (new_mode == AC_MODE_HEAT) {
          data_write_[10] = 9;
        }
        new_fan_speed = AC_FAN_HIGH;
        // skip preset when not COOL or HEAT mode
        break;
      case climate::CLIMATE_PRESET_SLEEP:
          if (new_mode == AC_MODE_COOL) {
            data_write_[10] = 0;
          } else if (new_mode == AC_MODE_HEAT) {
            data_write_[10] = 8;
          }
          new_fan_speed = new_fan_speed + 8;
          data_write_[QUIET] = 0x00;
        // something
        break;
      case climate::CLIMATE_PRESET_COMFORT:
        // abusing this to get a fixed middle-low setting to heat the bed. Too bad esphome does not support more swing modes
          // turn off all presets (since this is not really a preset)
          if (new_mode == AC_MODE_COOL) {
            data_write_[10] = 0;
          } else if (new_mode == AC_MODE_HEAT) {
            data_write_[10] = 8;
          }
          data_write_[SWING] = 0x50;
          new_fan_speed = AC_FAN_LOW;
          // enables quiet mode
          data_write_[QUIET] = 0x08;
        break;
      default:
        // something?
        break;
    }
  }

/*
  if (call.get_preset().value() == climate::CLIMATE_PRESET_NONE) {
      data_[POWER] |= COMFORT_PRESET_MASK;
    } else {
      data_[POWER] &= ~COMFORT_PRESET_MASK;
    }
  }
*/

  if (call.get_target_temperature().has_value()) {
    // check if temperature set in valid limits
    if (call.get_target_temperature().value() >= MIN_VALID_TEMPERATURE && call.get_target_temperature().value() <= MAX_VALID_TEMPERATURE)
      data_write_[TEMPERATURE] = (call.get_target_temperature().value() - MIN_VALID_TEMPERATURE) * 16;
  }

  if (call.get_swing_mode().has_value()) {
    switch (call.get_swing_mode().value()) {
      case climate::CLIMATE_SWING_OFF:
           data_write_[SWING] = AC_SWING_OFF;
        break;
      case climate::CLIMATE_SWING_VERTICAL:
           data_write_[SWING] = AC_SWING_VERTICAL;
        break;
      case climate::CLIMATE_SWING_HORIZONTAL:
           data_write_[SWING] = AC_SWING_HORIZONTAL;
        break;
      case climate::CLIMATE_SWING_BOTH:
           data_write_[SWING] = AC_SWING_BOTH;
        break;
    }
  }

  data_write_[MODE] = new_mode + new_fan_speed;

  // compute checksum & send data
  data_write_[CRC_WRITE] = get_checksum_(data_write_, sizeof(data_write_));
  send_data_(data_write_, sizeof(data_write_));

  // change of force_update byte to "passive" state
  data_write_[FORCE_UPDATE] = 0;
}

void GreeUARTClimate::send_data_(const uint8_t *message, uint8_t size) {
  this->write_array(message, size);
  dump_message_("Sent message", message, size);
}

void GreeUARTClimate::dump_message_(const char *title, const uint8_t *message, uint8_t size) {
  ESP_LOGV(TAG, "%s:", title);
  char str[250] = {0};
  char *pstr = str;
  if (size * 2 > sizeof(str)) ESP_LOGE(TAG, "too long byte data");
  for (int i = 0; i < size; i++) {
    pstr += sprintf(pstr, "%02X ", message[i]);
  }
  ESP_LOGV(TAG, "%s", str);
}

uint8_t GreeUARTClimate::get_checksum_(const uint8_t *message, size_t size) {
  // position of crc in packet
  uint8_t position = size - 1;
  uint8_t sum = 0;
  // ignore first 2 bytes & last one
  for (int i = 2; i < position; i++)
    sum += message[i];
  uint8_t crc = sum % 256;
  return crc;
}

}  // namespace greeuart
}  // namespace esphome
