// hisense_wings.cpp — Hisense Wings AEH-W4G2 ESPHome climate component.
//
// Protocol knowledge in this file is derived from direct packet captures on
// a real Hisense Wings 12k BTU unit (see hisense_decode.py for the trace
// decoder). Every byte value below has been verified against at least one
// labeled capture — nothing is pure guesswork.

#include "hisense_wings.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hisense_wings {

static const char *const TAG = "hisense_wings";

static constexpr uint8_t H1 = 0xF4, H2 = 0xF5, F1 = 0xF4, F2 = 0xFB;

static constexpr uint32_t POLL_INTERVAL_MS = 8000;
static constexpr uint32_t SEND_COOLDOWN_MS = 300;

// ---------------------------------------------------------------------------
//  Protocol constants — command byte values, derived from labeled captures.
// ---------------------------------------------------------------------------

// 0x29 byte[18] — mode command. Formula: ((mode<<1)|1)<<4
// Lower nibble is zero on this AC (contrary to pio2398 which uses 0x0C).
static constexpr uint8_t CMD_MODE_OFF      = 0x04;
static constexpr uint8_t CMD_MODE_FAN_ONLY = 0x10;
static constexpr uint8_t CMD_MODE_HEAT     = 0x30;
static constexpr uint8_t CMD_MODE_COOL     = 0x50;
static constexpr uint8_t CMD_MODE_DRY      = 0x70;
static constexpr uint8_t CMD_MODE_AUTO     = 0x90;

// 0x29 byte[16] — fan speed command (odd values with LSB=1 confirmation bit)
static constexpr uint8_t CMD_FAN_AUTO   = 0x01;
static constexpr uint8_t CMD_FAN_LOW    = 0x0B;
static constexpr uint8_t CMD_FAN_MLOW   = 0x0D;
static constexpr uint8_t CMD_FAN_MED    = 0x0F;
static constexpr uint8_t CMD_FAN_MHIGH  = 0x11;
static constexpr uint8_t CMD_FAN_HIGH   = 0x13;

// 0x29 byte[17] — sleep profile command (odd values)
static constexpr uint8_t CMD_SLEEP_OFF     = 0x01;
static constexpr uint8_t CMD_SLEEP_GENERAL = 0x03;
// 0x05 old / 0x07 young / 0x09 kids — not exposed via climate interface

// 0x29 byte[23] — buzzer control
static constexpr uint8_t CMD_BUZZER_BEEP = 0x04;
static constexpr uint8_t CMD_BUZZER_MUTE = 0x00;

// 0x29 byte[32] — swing command (confirmed: V. H and BOTH inferred.)
static constexpr uint8_t CMD_SWING_OFF    = 0x40;  // confirmed
static constexpr uint8_t CMD_SWING_VERT   = 0xC0;  // confirmed
static constexpr uint8_t CMD_SWING_HORIZ  = 0x70;  // from reference, unverified
static constexpr uint8_t CMD_SWING_BOTH   = 0xF0;  // from reference, unverified

// 0x29 byte[33] — ECO (upper nibble) + BOOST (lower nibble). Can combine.
static constexpr uint8_t CMD_ECO_OFF   = 0x10;
static constexpr uint8_t CMD_ECO_ON    = 0x30;
static constexpr uint8_t CMD_BOOST_OFF = 0x04;
static constexpr uint8_t CMD_BOOST_ON  = 0x0C;

// 0x29 byte[35] — quiet
static constexpr uint8_t CMD_QUIET_OFF = 0x10;
static constexpr uint8_t CMD_QUIET_ON  = 0x30;

// 0x29 byte[36] — display
static constexpr uint8_t CMD_DISPLAY_OFF = 0x40;
static constexpr uint8_t CMD_DISPLAY_ON  = 0xC0;

// 0x7B state byte[18] — mode state. Upper nibble = mode enum, bit 3 = running.
static constexpr uint8_t STATE_MODE_UPPER_MASK  = 0xF0;
static constexpr uint8_t STATE_MODE_RUNNING_BIT = 0x08;
// Note: AUTO uses upper nibble 7 (0x78) when in cool sub-mode.

// 0x7B state byte[35] — feature bitfield
static constexpr uint8_t STATE_FEAT_V_SWING = 0x80;
static constexpr uint8_t STATE_FEAT_H_SWING = 0x40;
static constexpr uint8_t STATE_FEAT_ECO     = 0x04;
static constexpr uint8_t STATE_FEAT_BOOST   = 0x02;

// 0x7B state byte[36] — more features
static constexpr uint8_t STATE_FEAT_QUIET   = 0x04;

// ---------------------------------------------------------------------------
//  Setup / traits
// ---------------------------------------------------------------------------
void HisenseWings::setup() {
  if (flow_control_pin_ != nullptr) {
    flow_control_pin_->setup();
    flow_control_pin_->digital_write(false);  // start in RX mode
  }
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 24.0f;
  this->publish_state();
}

climate::ClimateTraits HisenseWings::traits() {
  auto t = climate::ClimateTraits();
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(32);
  t.set_visual_temperature_step(1);
  t.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_HEAT_COOL,  // = AUTO
  });
  t.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_FOCUS,   // MED-LOW
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_MIDDLE,  // MED-HIGH
      climate::CLIMATE_FAN_HIGH,
  });
  t.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
      climate::CLIMATE_SWING_BOTH,
  });
  // Note: set_supports_current_temperature() was removed in newer ESPHome;
  // current_temperature is published implicitly whenever we call publish_state()
  // after assigning this->current_temperature.
  return t;
}

// ---------------------------------------------------------------------------
//  Main loop — read UART, respond to pending commands, poll periodically
// ---------------------------------------------------------------------------
void HisenseWings::loop() {
  while (this->available() > 0) {
    uint8_t b;
    if (this->read_byte(&b)) process_byte_(b);
  }

  const uint32_t now = millis();

  if (dirty_ && (now - last_send_ms_) > SEND_COOLDOWN_MS) {
    send_command_frame_();
    dirty_ = false;
    pending_.mask = 0;
    last_send_ms_ = now;
  } else if ((now - last_poll_ms_) > POLL_INTERVAL_MS) {
    send_status_request_();
    last_poll_ms_ = now;
  }
}

// ---------------------------------------------------------------------------
//  RX byte stream: look for F4 F5 ... F4 FB frames
// ---------------------------------------------------------------------------
void HisenseWings::process_byte_(uint8_t b) {
  // State machine: accumulate bytes starting at F4 F5, end at F4 FB
  if (rx_len_ == 0) {
    if (b == H1) {
      rx_buf_[0] = b;
      rx_len_ = 1;
    }
    last_byte_ = b;
    return;
  }
  if (rx_len_ == 1) {
    if (b == H2) {
      rx_buf_[1] = b;
      rx_len_ = 2;
    } else {
      rx_len_ = 0;
    }
    last_byte_ = b;
    return;
  }
  // Mid-frame: watch for F4 FB footer
  if (rx_len_ < RX_MAX) {
    rx_buf_[rx_len_++] = b;
  } else {
    // Overflow — reset
    rx_len_ = 0;
    last_byte_ = b;
    return;
  }
  if (last_byte_ == F1 && b == F2) {
    handle_frame_(rx_buf_, rx_len_);
    rx_len_ = 0;
  }
  last_byte_ = b;
}

// ---------------------------------------------------------------------------
//  Received a complete frame — decode and publish
// ---------------------------------------------------------------------------
void HisenseWings::handle_frame_(const uint8_t *data, size_t len) {
  if (len < 6) return;

  // Verify CRC: sum of bytes[2 .. len-4], stored big-endian at [len-4],[len-3]
  uint16_t calc = 0;
  for (size_t i = 2; i < len - 4; ++i) calc += data[i];
  uint16_t recv = (static_cast<uint16_t>(data[len - 4]) << 8) | data[len - 3];
  if ((calc & 0xFFFF) != recv) {
    ESP_LOGV(TAG, "frame CRC mismatch calc=%04X recv=%04X", calc & 0xFFFF, recv);
    return;
  }

  // We only decode the 0x7B full-state dump (132 bytes). Short polls and
  // response frames carry no useful state for us.
  if (len < 70 || data[4] != 0x7B) return;

  // Copy into our status buffer for later reference
  const size_t copy_len = (len < sizeof(StatusFrame)) ? len : sizeof(StatusFrame);
  memcpy(&status_, data, copy_len);
  status_valid_ = true;
  publish_from_status_();
}

// ---------------------------------------------------------------------------
//  Publish decoded state to ESPHome climate entity + sensors
// ---------------------------------------------------------------------------
void HisenseWings::publish_from_status_() {
  const uint8_t *s = reinterpret_cast<const uint8_t *>(&status_);

  // --- Mode (byte 18) ---
  const uint8_t m18 = s[18];
  const bool running = (m18 & STATE_MODE_RUNNING_BIT) != 0;
  const uint8_t mode_nibble = (m18 & STATE_MODE_UPPER_MASK) >> 4;

  if (!running) {
    this->mode = climate::CLIMATE_MODE_OFF;
  } else {
    switch (mode_nibble) {
      case 0: this->mode = climate::CLIMATE_MODE_FAN_ONLY;  break;
      case 1: this->mode = climate::CLIMATE_MODE_HEAT;      break;
      case 2: this->mode = climate::CLIMATE_MODE_COOL;      break;
      case 3: this->mode = climate::CLIMATE_MODE_DRY;       break;
      case 4: this->mode = climate::CLIMATE_MODE_HEAT_COOL; break;  // AUTO
      case 7: this->mode = climate::CLIMATE_MODE_HEAT_COOL; break;  // AUTO cool-sub
      default: this->mode = climate::CLIMATE_MODE_OFF;      break;
    }
  }

  // --- Target temp (byte 19) and current temp (byte 20) — both raw degC ---
  this->target_temperature = static_cast<float>(s[19]);
  this->current_temperature = static_cast<float>(s[20]);

  // --- Fan speed (byte 16, even values in state frames) ---
  switch (s[16]) {
    case 0x01: this->fan_mode = climate::CLIMATE_FAN_AUTO;   break;
    case 0x0A: this->fan_mode = climate::CLIMATE_FAN_LOW;    break;
    case 0x0C: this->fan_mode = climate::CLIMATE_FAN_FOCUS;  break;  // ML
    case 0x0E: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case 0x10: this->fan_mode = climate::CLIMATE_FAN_MIDDLE; break;  // MH
    case 0x12: this->fan_mode = climate::CLIMATE_FAN_HIGH;   break;
    default:   this->fan_mode = climate::CLIMATE_FAN_AUTO;   break;
  }

  // --- Swing (byte 35 bitfield) ---
  const uint8_t feat_a = s[35];
  const bool v_sw = (feat_a & STATE_FEAT_V_SWING) != 0;
  const bool h_sw = (feat_a & STATE_FEAT_H_SWING) != 0;
  if (v_sw && h_sw)      this->swing_mode = climate::CLIMATE_SWING_BOTH;
  else if (v_sw)         this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
  else if (h_sw)         this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  else                   this->swing_mode = climate::CLIMATE_SWING_OFF;

  this->publish_state();

  // --- Sensors ---
  if (s_indoor_temp_ != nullptr)
    s_indoor_temp_->publish_state(static_cast<float>(s[20]));
  if (s_outdoor_temp_ != nullptr)
    s_outdoor_temp_->publish_state(static_cast<float>(s[46]));
  // No reliable humidity byte identified on this AC — s_indoor_humid_ unused.
  // Pipe/coil temp = byte 46 (same as outdoor_temp sensor), optional.
}

// ---------------------------------------------------------------------------
//  Climate control: user changed something in the frontend
// ---------------------------------------------------------------------------
void HisenseWings::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    this->mode = *call.get_mode();
    dirty_ = true;
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    dirty_ = true;
  }
  if (call.get_fan_mode().has_value()) {
    this->fan_mode = *call.get_fan_mode();
    dirty_ = true;
  }
  if (call.get_swing_mode().has_value()) {
    this->swing_mode = *call.get_swing_mode();
    dirty_ = true;
  }
}

// ---------------------------------------------------------------------------
//  Send a status-request poll frame (17 bytes payload + CRC + footer)
// ---------------------------------------------------------------------------
void HisenseWings::send_status_request_() {
  static const uint8_t frame[17] = {
    0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01, 0x01,
    0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00
  };
  write_frame_(frame, sizeof(frame));
}

// ---------------------------------------------------------------------------
//  Build and send a command frame.
//
//  46-byte payload. Each command sets ONE field at a time (that's how the
//  Hisense app operates), but ESPHome's model is "apply current state", so
//  we send a frame with the field(s) that changed. Other feature flags are
//  carried in from the last known status or last pending state.
// ---------------------------------------------------------------------------
void HisenseWings::send_command_frame_() {
  uint8_t f[46] = {
    0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01,
    0x00, 0x00, 0x65, 0x00, 0x00,   // [0..15] header
    0x00, 0x00, 0x00, 0x00,         // [16 fan][17 sleep][18 mode][19 temp]
    0x00, 0x00, 0x00, 0x00,         // [20..22][23 buzzer] -- buzzer set below
    0x00, 0x00, 0x00, 0x00,         // [24..27]
    0x00, 0x00, 0x00, 0x00,         // [28..31]
    0x00, 0x00, 0x00, 0x00,         // [32 swing][33 eco/boost][34][35 quiet]
    0x00, 0x00, 0x00, 0x00,         // [36 display][37..39]
    0x00, 0x00, 0x00, 0x00,         // [40..43]
    0x00, 0x00                      // [44..45]
  };

  // --- Mode (byte 18) ---
  switch (this->mode) {
    case climate::CLIMATE_MODE_OFF:        f[18] = CMD_MODE_OFF;      break;
    case climate::CLIMATE_MODE_FAN_ONLY:   f[18] = CMD_MODE_FAN_ONLY; break;
    case climate::CLIMATE_MODE_HEAT:       f[18] = CMD_MODE_HEAT;     break;
    case climate::CLIMATE_MODE_COOL:       f[18] = CMD_MODE_COOL;     break;
    case climate::CLIMATE_MODE_DRY:        f[18] = CMD_MODE_DRY;      break;
    case climate::CLIMATE_MODE_HEAT_COOL:  f[18] = CMD_MODE_AUTO;     break;
    default:                               f[18] = CMD_MODE_COOL;     break;
  }

  // --- Target temp (byte 19): (temp << 1) | 1 ---
  {
    int t = static_cast<int>(this->target_temperature);
    if (t < 16) t = 16;
    if (t > 32) t = 32;
    f[19] = (static_cast<uint8_t>(t) << 1) | 1u;
  }

  // --- Fan speed (byte 16): direct command values ---
  switch (this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO)) {
    case climate::CLIMATE_FAN_AUTO:   f[16] = CMD_FAN_AUTO;  break;
    case climate::CLIMATE_FAN_LOW:    f[16] = CMD_FAN_LOW;   break;
    case climate::CLIMATE_FAN_FOCUS:  f[16] = CMD_FAN_MLOW;  break;
    case climate::CLIMATE_FAN_MEDIUM: f[16] = CMD_FAN_MED;   break;
    case climate::CLIMATE_FAN_MIDDLE: f[16] = CMD_FAN_MHIGH; break;
    case climate::CLIMATE_FAN_HIGH:   f[16] = CMD_FAN_HIGH;  break;
    default:                          f[16] = CMD_FAN_AUTO;  break;
  }

  // --- Swing (byte 32) ---
  switch (this->swing_mode) {
    case climate::CLIMATE_SWING_BOTH:       f[32] = CMD_SWING_BOTH;  break;
    case climate::CLIMATE_SWING_VERTICAL:   f[32] = CMD_SWING_VERT;  break;
    case climate::CLIMATE_SWING_HORIZONTAL: f[32] = CMD_SWING_HORIZ; break;
    case climate::CLIMATE_SWING_OFF:
    default:                                f[32] = CMD_SWING_OFF;   break;
  }

  // --- Feature flags: byte 33 (ECO+BOOST), byte 35 (QUIET), byte 36 (display)
  // Default to OFF-base values if the user hasn't touched them this frame,
  // so the AC doesn't interpret 0x00 as a feature state.
  const bool want_eco     = (pending_.mask & M_ECO)     ? pending_.eco     : false;
  const bool want_boost   = (pending_.mask & M_BOOST)   ? pending_.boost   : false;
  const bool want_quiet   = (pending_.mask & M_QUIET)   ? pending_.quiet   : false;
  const bool want_display = (pending_.mask & M_DISPLAY) ? pending_.display : true;

  f[33] = (want_eco ? CMD_ECO_ON : CMD_ECO_OFF)
        | (want_boost ? CMD_BOOST_ON : CMD_BOOST_OFF);

  f[35] = want_quiet ? CMD_QUIET_ON : CMD_QUIET_OFF;

  f[36] = want_display ? CMD_DISPLAY_ON : CMD_DISPLAY_OFF;

  // --- Sleep (byte 17) ---
  if (pending_.mask & M_SLEEP) {
    f[17] = pending_.sleep ? CMD_SLEEP_GENERAL : CMD_SLEEP_OFF;
  } else {
    f[17] = CMD_SLEEP_OFF;
  }

  // --- Buzzer (byte 23) ---
  // Mute if the user has turned the beep switch OFF; otherwise beep.
  const bool want_beep = (pending_.mask & M_BEEP) ? pending_.beep : beep_enabled_;
  if (pending_.mask & M_BEEP) beep_enabled_ = want_beep;
  f[23] = want_beep ? CMD_BUZZER_BEEP : CMD_BUZZER_MUTE;

  write_frame_(f, sizeof(f));
}

// ---------------------------------------------------------------------------
//  CRC-16 (byte-wise sum, big-endian storage done in write_frame_)
// ---------------------------------------------------------------------------
uint16_t HisenseWings::crc16_(const uint8_t *data, size_t len) const {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; ++i) sum += data[i];
  return sum;
}

// ---------------------------------------------------------------------------
//  Frame writer: payload in, append CRC (big-endian) + F4 FB, send with
//  RS-485 flow control.
// ---------------------------------------------------------------------------
void HisenseWings::write_frame_(const uint8_t *data, size_t len) {
  const uint16_t crc = crc16_(&data[2], len - 2);
  const uint8_t cr1 = (crc >> 8) & 0xFF;  // high byte first (big-endian)
  const uint8_t cr2 = crc & 0xFF;

  flow_tx_();
  this->write_array(data, len);
  this->write_byte(cr1);
  this->write_byte(cr2);
  this->write_byte(0xF4);
  this->write_byte(0xFB);
  this->flush();
  flow_rx_();
}

// ---------------------------------------------------------------------------
//  FeatureSwitch::write_state — dispatch by feature id
// ---------------------------------------------------------------------------
void FeatureSwitch::write_state(bool state) {
  if (parent_ == nullptr) return;
  switch (feature_id_) {
    case 0: parent_->set_display(state); break;
    case 1: parent_->set_boost(state);   break;
    case 2: parent_->set_eco(state);     break;
    case 3: parent_->set_quiet(state);   break;
    case 4: parent_->set_sleep(state);   break;
    case 5: parent_->set_beep(state);    break;
  }
  this->publish_state(state);
}

}  // namespace hisense_wings
}  // namespace esphome
