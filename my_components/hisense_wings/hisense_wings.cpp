// hisense_wings.cpp — Hisense Wings AEH-W4G2 ESPHome climate component.
//
// Protocol knowledge in this file is derived from direct packet captures on
// a real Hisense Wings 12k BTU unit (see hisense_decode.py for the trace
// decoder). Every byte value below has been verified against at least one
// labeled capture — nothing is pure guesswork.

#include "hisense_wings.h"
#include "esphome/core/log.h"

#include <string>

namespace esphome {
namespace hisense_wings {

static const char *const TAG = "hisense_wings";

static constexpr uint8_t H1 = 0xF4, H2 = 0xF5, F1 = 0xF4, F2 = 0xFB;

static constexpr uint32_t POLL_INTERVAL_MS = 8000;
static constexpr uint32_t SEND_COOLDOWN_MS = 300;

// Custom fan-mode labels shown in Home Assistant. ESPHome's built-in fan
// enum has no "medium-low"/"medium-high", so using it forced the UI to show
// the generic names "Focus" and "Middle". Custom modes let us label the six
// real speeds exactly as the AC exposes them.
static const char *const FAN_LOW_STR   = "Low";
static const char *const FAN_MLOW_STR  = "Low-Medium";
static const char *const FAN_MED_STR   = "Medium";
static const char *const FAN_MHIGH_STR = "Medium-High";
static const char *const FAN_HIGH_STR  = "High";

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

// 0x29 byte[18] low nibble — "power-on" / apply bits. HYPOTHESIS (unverified):
// the low nibble encodes the power transition, decoupled from the mode enum
// in the upper nibble:
//   0x00 = no power-state change (used to change mode WHILE already running)
//   0x04 = power OFF   (CMD_MODE_OFF = 0x04, upper nibble ignored)
//   0x0C = power ON / apply ("keepalive" value seen on the bus)
// A bare 0x50 (COOL, low nibble 0) changes mode when the AC is already on but
// is rejected from the OFF state — which matches the known power-on bug. So
// when turning the AC on from OFF we OR in this nibble, e.g. COOL => 0x5C.
static constexpr uint8_t CMD_MODE_POWER_ON_NIBBLE = 0x0C;

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
  // AUTO stays a built-in mode ("Auto"); the five real speeds are custom
  // modes so Home Assistant shows them by their proper names instead of the
  // generic built-in labels "Focus" / "Middle".
  t.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
  });
  t.set_supported_custom_fan_modes({
      FAN_LOW_STR,
      FAN_MLOW_STR,
      FAN_MED_STR,
      FAN_MHIGH_STR,
      FAN_HIGH_STR,
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

  // Keep sending while there are pending changes. Each call to
  // send_command_frame_() sets ONE field and clears its mask bit.
  // We throttle by SEND_COOLDOWN_MS so successive beeps don't overlap.
  if (pending_.mask != 0 && (now - last_send_ms_) > SEND_COOLDOWN_MS) {
    send_command_frame_();
    last_send_ms_ = now;
    if (pending_.mask == 0) dirty_ = false;
  } else if (pending_.mask == 0 && (now - last_poll_ms_) > POLL_INTERVAL_MS) {
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

  ESP_LOGD(TAG, "RX frame kind=0x%02X len=%u CRC OK", data[4], (unsigned) len);

  // We only decode the 0x7B full-state dump (132 bytes). Short polls and
  // response frames carry no useful state for us.
  if (len < 70 || data[4] != 0x7B) return;

  ESP_LOGD(TAG, "state frame: mode=0x%02X target=%u room=%u coil=%u",
           data[18], data[19], data[20], data[46]);

  // Copy into our status buffer for later reference
  const size_t copy_len = (len < sizeof(StatusFrame)) ? len : sizeof(StatusFrame);
  memcpy(&status_, data, copy_len);
  status_valid_ = true;
  publish_from_status_();
}

// ---------------------------------------------------------------------------
//  Is the AC currently running, per the last decoded state frame?
// ---------------------------------------------------------------------------
bool HisenseWings::ac_running_() const {
  if (!status_valid_) return false;
  const uint8_t *s = reinterpret_cast<const uint8_t *>(&status_);
  return (s[18] & STATE_MODE_RUNNING_BIT) != 0;
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
  // AUTO is a built-in fan mode; the five real speeds are custom modes. Set
  // exactly one of fan_mode / custom_fan_mode and clear the other so the UI
  // reflects a single, unambiguous selection.
  switch (s[16]) {
    case 0x01:
      this->fan_mode = climate::CLIMATE_FAN_AUTO; this->custom_fan_mode.reset();
      break;
    case 0x0A:
      this->custom_fan_mode = std::string(FAN_LOW_STR);   this->fan_mode.reset();
      break;
    case 0x0C:
      this->custom_fan_mode = std::string(FAN_MLOW_STR);  this->fan_mode.reset();
      break;
    case 0x0E:
      this->custom_fan_mode = std::string(FAN_MED_STR);   this->fan_mode.reset();
      break;
    case 0x10:
      this->custom_fan_mode = std::string(FAN_MHIGH_STR); this->fan_mode.reset();
      break;
    case 0x12:
      this->custom_fan_mode = std::string(FAN_HIGH_STR);  this->fan_mode.reset();
      break;
    default:
      // Includes the QUIET-only ultra-low value (0x02) and any unknown byte.
      this->fan_mode = climate::CLIMATE_FAN_AUTO; this->custom_fan_mode.reset();
      break;
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
    // Remember whether this change starts from OFF so the command frame can
    // carry the power-on nibble when waking the AC up.
    mode_from_off_ = (this->mode == climate::CLIMATE_MODE_OFF) && !ac_running_();
    this->mode = *call.get_mode();
    pending_.mask |= M_MODE;
    dirty_ = true;
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = *call.get_target_temperature();
    pending_.mask |= M_TEMP;
    dirty_ = true;
  }
  if (call.get_fan_mode().has_value()) {
    // Built-in fan mode (AUTO).
    this->fan_mode = *call.get_fan_mode();
    this->custom_fan_mode.reset();
    pending_.mask |= M_FAN;
    dirty_ = true;
  }
  if (call.get_custom_fan_mode().has_value()) {
    // One of our custom speed labels.
    this->custom_fan_mode = *call.get_custom_fan_mode();
    this->fan_mode.reset();
    pending_.mask |= M_FAN;
    dirty_ = true;
  }
  if (call.get_swing_mode().has_value()) {
    this->swing_mode = *call.get_swing_mode();
    pending_.mask |= M_SWING;
    dirty_ = true;
  }

  // Reflect the requested state in Home Assistant immediately. Without this,
  // the UI shows the request only optimistically and then snaps back to the
  // last published state until the next 0x7B poll confirms (or contradicts)
  // the change.
  this->publish_state();
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
//  CRITICAL: The AC rejects frames that set multiple fields at once. Each
//  0x29 command must set EXACTLY ONE field (plus byte 23 buzzer flag).
//  All other field bytes stay 0x00. This matches exactly what the Hisense
//  app does over the wire.
//
//  So each call here produces ONE field-change frame, picked by priority
//  from the pending mask. If the user changes multiple things at once in
//  the HA UI, we queue them: loop() will call this repeatedly, each call
//  sending one field, until the mask is empty.
// ---------------------------------------------------------------------------
void HisenseWings::send_command_frame_() {
  uint8_t f[46] = {
    0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01,
    0x00, 0x00, 0x65, 0x00, 0x00,   // [0..15] header
    0x00, 0x00, 0x00, 0x00,         // [16 fan][17 sleep][18 mode][19 temp]
    0x00, 0x00, 0x00, 0x00,         // [20..22][23 buzzer]
    0x00, 0x00, 0x00, 0x00,         // [24..27]
    0x00, 0x00, 0x00, 0x00,         // [28..31]
    0x00, 0x00, 0x00, 0x00,         // [32 swing][33 eco/boost][34][35 quiet]
    0x00, 0x00, 0x00, 0x00,         // [36 display][37..39]
    0x00, 0x00, 0x00, 0x00,         // [40..43]
    0x00, 0x00                      // [44..45]
  };

  // --- Buzzer (byte 23) — persistent flag, applied to every command.
  // Default: 0x04 (beep on receive). Mute switch flips it to 0x00.
  f[23] = mute_beep_ ? CMD_BUZZER_MUTE : CMD_BUZZER_BEEP;

  // --- Pick ONE field to send this frame, in priority order ---
  // MODE first (changing mode often resets other things so it should go first)
  if (pending_.mask & M_MODE) {
    uint8_t mode_byte;
    switch (this->mode) {
      case climate::CLIMATE_MODE_OFF:        mode_byte = CMD_MODE_OFF;      break;
      case climate::CLIMATE_MODE_FAN_ONLY:   mode_byte = CMD_MODE_FAN_ONLY; break;
      case climate::CLIMATE_MODE_HEAT:       mode_byte = CMD_MODE_HEAT;     break;
      case climate::CLIMATE_MODE_COOL:       mode_byte = CMD_MODE_COOL;     break;
      case climate::CLIMATE_MODE_DRY:        mode_byte = CMD_MODE_DRY;      break;
      case climate::CLIMATE_MODE_HEAT_COOL:  mode_byte = CMD_MODE_AUTO;     break;
      default:                               mode_byte = CMD_MODE_COOL;     break;
    }
    // Power-on wake: when switching from OFF into a running mode, OR in the
    // power-on nibble (e.g. COOL 0x50 -> 0x5C). A bare mode byte is accepted
    // only while the AC is already running; from OFF it is rejected (single
    // beep, no power-up). Leave OFF itself (0x04) untouched.
    if (this->mode != climate::CLIMATE_MODE_OFF && mode_from_off_) {
      mode_byte |= CMD_MODE_POWER_ON_NIBBLE;
    }
    f[18] = mode_byte;
    mode_from_off_ = false;
    pending_.mask &= ~M_MODE;
  } else if (pending_.mask & M_TEMP) {
    int t = static_cast<int>(this->target_temperature);
    if (t < 16) t = 16;
    if (t > 32) t = 32;
    f[19] = (static_cast<uint8_t>(t) << 1) | 1u;
    pending_.mask &= ~M_TEMP;
  } else if (pending_.mask & M_FAN) {
    uint8_t fan_byte = CMD_FAN_AUTO;
    if (this->custom_fan_mode.has_value()) {
      const std::string &c = *this->custom_fan_mode;
      if (c == FAN_LOW_STR)        fan_byte = CMD_FAN_LOW;
      else if (c == FAN_MLOW_STR)  fan_byte = CMD_FAN_MLOW;
      else if (c == FAN_MED_STR)   fan_byte = CMD_FAN_MED;
      else if (c == FAN_MHIGH_STR) fan_byte = CMD_FAN_MHIGH;
      else if (c == FAN_HIGH_STR)  fan_byte = CMD_FAN_HIGH;
    }
    // else: fan_mode is AUTO (or unset) -> CMD_FAN_AUTO
    f[16] = fan_byte;
    pending_.mask &= ~M_FAN;
  } else if (pending_.mask & M_SWING) {
    switch (this->swing_mode) {
      case climate::CLIMATE_SWING_BOTH:       f[32] = CMD_SWING_BOTH;  break;
      case climate::CLIMATE_SWING_VERTICAL:   f[32] = CMD_SWING_VERT;  break;
      case climate::CLIMATE_SWING_HORIZONTAL: f[32] = CMD_SWING_HORIZ; break;
      case climate::CLIMATE_SWING_OFF:
      default:                                f[32] = CMD_SWING_OFF;   break;
    }
    pending_.mask &= ~M_SWING;
  } else if (pending_.mask & M_SLEEP) {
    f[17] = pending_.sleep ? CMD_SLEEP_GENERAL : CMD_SLEEP_OFF;
    pending_.mask &= ~M_SLEEP;
  } else if (pending_.mask & (M_ECO | M_BOOST)) {
    // ECO and BOOST share byte 33 — handle both together if either changed.
    // Preserve the existing state for whichever isn't being changed right now.
    const bool want_eco   = (pending_.mask & M_ECO)   ? pending_.eco   : false;
    const bool want_boost = (pending_.mask & M_BOOST) ? pending_.boost : false;
    f[33] = (want_eco ? CMD_ECO_ON : CMD_ECO_OFF)
          | (want_boost ? CMD_BOOST_ON : CMD_BOOST_OFF);
    pending_.mask &= ~(M_ECO | M_BOOST);
  } else if (pending_.mask & M_QUIET) {
    f[35] = pending_.quiet ? CMD_QUIET_ON : CMD_QUIET_OFF;
    pending_.mask &= ~M_QUIET;
  } else if (pending_.mask & M_DISPLAY) {
    f[36] = pending_.display ? CMD_DISPLAY_ON : CMD_DISPLAY_OFF;
    pending_.mask &= ~M_DISPLAY;
  }
  // If pending_.mask is still non-zero after this call, loop() will send
  // another frame on the next tick (one field at a time).

  ESP_LOGD(TAG, "TX cmd: byte[16]=0x%02X byte[17]=0x%02X byte[18]=0x%02X "
                "byte[19]=0x%02X byte[23]=0x%02X byte[32]=0x%02X "
                "byte[33]=0x%02X byte[35]=0x%02X byte[36]=0x%02X mask_left=0x%X",
           f[16], f[17], f[18], f[19], f[23], f[32], f[33], f[35], f[36],
           (unsigned) pending_.mask);
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
  // Only publish if the state actually changed — prevents feedback loops
  // where ESPHome's state restoration or periodic refresh causes write_state
  // to be called with the same value and re-send a redundant command.
  if (initialized_ && state == this->state) return;
  initialized_ = true;

  switch (feature_id_) {
    case 0: parent_->set_disable_display(state); break;
    case 1: parent_->set_boost(state);     break;
    case 2: parent_->set_eco(state);       break;
    case 3: parent_->set_quiet(state);     break;
    case 4: parent_->set_sleep(state);     break;
    case 5: parent_->set_mute_beep(state); break;
  }
  this->publish_state(state);
}

}  // namespace hisense_wings
}  // namespace esphome
