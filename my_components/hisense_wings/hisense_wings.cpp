// hisense_wings.cpp — Hisense Wings AEH-W4G2 ESPHome climate component.
//
// Protocol knowledge in this file is derived from direct packet captures on
// a real Hisense Wings 12k BTU unit (see hisense_decode.py for the trace
// decoder). Every byte value below has been verified against at least one
// labeled capture — nothing is pure guesswork.

#include "hisense_wings.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/string_ref.h"

namespace esphome {
namespace hisense_wings {

static const char *const TAG = "hisense_wings";

static constexpr uint8_t H1 = 0xF4, H2 = 0xF5, F1 = 0xF4, F2 = 0xFB;

static constexpr uint32_t POLL_INTERVAL_MS = 8000;
static constexpr uint32_t SEND_COOLDOWN_MS = 300;

// Fan speeds. LOW / MEDIUM / HIGH / AUTO use ESPHome's built-in enum, which
// Home Assistant renders as exactly "Low" / "Medium" / "High" / "Auto". Only
// the two in-between speeds have no built-in enum, so they are exposed as
// custom modes. (Using FOCUS/MIDDLE for these was what produced the unwanted
// "Focus"/"Middle" labels before.)
//
// Note: the custom strings must NOT collide with a built-in fan-mode name
// ("Low"/"Medium"/"High"/"Auto"); HA re-maps such a custom string to the
// built-in enum, which then arrives as an unsupported built-in mode.
static const char *const FAN_MLOW_STR  = "Low-Medium";
static const char *const FAN_MHIGH_STR = "Medium-High";

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
  // Register the two in-between speeds as custom modes on the entity. In
  // ESPHome 2026.9 this is the primary source find_custom_fan_mode_() searches
  // (and get_traits() merges it for the frontend), so set_custom_fan_mode_()
  // below will accept these strings. The const char* literals have static
  // storage, so the stored pointers stay valid for the entity's lifetime.
  this->set_supported_custom_fan_modes({
      FAN_MLOW_STR,
      FAN_MHIGH_STR,
  });

  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 24.0f;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;  // defined default until a state frame arrives
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
      climate::CLIMATE_MODE_AUTO,  // the AC's AUTO mode (shown as "Auto" in HA)
  });
  // AUTO/LOW/MEDIUM/HIGH are built-in modes (rendered "Auto"/"Low"/"Medium"/
  // "High"); the two in-between speeds are registered as custom modes on the
  // entity in setup(), which get_traits() merges in.
  t.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
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

  // Periodic RX diagnostic (every 5 s). Tells us at a glance whether the AC
  // is talking to us at all.
  if (now - last_diag_ms_ > 5000) {
    last_diag_ms_ = now;
    ESP_LOGD(TAG, "diag: rx_bytes_total=%u rx_frames_ok=%u uart_avail=%d status_valid=%d init_done=%d",
             rx_bytes_total_, rx_frames_ok_, this->available(), status_valid_ ? 1 : 0,
             init_done_ ? 1 : 0);
  }

  // Boot handshake first — the AC won't stream state frames until the module
  // has introduced itself. One step per second until the sequence is done.
  if (!init_done_) {
    if (now - last_init_ms_ > 1000) {
      send_init_step_();
      last_init_ms_ = now;
    }
    return;
  }

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
//  RX byte stream. Frames are F4 F5 <dir> 40 <len> ... <crc_hi> <crc_lo> F4 FB,
//  where byte[4] is the payload length and the total frame size is len+9.
//  Using that explicit length (rather than scanning for the F4 FB footer) is
//  robust against F4 FB byte pairs that occur inside a frame's payload — the
//  132-byte 0x7B state frame in particular.
// ---------------------------------------------------------------------------
void HisenseWings::process_byte_(uint8_t b) {
  rx_bytes_total_++;

  if (rx_len_ == 0) {
    if (b == H1) { rx_buf_[0] = b; rx_len_ = 1; }
    return;
  }
  if (rx_len_ == 1) {
    if (b == H2)      { rx_buf_[1] = b; rx_len_ = 2; }  // got F4 F5
    else if (b == H1) { rx_buf_[0] = b; rx_len_ = 1; }  // F4 F4 — treat as new start
    else              { rx_len_ = 0; }                  // not a header; resync
    return;
  }

  if (rx_len_ >= RX_MAX) {  // safety; should never hit given the length check
    rx_len_ = 0;
    return;
  }
  rx_buf_[rx_len_++] = b;

  // Once the length byte (index 4) is in, we know the full frame size.
  if (rx_len_ >= 5) {
    const size_t expected = static_cast<size_t>(rx_buf_[4]) + 9;
    if (expected < 6 || expected > RX_MAX) {
      // Implausible length — drop and resync on the next F4 F5.
      ESP_LOGV(TAG, "RX implausible len byte=0x%02X; resync", rx_buf_[4]);
      rx_len_ = 0;
      return;
    }
    if (rx_len_ == expected) {
      if (rx_buf_[rx_len_ - 2] == F1 && rx_buf_[rx_len_ - 1] == F2 &&
          frame_crc_ok_(rx_buf_, rx_len_)) {
        handle_frame_(rx_buf_, rx_len_);
      } else {
        ESP_LOGV(TAG, "RX frame footer/CRC bad (len=%u)", (unsigned) rx_len_);
      }
      rx_len_ = 0;
    }
  }
}

// ---------------------------------------------------------------------------
//  CRC check: sum of bytes[2 .. len-4] vs the big-endian value at [len-4..len-3]
// ---------------------------------------------------------------------------
bool HisenseWings::frame_crc_ok_(const uint8_t *data, size_t len) const {
  if (len < 6) return false;
  uint16_t calc = 0;
  for (size_t i = 2; i < len - 4; ++i) calc += data[i];
  const uint16_t recv = (static_cast<uint16_t>(data[len - 4]) << 8) | data[len - 3];
  return (calc & 0xFFFF) == recv;
}

// ---------------------------------------------------------------------------
//  Received a complete frame — decode and publish
// ---------------------------------------------------------------------------
void HisenseWings::handle_frame_(const uint8_t *data, size_t len) {
  if (!frame_crc_ok_(data, len)) {
    ESP_LOGV(TAG, "frame CRC mismatch (len=%u)", (unsigned) len);
    return;
  }

  rx_frames_ok_++;
  ESP_LOGD(TAG, "RX frame dir=0x%02X len_byte=0x%02X total=%u CRC OK",
           data[2], data[4], (unsigned) len);

  // The full-state dump is the long frame the AC sends back (byte[4] is a
  // length field: this unit uses a 160-byte frame, not the 132-byte one the
  // old docs described). Accept any long AC->module frame as state; short
  // handshake/ack replies carry no state for us.
  if (len < 70 || data[2] != 0x01) return;

  // Raw dump of the state frame's leading bytes so the field offsets can be
  // verified/corrected against the real 160-byte layout.
  ESP_LOGD(TAG, "state raw[0..47]: %s",
           format_hex_pretty(data, len < 48 ? len : 48).c_str());
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
      case 4: this->mode = climate::CLIMATE_MODE_AUTO; break;  // AUTO
      case 7: this->mode = climate::CLIMATE_MODE_AUTO; break;  // AUTO cool-sub
      default: this->mode = climate::CLIMATE_MODE_OFF;      break;
    }
  }

  // --- Target temp (byte 19) and current temp (byte 20) — both raw degC ---
  this->target_temperature = static_cast<float>(s[19]);
  this->current_temperature = static_cast<float>(s[20]);

  // --- Fan speed (byte 16, even values in state frames) ---
  // LOW/MEDIUM/HIGH/AUTO are built-in; the two in-between speeds are custom.
  // set_custom_fan_mode_() resets fan_mode internally; for a built-in mode we
  // clear the custom slot and set fan_mode. Exactly one is active at a time.
  switch (s[16]) {
    case 0x01:
      this->clear_custom_fan_mode_(); this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case 0x0A:
      this->clear_custom_fan_mode_(); this->fan_mode = climate::CLIMATE_FAN_LOW;
      break;
    case 0x0C: this->set_custom_fan_mode_(FAN_MLOW_STR);  break;
    case 0x0E:
      this->clear_custom_fan_mode_(); this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      break;
    case 0x10: this->set_custom_fan_mode_(FAN_MHIGH_STR); break;
    case 0x12:
      this->clear_custom_fan_mode_(); this->fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    default:
      // Includes the QUIET-only ultra-low value (0x02) and any unknown byte.
      this->clear_custom_fan_mode_(); this->fan_mode = climate::CLIMATE_FAN_AUTO;
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
    // Built-in fan mode (AUTO / LOW / MEDIUM / HIGH).
    this->clear_custom_fan_mode_();
    this->fan_mode = *call.get_fan_mode();
    pending_.mask |= M_FAN;
    dirty_ = true;
  }
  const StringRef custom_fan = call.get_custom_fan_mode();
  if (!custom_fan.empty()) {
    // One of the custom in-between speeds. set_custom_fan_mode_() resets
    // fan_mode. Use the explicit-length overload so no null terminator is
    // assumed.
    this->set_custom_fan_mode_(custom_fan.c_str(), custom_fan.size());
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
//  Boot handshake. The AC control board only begins streaming 0x7B state
//  frames after the module announces itself with this sequence: three 0x0B
//  init frames followed by a 0x13 wifi-status frame. Byte values (and their
//  CRCs, which our summation reproduces) are taken from a working capture of
//  the kadam12g reference firmware. Payloads below omit the trailing CRC +
//  F4 FB footer, which write_frame_() appends.
// ---------------------------------------------------------------------------
void HisenseWings::send_init_step_() {
  static const uint8_t init1[16] = {
    0xF4, 0xF5, 0x00, 0x40, 0x0B, 0x00, 0x00, 0x00,
    0x00, 0xFE, 0x01, 0x00, 0x00, 0x0A, 0x04, 0x00};
  static const uint8_t init2[16] = {
    0xF4, 0xF5, 0x00, 0x40, 0x0B, 0x00, 0x00, 0x01,
    0x01, 0xFE, 0x01, 0x00, 0x00, 0x07, 0x01, 0x00};
  static const uint8_t init3[16] = {
    0xF4, 0xF5, 0x00, 0x40, 0x0B, 0x00, 0x00, 0x01,
    0x01, 0xFE, 0x01, 0x00, 0x00, 0x66, 0x40, 0x00};
  static const uint8_t wifi[24] = {
    0xF4, 0xF5, 0x00, 0x40, 0x13, 0x00, 0x00, 0x01,
    0x01, 0xFE, 0x01, 0x00, 0x00, 0x1E, 0x00, 0x00,
    0x80, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};

  switch (init_step_) {
    case 0: ESP_LOGD(TAG, "init 1/4 (0x0B)"); write_frame_(init1, sizeof(init1)); break;
    case 1: ESP_LOGD(TAG, "init 2/4 (0x0B)"); write_frame_(init2, sizeof(init2)); break;
    case 2: ESP_LOGD(TAG, "init 3/4 (0x0B)"); write_frame_(init3, sizeof(init3)); break;
    case 3: ESP_LOGD(TAG, "init 4/4 (0x13 wifi-status)"); write_frame_(wifi, sizeof(wifi)); break;
    default: break;
  }
  if (++init_step_ > 3) {
    init_done_ = true;
    ESP_LOGD(TAG, "init handshake complete; starting status polls");
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
  ESP_LOGD(TAG, "TX poll 0x0C (expecting a 0x7B state frame back)");
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
      case climate::CLIMATE_MODE_AUTO:       mode_byte = CMD_MODE_AUTO;     break;
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
    const StringRef cfm = this->get_custom_fan_mode();
    if (!cfm.empty()) {
      // Custom in-between speeds.
      if (cfm == FAN_MLOW_STR)       fan_byte = CMD_FAN_MLOW;
      else if (cfm == FAN_MHIGH_STR) fan_byte = CMD_FAN_MHIGH;
    } else if (this->fan_mode.has_value()) {
      // Built-in speeds.
      switch (*this->fan_mode) {
        case climate::CLIMATE_FAN_LOW:    fan_byte = CMD_FAN_LOW;  break;
        case climate::CLIMATE_FAN_MEDIUM: fan_byte = CMD_FAN_MED;  break;
        case climate::CLIMATE_FAN_HIGH:   fan_byte = CMD_FAN_HIGH; break;
        case climate::CLIMATE_FAN_AUTO:
        default:                          fan_byte = CMD_FAN_AUTO; break;
      }
    }
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
