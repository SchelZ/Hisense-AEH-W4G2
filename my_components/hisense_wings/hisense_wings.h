// hisense_wings.h
//
// Compact ESPHome climate component for Hisense Wings AC via AEH-W4G2 module.
// Optimized for RTL8710BN (LibreTiny) constrained flash.
//
// Design:
//   - Single .cpp + this header (better link-time dead-code elimination)
//   - Bit-packed status struct: compiler generates minimal accessor code
//   - Dynamic command builder: no precomputed frame tables (saves ~3.5KB
//     over akrabi's approach)
//   - No virtual dispatch beyond what ESPHome's Climate base class needs
//   - No STL exceptions / RTTI

#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#include <cstdint>
#include <cstring>

namespace esphome {
namespace hisense_wings {

// ---------------------------------------------------------------------------
//  Bit-packed status frame (52 bytes, exactly as it arrives on the wire).
//  This layout is derived from akrabi/hisense_ac_esphome's device_status.h,
//  cross-checked against kadam12g's bit-offset table. The AC transmits this
//  little-endian; RTL8710B is ARM little-endian, so no byte swapping needed.
//
//  Accessing e.g. `status_.indoor_temperature_status` compiles down to a
//  single load-byte instruction — no offset arithmetic, no shift+mask,
//  because the fields are aligned to byte boundaries wherever possible.
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct StatusFrame {
  uint8_t header[16];               // F4 F5 00 40 29 ... 65 00 00

  uint8_t fan_status;               // byte 16
  uint8_t sleep_status;             // byte 17

  uint8_t direction_status : 2;     // byte 18: vert swing dir
  uint8_t run_status : 2;           //         run/off state
  uint8_t mode_status : 4;          //         cool/heat/dry/fan/auto

  uint8_t indoor_temperature_setting; // byte 19 (target temp)
  uint8_t indoor_temperature_status;  // byte 20 (current temp)
  uint8_t indoor_pipe_temperature;    // byte 21

  int8_t  indoor_humidity_setting;    // byte 22
  int8_t  indoor_humidity_status;     // byte 23 (current humidity)

  uint8_t somatosensory_temperature;  // byte 24
  uint8_t soma_ctrl : 3;              // byte 25
  uint8_t soma_comp : 5;
  uint8_t temp_fahrenheit : 3;        // byte 26
  uint8_t temp_comp : 5;

  uint8_t timer;                      // byte 27
  uint8_t hour;                       // byte 28
  uint8_t minute;                     // byte 29
  uint8_t poweron_hour;               // byte 30
  uint8_t poweron_minute;             // byte 31
  uint8_t poweroff_hour;              // byte 32
  uint8_t poweroff_minute;            // byte 33

  uint8_t wind_door : 4;              // byte 34
  uint8_t drying : 4;

  // byte 35 — key feature flags
  uint8_t dual_frequency : 1;
  uint8_t efficient : 1;              // aka "boost"/"turbo"
  uint8_t low_electricity : 1;        // aka "eco"
  uint8_t low_power : 1;              // aka "quiet"
  uint8_t heat_flag : 1;
  uint8_t nature : 1;
  uint8_t left_right : 1;             // horizontal swing enabled
  uint8_t up_down : 1;                // vertical swing enabled

  // byte 36 — display / auxiliary flags
  uint8_t smoke : 1;
  uint8_t voice : 1;
  uint8_t mute : 1;
  uint8_t smart_eye : 1;
  uint8_t outdoor_clear : 1;
  uint8_t indoor_clear : 1;
  uint8_t swap : 1;
  uint8_t dew : 1;

  uint8_t byte37;                     // misc
  uint8_t byte38;                     // display_led lives here per akrabi
  uint8_t byte39;
  uint8_t byte40;

  uint8_t compressor_frequency;       // byte 41
  uint8_t compressor_freq_setting;    // byte 42
  uint8_t compressor_freq_send;       // byte 43
  int8_t  outdoor_temperature;        // byte 44
  int8_t  outdoor_condenser_temp;     // byte 45

  int8_t  compressor_exhaust_temp;    // byte 46
  int8_t  target_exhaust_temp;        // byte 47
  uint8_t expand_threshold;           // byte 48

  uint8_t pad[3];                     // reserved

  uint16_t checksum;                  // bytes 52-53
  uint8_t footer[2];                  // F4 FB
};
#pragma pack(pop)

static_assert(sizeof(StatusFrame) >= 40, "StatusFrame layout must not shrink");

// ---------------------------------------------------------------------------
//  Enums that map raw protocol values to human-meaningful states.
//  Kept as plain uint8_t to avoid any hidden RTTI/type-info overhead.
// ---------------------------------------------------------------------------
enum ProtoMode : uint8_t {
  MODE_FAN_ONLY = 0,
  MODE_HEAT = 1,
  MODE_COOL = 2,
  MODE_DRY = 3,
  MODE_AUTO = 4,
};

enum ProtoFan : uint8_t {
  FAN_AUTO  = 1,
  FAN_LOW   = 10,
  FAN_MLOW  = 12,
  FAN_MED   = 14,
  FAN_MHIGH = 16,
  FAN_HIGH  = 18,
};

// ---------------------------------------------------------------------------
//  Main component class.
//
//  Public API is intentionally minimal — everything the user might toggle
//  is exposed via one setter (bool for switches, enum for selects). Keeps
//  the vtable slim; the linker can then GC unused setters if nothing calls
//  them from user code.
// ---------------------------------------------------------------------------
class HisenseWings : public Component,
                     public uart::UARTDevice,
                     public climate::Climate {
 public:
  // ESPHome lifecycle
  void setup() override;
  void loop() override;
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration setters (called from generated code)
  void set_flow_control_pin(GPIOPin *pin) { flow_control_pin_ = pin; }
  void set_indoor_temperature_sensor(sensor::Sensor *s) { s_indoor_temp_ = s; }
  void set_outdoor_temperature_sensor(sensor::Sensor *s) { s_outdoor_temp_ = s; }
  void set_indoor_humidity_sensor(sensor::Sensor *s) { s_indoor_humid_ = s; }
  void set_indoor_pipe_temperature_sensor(sensor::Sensor *s) { s_indoor_pipe_ = s; }

  // Feature toggles (called from YAML lambdas)
  // Switch ON = display OFF, Switch OFF = display ON. Default (OFF) = display on.
  void set_disable_display(bool on) { pending_.display = on ? 0 : 1; pending_.mask |= M_DISPLAY; dirty_ = true; }
  void set_boost(bool on)     { pending_.boost = on ? 1 : 0;   pending_.mask |= M_BOOST;   dirty_ = true; }
  void set_eco(bool on)       { pending_.eco = on ? 1 : 0;     pending_.mask |= M_ECO;     dirty_ = true; }
  void set_quiet(bool on)     { pending_.quiet = on ? 1 : 0;   pending_.mask |= M_QUIET;   dirty_ = true; }
  void set_sleep(bool on)     { pending_.sleep = on ? 1 : 0;   pending_.mask |= M_SLEEP;   dirty_ = true; }
  // Switch ON = silent (mute), OFF = normal beeping. Default (OFF) = beeps.
  void set_mute_beep(bool on) { mute_beep_ = on; }

 protected:
  // ---- Incoming frame handling ----
  void process_byte_(uint8_t b);
  void handle_frame_(const uint8_t *data, size_t len);
  void publish_from_status_();

  // ---- Outgoing frame construction ----
  void send_status_request_();
  void send_command_frame_();
  uint16_t crc16_(const uint8_t *data, size_t len) const;
  void write_frame_(const uint8_t *data, size_t len);

  // True when the last decoded state frame shows the AC running (byte 18
  // bit 3). Used to decide whether a mode command needs the "power-on"
  // nibble. Returns false when we have never decoded a state frame.
  bool ac_running_() const;

  // ---- UART flow control (RS-485 DE/RE pin) ----
  void flow_tx_() { if (flow_control_pin_ != nullptr) flow_control_pin_->digital_write(true); }
  void flow_rx_() { if (flow_control_pin_ != nullptr) flow_control_pin_->digital_write(false); }

  // ---- State ----
  StatusFrame status_ {};      // last decoded frame from AC
  bool status_valid_ {false};

  // RX byte-stream state machine (no dynamic allocation).
  // RX_MAX must be large enough for the biggest frame we parse: the 0x7B
  // full-state dump is 132 bytes. A smaller buffer silently drops every
  // state frame (overflow-reset), so the climate entity never gets real
  // state back and the HA UI reverts to defaults.
  static constexpr size_t RX_MAX = 160;
  uint8_t rx_buf_[RX_MAX] {};
  size_t rx_len_ {0};
  uint8_t last_byte_ {0};

  // Pending command state. The AC rejects frames that set multiple fields
  // at once — each 0x29 command should set exactly ONE field. The mask
  // tracks which field changed since the last send.
  enum : uint16_t {
    M_DISPLAY = 1 << 0,
    M_BOOST   = 1 << 1,
    M_ECO     = 1 << 2,
    M_QUIET   = 1 << 3,
    M_SLEEP   = 1 << 4,
    M_MODE    = 1 << 6,
    M_TEMP    = 1 << 7,
    M_FAN     = 1 << 8,
    M_SWING   = 1 << 9,
  };
  struct Pending {
    uint16_t mask;
    uint8_t display : 1;
    uint8_t boost   : 1;
    uint8_t eco     : 1;
    uint8_t quiet   : 1;
    uint8_t sleep   : 1;
  } pending_ {};

  // Set in control() when a mode change starts from the OFF state, so the
  // next command frame can carry the power-on nibble (see send_command_frame_).
  bool mode_from_off_ {false};

  // Mute-beep is NOT a pending change — it's a persistent per-command flag
  // applied to every outgoing command. Default OFF = AC beeps normally.
  bool mute_beep_ {false};
  bool dirty_ {false};
  uint32_t last_poll_ms_ {0};
  uint32_t last_send_ms_ {0};


  // Hardware config
  GPIOPin *flow_control_pin_ {nullptr};

  // Sensor pointers (nullable — only wired up if configured in YAML)
  sensor::Sensor *s_indoor_temp_ {nullptr};
  sensor::Sensor *s_outdoor_temp_ {nullptr};
  sensor::Sensor *s_indoor_humid_ {nullptr};
  sensor::Sensor *s_indoor_pipe_ {nullptr};
};

// ---------------------------------------------------------------------------
//  FeatureSwitch — one class instance per exposed feature toggle
//  (display/boost/eco/quiet/sleep/beep). Compiles to ~40 bytes of code plus
//  a small vtable, per feature. Compare to a template switch which drags in
//  lambda plumbing per instance.
// ---------------------------------------------------------------------------
class FeatureSwitch : public esphome::switch_::Switch, public Component {
 public:
  void set_parent(HisenseWings *p) { parent_ = p; }
  void set_feature_id(uint8_t id) { feature_id_ = id; }
  void write_state(bool state) override;

 protected:
  HisenseWings *parent_ {nullptr};
  uint8_t feature_id_ {0};
  bool initialized_ {false};
};

}  // namespace hisense_wings
}  // namespace esphome
