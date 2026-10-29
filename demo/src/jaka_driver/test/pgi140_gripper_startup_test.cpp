#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "jaka_driver/pgi140_gripper.hpp"
#include "jaka_driver/pgi140_protocol.hpp"

// Fake definitions for the SDK declarations used by the REAL gripper source.
// This executable never links libjakaAPI or creates a device connection.
namespace fake {
struct Write { uint16_t address, value; };
std::vector<Write> writes;
int physical_position = 350;
int pin_calls = 0, serial_calls = 0, power_calls = 0;
int next_return = 0;
void check(bool value, const std::string &message) {
  if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
void reset() {
  writes.clear(); pin_calls = serial_calls = power_calls = 0;
  physical_position = 350;
  next_return = 0;
}
int count(uint16_t address) {
  int n = 0;
  for (const auto &w : writes) if (w.address == address) ++n;
  return n;
}
void dump() {
  std::cout << "Decoded TIO writes:";
  for (const auto &w : writes)
    std::cout << " [0x" << std::hex << w.address << std::dec << "=" << w.value << "]";
  std::cout << " physical_position=" << physical_position << '\n';
}
pgi140::GripperConfig config() {
  pgi140::GripperConfig c;
  c.initialize = false;
  c.initialize_delay_ms = 0;
  c.configure_tio = true;
  c.open_position = 1000;
  c.closed_position = 0;
  c.force_percent = 25;
  c.speed_percent = 40;
  return c;
}
void preserve() {
  reset(); JAKAZuRobot robot;
  for (int restart = 0; restart < 2; ++restart) {
    pgi140::Pgi140Gripper gripper(robot, config());
    check(gripper.initialize(), "communication configuration succeeds");
    check(!gripper.has_position_command(), "restart does not invent a gripper position state");
  }
  dump();
  check(count(0x0100) == 0, "restart without calibration must not write initialization register");
  check(count(0x0103) == 0, "restart with initialize=false must not write open/position register");
  check(physical_position == 350, "two restarts preserve an already loaded gripper position");
  check(writes.empty(), "preserve startup must not alter held-load force or speed settings either");
  check(pin_calls == 2 && serial_calls == 4, "communication configuration remains available");
}
void default_preserves() {
  reset(); JAKAZuRobot robot;
  pgi140::GripperConfig c;
  c.initialize_delay_ms = 0;
  pgi140::Pgi140Gripper gripper(robot, c);
  check(gripper.initialize(), "default communication configuration succeeds");
  dump();
  check(count(0x0100) == 0, "default startup must not command physical calibration");
  check(count(0x0103) == 0, "default startup must not command active opening");
  check(writes.empty(), "default startup contains no motion/calibration Modbus writes");
}
void calibration_only() {
  reset(); JAKAZuRobot robot; auto c = config();
  c.initialize = true;
  c.initialize_command = 0xA5;
  pgi140::Pgi140Gripper gripper(robot, c);
  check(gripper.initialize(), "explicit calibration succeeds"); dump();
  check(writes.size() == 1 && writes[0].address == 0x0100 && writes[0].value == 0xA5,
    "explicit calibration writes only initialization register with requested value");
  check(!gripper.has_position_command(), "calibration is not claimed to prove any position");
}
void motion_parameters_only() {
  reset(); JAKAZuRobot robot; auto c = config();
  c.configure_motion_parameters = true;
  pgi140::Pgi140Gripper gripper(robot, c);
  check(gripper.initialize(), "explicit force/speed configuration succeeds"); dump();
  check(writes.size() == 2 && writes[0].address == 0x0101 && writes[0].value == 25 &&
    writes[1].address == 0x0104 && writes[1].value == 40,
    "motion-parameter opt-in writes force and speed, never calibration or opening");
  check(!gripper.has_position_command(), "force/speed configuration does not invent position");
}
void explicit_open() {
  reset(); JAKAZuRobot robot; auto c = config();
  c.startup_open = true;
  pgi140::Pgi140Gripper gripper(robot, c);
  check(gripper.initialize(), "explicit startup opening succeeds"); dump();
  check(writes.size() == 3 && writes[0].address == 0x0101 && writes[0].value == 25 &&
    writes[1].address == 0x0104 && writes[1].value == 40 &&
    writes[2].address == 0x0103 && writes[2].value == 1000,
    "explicit opening lazily configures force/speed then writes configured open target");
  check(count(0x0100) == 0, "opening opt-in does not imply calibration");
  check(gripper.has_position_command() && !gripper.closed(), "opening records submitted command only");
}
void manual_commands_and_invalid_trigger() {
  reset(); JAKAZuRobot robot;
  pgi140::Pgi140Gripper gripper(robot, config());
  check(gripper.initialize() && writes.empty(), "manual path begins with preserved state");
  const float invalid[] = {std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -0.01f, 1.01f};
  for (float value : invalid) check(!gripper.Grip(value), "malformed trigger fails before any TIO command");
  check(writes.empty() && !gripper.has_position_command(), "invalid triggers do not alter physical or command state");
  check(gripper.Grip(.25f), "first explicit analog command succeeds"); dump();
  check(writes.size() == 3 && writes[0].address == 0x0101 && writes[0].value == 25 &&
    writes[1].address == 0x0104 && writes[1].value == 40 &&
    writes[2].address == 0x0103 && writes[2].value == 750,
    "first manual command configures motion settings before correct interpolated position");
  check(gripper.Grip(.75f) && writes.size() == 4 && writes[3].address == 0x0103 && writes[3].value == 250,
    "second explicit analog command sends only fresh requested position");
  check(gripper.set_closed(true) && writes.back().address == 0x0103 && writes.back().value == 0 && gripper.closed(),
    "digital close remains available and uses configured closed target");
  check(gripper.set_closed(false) && writes.back().value == 1000 && !gripper.closed(),
    "digital open remains available on explicit request");
}
void lazy_configuration_failure() {
  reset(); JAKAZuRobot robot;
  pgi140::Pgi140Gripper gripper(robot, config());
  check(gripper.initialize(), "preserve startup succeeds before failure test");
  next_return = -3;
  check(!gripper.Grip(.5f), "first force write failure rejects manual motion");
  check(writes.size() == 1 && writes[0].address == 0x0101 && count(0x0103) == 0 &&
    gripper.last_sdk_return() == -3 && !gripper.has_position_command(),
    "raw SDK failure survives and no position write follows failed lazy setup");
  check(gripper.Grip(.5f) && writes.size() == 4 && writes.back().address == 0x0103 && writes.back().value == 500,
    "retry explicitly reconfigures failed setting then submits intended position");
}
} // namespace fake

JAKAZuRobot::JAKAZuRobot() {}
JAKAZuRobot::~JAKAZuRobot() {}
errno_t JAKAZuRobot::set_tio_vout_param(int, int) { ++fake::power_calls; return 0; }
errno_t JAKAZuRobot::set_tio_pin_mode(int, int) { ++fake::pin_calls; return 0; }
errno_t JAKAZuRobot::set_rs485_chn_mode(int, int) { ++fake::serial_calls; return 0; }
errno_t JAKAZuRobot::set_rs485_chn_comm(ModRtuComm) { ++fake::serial_calls; return 0; }
errno_t JAKAZuRobot::send_tio_rs_command(int channel, uint8_t *bytes, int size) {
  fake::check(channel == 1 && size == 8, "TIO channel and Modbus length");
  fake::check(bytes[0] == 1 && bytes[1] == 6, "Modbus slave and write-single-register opcode");
  const auto crc = pgi140::crc16_modbus(bytes, 6);
  fake::check(bytes[6] == (crc & 255) && bytes[7] == (crc >> 8), "Modbus CRC validates");
  const uint16_t address = static_cast<uint16_t>((bytes[2] << 8) | bytes[3]);
  const uint16_t value = static_cast<uint16_t>((bytes[4] << 8) | bytes[5]);
  fake::writes.push_back({address, value});
  const int ret = fake::next_return; fake::next_return = 0;
  if (ret == 0 && address == 0x0103) fake::physical_position = value;
  return ret;
}

int main(int argc, char **argv) {
  const std::string which = argc > 1 ? argv[1] : "all";
  if (which == "all" || which == "preserve") fake::preserve();
  if (which == "all" || which == "default") fake::default_preserves();
  if (which == "all" || which == "calibration") fake::calibration_only();
  if (which == "all" || which == "parameters") fake::motion_parameters_only();
  if (which == "all" || which == "open") fake::explicit_open();
  if (which == "all" || which == "manual") fake::manual_commands_and_invalid_trigger();
  if (which == "all" || which == "failure") fake::lazy_configuration_failure();
  std::cout << "PASS gripper startup policy " << which << '\n';
  return 0;
}
