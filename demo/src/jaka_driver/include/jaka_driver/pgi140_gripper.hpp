#ifndef JAKA_DRIVER_PGI140_GRIPPER_HPP_
#define JAKA_DRIVER_PGI140_GRIPPER_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include "jaka_driver/JAKAZuRobot.h"

namespace pgi140
{

struct GripperConfig
{
  int rs485_channel = 1;       // 0: TIO RS485H, 1: TIO RS485L (TIO channel 2)
  int slave_id = 1;
  int baudrate = 115200;
  int databit = 8;
  int stopbit = 1;
  int parity = 78;             // JAKA SDK: 78 means no parity
  int force_percent = 50;      // PGI range: 20..100
  int speed_percent = 100;      // PGI range: 1..100
  int open_position = 0;       // PGI range: 0..1000 per mille
  int closed_position = 1000;  // PGI range: 0..1000 per mille
  // Safe restart policy: configure communication without issuing calibration,
  // force/speed or position writes unless the operator explicitly opts in.
  bool initialize = false;
  bool configure_motion_parameters = false;
  bool startup_open = false;
  int initialize_command = 0x01;
  int initialize_delay_ms = 2000;
  bool configure_tio = true;   // false: use the TIO configuration saved in JAKA App
  bool enable_tio_power = true;
  int tio_voltage = 0;         // JAKA TIO_VOUT_24V = 0, TIO_VOUT_12V = 1
};

class Pgi140Gripper
{
public:
  Pgi140Gripper(JAKAZuRobot & robot, GripperConfig config);

  // Configure communication. Calibration, motion-parameter writes and active
  // opening are independent explicit startup choices in GripperConfig.
  bool initialize(std::string * error = nullptr);

  // Send a target position. The value is validated before any frame is sent.
  bool set_position(int position, std::string * error = nullptr);
  bool set_force(int force_percent, std::string * error = nullptr);
  bool set_speed(int speed_percent, std::string * error = nullptr);
  bool set_closed(bool closed, std::string * error = nullptr);
  bool Grip(float grip, std::string * error = nullptr);

  // These describe successful command submission, never physical readback.
  // With the default preserve policy no position is assumed after restart.
  bool has_position_command() const { return has_position_command_; }
  bool closed() const { return closed_; }
  int last_sdk_return() const { return last_sdk_return_; }
  // Owner-supplied authorization checked before EACH synchronous register write.
  // It cannot cancel a call already inside the SDK.
  void set_command_guard(std::function<bool()> guard) { command_guard_ = std::move(guard); }
  const GripperConfig & config() const { return config_; }

private:
  bool write_register(uint16_t address, uint16_t value, std::string * error);
  bool fail(const std::string & message, std::string * error) const;

  JAKAZuRobot & robot_;
  GripperConfig config_;
  bool configured_ = false;
  bool force_configured_ = false;
  bool speed_configured_ = false;
  bool has_position_command_ = false;
  bool closed_ = false;
  int last_sdk_return_ = 0; // raw result of the most recent synchronous TIO write
  std::function<bool()> command_guard_;
};

}  // namespace pgi140

#endif  // JAKA_DRIVER_PGI140_GRIPPER_HPP_
