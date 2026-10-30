#ifndef JAKA_DRIVER_SERVO_GUARD_HPP_
#define JAKA_DRIVER_SERVO_GUARD_HPP_

#include <algorithm>
#include <cmath>

// Pure arithmetic helpers for the three conservative servo governors.
// No ROS, no SDK, no clock: every caller passes the numbers it measured so the
// behaviour can be reasoned about (and unit-tested) without a robot.
namespace jaka_guard
{

// Translation increment (mm per 8 ms cycle) that was actually submitted last
// cycle. Only a successful servo_p call may advance it.
struct AccelState
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  void reset() {x = 0.0; y = 0.0; z = 0.0;}
};

// Clamp a displacement vector to a sphere of `radius_mm`. Returns true when the
// value was clamped. radius_mm <= 0 disables the check.
inline bool clamp_sphere(double radius_mm, double & x, double & y, double & z)
{
  if (!(radius_mm > 0.0)) {return false;}
  const double norm = std::sqrt(x * x + y * y + z * z);
  if (!std::isfinite(norm) || !(norm > radius_mm)) {return false;}
  const double scale = radius_mm / norm;
  x *= scale;
  y *= scale;
  z *= scale;
  return true;
}

// Limit the change of the per-cycle increment so the commanded Cartesian
// acceleration stays below `accel_mm_s2`. `last` is not modified: the caller
// commits it only after servo_p accepted the value.
// accel_mm_s2 <= 0 or cycle_s <= 0 disables the limit.
inline void limit_accel(double accel_mm_s2, double cycle_s, const AccelState & last,
                        double & x, double & y, double & z)
{
  if (!(accel_mm_s2 > 0.0) || !(cycle_s > 0.0)) {return;}
  const double span = accel_mm_s2 * cycle_s * cycle_s;  // mm per cycle per cycle
  x = std::max(last.x - span, std::min(last.x + span, x));
  y = std::max(last.y - span, std::min(last.y + span, y));
  z = std::max(last.z - span, std::min(last.z + span, z));
}

inline double distance_mm(double ax, double ay, double az,
                          double bx, double by, double bz)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  const double dz = az - bz;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace jaka_guard

#endif  // JAKA_DRIVER_SERVO_GUARD_HPP_
