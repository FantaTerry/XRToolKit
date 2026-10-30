#ifndef JAKA_DRIVER_ORIENTATION_MATH_HPP_
#define JAKA_DRIVER_ORIENTATION_MATH_HPP_

#include <algorithm>
#include <cmath>

#include "jaka_driver/jktypes.h"

namespace jaka_teleop
{

constexpr double kQuaternionEpsilon = 1e-12;
constexpr double kAngleEpsilon = 1e-9;

inline Quaternion quat_identity()
{
  Quaternion result;
  result.s = 1.0;
  result.x = 0.0;
  result.y = 0.0;
  result.z = 0.0;
  return result;
}

inline bool quat_normalize(const Quaternion & input, Quaternion * output)
{
  const double norm = std::sqrt(
    input.s * input.s + input.x * input.x +
    input.y * input.y + input.z * input.z);
  if (!std::isfinite(norm) || norm < kQuaternionEpsilon) {
    return false;
  }
  output->s = input.s / norm;
  output->x = input.x / norm;
  output->y = input.y / norm;
  output->z = input.z / norm;
  return true;
}

inline Quaternion quat_normalized(const Quaternion & input)
{
  Quaternion output;
  if (!quat_normalize(input, &output)) {
    return quat_identity();
  }
  return output;
}

inline Quaternion quat_conjugate(const Quaternion & quaternion)
{
  Quaternion result;
  result.s = quaternion.s;
  result.x = -quaternion.x;
  result.y = -quaternion.y;
  result.z = -quaternion.z;
  return result;
}

// Hamilton product a * b. Rotations are applied in b-then-a order.
inline Quaternion quat_multiply(const Quaternion & a, const Quaternion & b)
{
  Quaternion result;
  result.s = a.s * b.s - a.x * b.x - a.y * b.y - a.z * b.z;
  result.x = a.s * b.x + a.x * b.s + a.y * b.z - a.z * b.y;
  result.y = a.s * b.y - a.x * b.z + a.y * b.s + a.z * b.x;
  result.z = a.s * b.z + a.x * b.y - a.y * b.x + a.z * b.s;
  return result;
}

inline Quaternion quat_canonical(const Quaternion & input)
{
  Quaternion result = input;
  if (result.s < 0.0) {
    result.s = -result.s;
    result.x = -result.x;
    result.y = -result.y;
    result.z = -result.z;
  }
  return result;
}

inline double quat_rotation_angle(const Quaternion & input)
{
  const Quaternion q = quat_canonical(input);
  const double scalar = std::max(-1.0, std::min(1.0, q.s));
  return 2.0 * std::acos(scalar);
}

// Return the same rotation reduced to a positive rotation of at most step_angle.
inline Quaternion quat_scaled_to_angle(const Quaternion & input, double step_angle)
{
  const Quaternion q = quat_canonical(input);
  const double angle = quat_rotation_angle(q);
  if (angle < kAngleEpsilon) {
    return quat_identity();
  }

  const double sine_half = std::sin(angle * 0.5);
  if (sine_half < kAngleEpsilon) {
    return quat_identity();
  }
  const double axis_x = q.x / sine_half;
  const double axis_y = q.y / sine_half;
  const double axis_z = q.z / sine_half;
  const double half_step = step_angle * 0.5;
  const double sine_step = std::sin(half_step);

  Quaternion result;
  result.s = std::cos(half_step);
  result.x = axis_x * sine_step;
  result.y = axis_y * sine_step;
  result.z = axis_z * sine_step;
  return result;
}

inline bool grip_state(bool was_pressed, double value, double press, double release)
{
  return was_pressed ? value > release : value > press;
}

}  // namespace jaka_teleop

#endif  // JAKA_DRIVER_ORIENTATION_MATH_HPP_
