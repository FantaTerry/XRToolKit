#include <cassert>
#include <cmath>

#include "jaka_driver/orientation_math.hpp"

namespace
{

bool almost_equal(double first, double second, double tolerance = 1e-9)
{
  return std::fabs(first - second) <= tolerance;
}

}  // namespace

int main()
{
  constexpr double pi = 3.14159265358979323846;
  using jaka_teleop::quat_canonical;
  using jaka_teleop::quat_identity;
  using jaka_teleop::quat_multiply;
  using jaka_teleop::quat_normalize;
  using jaka_teleop::quat_rotation_angle;
  using jaka_teleop::quat_scaled_to_angle;
  using jaka_teleop::grip_state;

  Quaternion half_turn;
  half_turn.s = 0.0;
  half_turn.x = 0.0;
  half_turn.y = 0.0;
  half_turn.z = 1.0;
  assert(almost_equal(quat_rotation_angle(half_turn), pi));

  const Quaternion quarter_turn = quat_scaled_to_angle(half_turn, pi * 0.5);
  assert(almost_equal(quat_rotation_angle(quarter_turn), pi * 0.5));

  const Quaternion limited_step = quat_scaled_to_angle(half_turn, 0.005);
  assert(almost_equal(quat_rotation_angle(limited_step), 0.005));

  const Quaternion reversed = quat_canonical(
    {half_turn.s, -half_turn.x, -half_turn.y, -half_turn.z});
  assert(almost_equal(quat_rotation_angle(reversed), pi));
  assert(almost_equal(reversed.z, half_turn.z));
  const Quaternion negative_half_turn{-0.0, 0.0, 0.0, -1.0};
  const Quaternion equivalent_step = quat_scaled_to_angle(negative_half_turn, 0.005);
  assert(almost_equal(equivalent_step.s, limited_step.s));
  assert(almost_equal(equivalent_step.z, limited_step.z));
  for (const Quaternion q : {Quaternion{0.0, -1.0, 0.0, 0.0},
      Quaternion{-0.0, 0.0, -1.0, 0.0}, Quaternion{0.0, 0.0, 0.0, -1.0}}) {
    const Quaternion canonical = quat_canonical(q);
    assert(canonical.x >= 0.0 && canonical.y >= 0.0 && canonical.z >= 0.0);
  }

  Quaternion normalized;
  const Quaternion unnormalized{2.0, 0.0, 0.0, 0.0};
  assert(quat_normalize(unnormalized, &normalized));
  assert(almost_equal(normalized.s, 1.0));

  const Quaternion identity = quat_identity();
  const Quaternion product = quat_multiply(identity, quarter_turn);
  assert(almost_equal(product.s, quarter_turn.s));
  assert(almost_equal(product.z, quarter_turn.z));

  assert(!grip_state(false, 0.5, 0.6, 0.4));
  assert(grip_state(false, 0.61, 0.6, 0.4));
  assert(grip_state(true, 0.5, 0.6, 0.4));
  assert(!grip_state(true, 0.39, 0.6, 0.4));
  return 0;
}
