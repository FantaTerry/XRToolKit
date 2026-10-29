import math
import unittest

from xrt_teleop.xrt_adapter import (
    DEFAULT_HEADSET_TO_ROBOT,
    SourceFreshness,
    BUTTON_ORDER,
    joy_payload,
    matrix_3x3,
    quaternion_from_matrix,
    raw_pose_is_valid,
    smooth_pose,
    transform_pose,
)


class XrtAdapterTest(unittest.TestCase):
    def test_default_mapping_matches_legacy_axis_conversion(self):
        position, quaternion = transform_pose(
            (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
        )
        self.assertEqual(position, (-3.0, -1.0, 2.0))
        self.assertEqual(quaternion, (1.0, 0.0, 0.0, 0.0))

    def test_identity_mapping_preserves_pose(self):
        identity = matrix_3x3((1, 0, 0, 0, 1, 0, 0, 0, 1))
        position, quaternion = transform_pose(
            (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0),
            mapping=identity,
        )
        self.assertEqual(position, (0.1, 0.2, 0.3))
        self.assertAlmostEqual(quaternion[0], 1.0)

    def test_default_mapping_quaternion_is_proper_rotation(self):
        quaternion = quaternion_from_matrix(matrix_3x3(DEFAULT_HEADSET_TO_ROBOT))
        self.assertAlmostEqual(math.sqrt(sum(value * value for value in quaternion)), 1.0)

    def test_default_mapping_rotates_controller_axes(self):
        half_sqrt_two = math.sqrt(0.5)
        _, quaternion = transform_pose(
            (0.0, 0.0, 0.0, 0.0, half_sqrt_two, 0.0, half_sqrt_two)
        )
        self.assertAlmostEqual(quaternion[0], half_sqrt_two)
        self.assertAlmostEqual(quaternion[1], 0.0)
        self.assertAlmostEqual(quaternion[2], 0.0)
        self.assertAlmostEqual(quaternion[3], half_sqrt_two)

    def test_zero_pose_is_untracked(self):
        self.assertFalse(raw_pose_is_valid((0.0,) * 7))
        self.assertTrue(
            raw_pose_is_valid((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
        )

    def test_joy_layout_preserves_legacy_axes_and_buttons(self):
        axes, buttons = joy_payload(
            {
                "grip": 0.8,
                "trigger": 0.25,
                "axis": (0.1, -0.2),
                "X": True,
                "menu": True,
                "axis_click": True,
            },
            {
                "grip": 0.2,
                "trigger": 0.9,
                "axis": (-0.3, 0.4),
                "A": True,
            },
        )
        self.assertEqual(axes, [0.8, 0.2, 0.25, 0.9, 0.1, -0.2, -0.3, 0.4])
        self.assertEqual(len(buttons), len(BUTTON_ORDER))
        self.assertEqual(buttons[BUTTON_ORDER.index("X")], 1)
        self.assertEqual(buttons[BUTTON_ORDER.index("LG")], 1)
        self.assertEqual(buttons[BUTTON_ORDER.index("RG")], 0)
        self.assertEqual(buttons[BUTTON_ORDER.index("RTr")], 1)

    def test_smoothing_spans_between_samples(self):
        position, _ = smooth_pose(
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
            (1.0, 2.0, 3.0),
            (1.0, 0.0, 0.0, 0.0),
            0.5,
        )
        self.assertEqual(position, (0.5, 1.0, 1.5))

    def test_smoothing_preserves_equivalent_quaternion_signs(self):
        previous = ((0.1, 0.2, 0.3), (0.5, 0.5, 0.5, 0.5))
        position, quat = smooth_pose(previous, previous[0], (-0.5,) * 4, 0.4)
        self.assertEqual(position, previous[0])
        self.assertEqual(quat, previous[1])

    def test_source_progress_is_independent_of_stationary_pose(self):
        source = SourceFreshness(timeout_s=0.2)
        for frame in range(100):
            self.assertEqual(source.observe(1000 + frame, frame * 0.01), (True, True))

    def test_duplicate_source_does_not_refresh_age(self):
        source = SourceFreshness(timeout_s=0.2)
        self.assertEqual(source.observe(1000, 1.0), (True, True))
        self.assertEqual(source.observe(1000, 1.1), (True, False))
        self.assertEqual(source.observe(1000, 1.201), (False, False))
        self.assertEqual(source.last_advance_s, 1.0)
        self.assertEqual(source.observe(1001, 1.21), (True, True))

    def test_source_rollback_invalidates_frame(self):
        source = SourceFreshness()
        self.assertEqual(source.observe(1000, 1.0), (True, True))
        self.assertEqual(source.observe(500, 1.1), (False, False))
        self.assertEqual(source.observe(500, 1.12), (False, False))
        self.assertEqual(source.observe(501, 1.13), (True, True))

    def test_source_missing_is_fail_closed_unless_explicit_fallback(self):
        self.assertEqual(SourceFreshness().observe(None, 1.0, False), (False, False))
        fallback = SourceFreshness(require_timestamp=False)
        self.assertEqual(fallback.observe(None, 1.0, False), (True, True))
        # A present getter that fails is not silently replaced by receipt time.
        self.assertEqual(fallback.observe(None, 1.0, True), (False, False))

    def test_invalid_source_values(self):
        for invalid in (None, 0, -1, True, 1.25, float("nan"), float("inf"), "oops"):
            self.assertEqual(SourceFreshness().observe(invalid, 1.0), (False, False))


if __name__ == "__main__":
    unittest.main()
