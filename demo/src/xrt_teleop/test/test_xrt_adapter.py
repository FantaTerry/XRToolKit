import math
import unittest

from xrt_teleop.xrt_adapter import (
    DEFAULT_HEADSET_TO_ROBOT,
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


if __name__ == "__main__":
    unittest.main()
