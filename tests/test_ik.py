import unittest
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.inverse_kinematics import ArmKinematics

class TestInverseKinematics(unittest.TestCase):
    def setUp(self):
        # Arm with two 120mm linkages
        self.arm = ArmKinematics(l1=120.0, l2=120.0, base_height=50.0)

    def test_neutral_reach(self):
        # Point reachable in 2D plane
        s, e = self.arm.calculate_2d_ik(x=120.0, y=120.0)
        self.assertIsNotNone(s)
        self.assertIsNotNone(e)
        self.assertTrue(0 <= s <= 180)
        self.assertTrue(0 <= e <= 180)

    def test_out_of_reach(self):
        # Total arm length is 240mm; target at 300mm should return None
        s, e = self.arm.calculate_2d_ik(x=200.0, y=200.0)
        self.assertIsNone(s)
        self.assertIsNone(e)

    def test_3d_kinematics(self):
        # Target in front of base
        base, shoulder, elbow = self.arm.calculate_3d_ik(x=0.0, y=140.0, z=50.0)
        self.assertIsNotNone(base)
        self.assertEqual(base, 90)  # Directly forward is 90 deg
        self.assertIsNotNone(shoulder)
        self.assertIsNotNone(elbow)

    def test_pixel_to_joint_targets(self):
        res = self.arm.coin_to_joint_targets(pixel_x=320, pixel_y=240, frame_width=640, frame_height=480)
        self.assertIsNotNone(res)
        self.assertIn("joints", res)
        self.assertIn("base", res["joints"])
        self.assertIn("shoulder", res["joints"])
        self.assertIn("elbow", res["joints"])
        self.assertIn("wrist", res["joints"])

if __name__ == '__main__':
    unittest.main()
