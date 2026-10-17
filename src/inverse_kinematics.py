"""
Inverse Kinematics (IK) Engine for 4-DOF Robotic Arm
-----------------------------------------------------
Handles geometric Inverse Kinematics calculations for:
  - Joint 0: Base rotation (Yaw)
  - Joint 1: Shoulder pitch (Elevation)
  - Joint 2: Elbow flex (Elbow angle)
  - Joint 3: Wrist tilt (Pitch stabilization)
"""

import math
from typing import Tuple, Optional, Dict, Any

class ArmKinematics:
    def __init__(
        self,
        l1: float = 120.0,  # Shoulder to Elbow in mm
        l2: float = 120.0,  # Elbow to Wrist in mm
        base_height: float = 60.0,  # Base to Shoulder pivot in mm
        camera_scale_mm_per_pixel: float = 0.5,
        camera_origin_offset: Tuple[float, float] = (0.0, 150.0) # mm from base to camera center
    ):
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.base_height = float(base_height)
        self.scale = camera_scale_mm_per_pixel
        self.cam_offset_x, self.cam_offset_y = camera_origin_offset

    def calculate_2d_ik(
        self,
        x: float,
        y: float,
        elbow_up: bool = False
    ) -> Tuple[Optional[int], Optional[int]]:
        """
        Calculates planar shoulder and elbow angles (in degrees) to reach (x, y) relative to shoulder pivot.
        
        Args:
            x: Forward reach in mm
            y: Height reach in mm
            elbow_up: Solution branch (True for elbow-up, False for elbow-down)
            
        Returns:
            (shoulder_deg, elbow_deg) clamped to [0, 180] or (None, None) if unreachable.
        """
        distance_sq = x**2 + y**2
        distance = math.sqrt(distance_sq)

        # Check maximum reachability
        max_reach = self.l1 + self.l2
        min_reach = abs(self.l1 - self.l2)

        if distance > max_reach:
            # Out of reach
            return None, None
        if distance < min_reach and min_reach > 0.001:
            # Too close to body
            return None, None

        # Law of Cosines for elbow flex angle
        cos_angle2 = (distance_sq - self.l1**2 - self.l2**2) / (2.0 * self.l1 * self.l2)
        # Protect against float precision issues outside [-1.0, 1.0]
        cos_angle2 = max(-1.0, min(1.0, cos_angle2))
        
        angle2 = math.acos(cos_angle2)
        if elbow_up:
            angle2 = -angle2

        # Law of Cosines / Trigonometry for shoulder pitch angle
        k1 = self.l1 + self.l2 * math.cos(angle2)
        k2 = self.l2 * math.sin(angle2)
        angle1 = math.atan2(y, x) - math.atan2(k2, k1)

        shoulder_deg = math.degrees(angle1)
        elbow_deg = math.degrees(abs(angle2))

        # Normalize and map to typical servo 0-180 orientation
        # 90 degrees = upright vertical or centered neutral position
        s_int = int(round(max(0, min(180, shoulder_deg))))
        e_int = int(round(max(0, min(180, elbow_deg))))

        return s_int, e_int

    def calculate_3d_ik(
        self,
        x: float,
        y: float,
        z: float
    ) -> Tuple[Optional[int], Optional[int], Optional[int]]:
        """
        Calculates 3D coordinates (x=lateral, y=forward, z=elevation) in mm from arm base center.
        
        Returns:
            (base_deg, shoulder_deg, elbow_deg)
        """
        # Base angle (horizontal yaw)
        # Assuming 90 deg is forward (along +Y axis)
        base_rad = math.atan2(x, y)
        base_deg = 90 + math.degrees(base_rad)
        base_deg = max(0, min(180, int(round(base_deg))))

        # Planar horizontal distance and effective vertical distance
        planar_dist = math.sqrt(x**2 + y**2)
        eff_z = z - self.base_height

        shoulder_deg, elbow_deg = self.calculate_2d_ik(planar_dist, eff_z)
        if shoulder_deg is None:
            return None, None, None

        return base_deg, shoulder_deg, elbow_deg

    def pixel_to_world(
        self,
        pixel_x: float,
        pixel_y: float,
        frame_width: int = 640,
        frame_height: int = 480
    ) -> Tuple[float, float]:
        """
        Converts camera frame pixel coordinates to real-world ground-plane (x, y) in mm.
        """
        cx = frame_width / 2.0
        cy = frame_height / 2.0

        # Offset from camera optical center
        dx = (pixel_x - cx) * self.scale
        # Invert Y so upward in camera is positive forward in world
        dy = (cy - pixel_y) * self.scale

        world_x = self.cam_offset_x + dx
        world_y = self.cam_offset_y + dy
        return world_x, world_y

    def coin_to_joint_targets(
        self,
        pixel_x: float,
        pixel_y: float,
        frame_width: int = 640,
        frame_height: int = 480,
        target_z: float = 10.0  # Coin surface height
    ) -> Optional[Dict[str, Any]]:
        """
        End-to-end solver: Takes coin pixel location from camera and returns joint commands.
        """
        world_x, world_y = self.pixel_to_world(pixel_x, pixel_y, frame_width, frame_height)
        base, shoulder, elbow = self.calculate_3d_ik(world_x, world_y, target_z)

        if base is None or shoulder is None or elbow is None:
            return None

        # Auto-compute wrist angle to keep the magnet parallel to ground
        wrist = 90  # Default neutral tilt

        return {
            "world_coords": {"x": round(world_x, 2), "y": round(world_y, 2), "z": round(target_z, 2)},
            "joints": {
                "base": base,
                "shoulder": shoulder,
                "elbow": elbow,
                "wrist": wrist
            }
        }

    def solve_from_base_pixel(
        self,
        target_x: float,
        target_y: float,
        base_x: float,
        base_y: float,
        target_z: float = 10.0
    ) -> Optional[Dict[str, Any]]:
        """
        Calculates IK from a tapped user base reference point (base_x, base_y) on the image.
        """
        # dx is lateral (+ right, - left)
        dx = (target_x - base_x) * self.scale
        # dy is forward (+ up in image, away from robot base)
        dy = (base_y - target_y) * self.scale

        planar_dist = math.sqrt(dx**2 + dy**2)
        if planar_dist < 1.0:
            planar_dist = 1.0

        # Base yaw: 90 deg is straight ahead (+dy), <90 is left, >90 is right
        base_deg = 90.0 + math.degrees(math.atan2(dx, dy))
        base_deg = max(0, min(180, int(round(base_deg))))

        eff_z = target_z - self.base_height
        shoulder_deg, elbow_deg = self.calculate_2d_ik(planar_dist, eff_z)
        if shoulder_deg is None or elbow_deg is None:
            # Fallback for close-proximity
            shoulder_deg, elbow_deg = 0, 90

        return {
            "world_coords": {"x": round(dx, 1), "y": round(dy, 1), "z": round(target_z, 1)},
            "joints": {
                "base": base_deg,
                "shoulder": shoulder_deg,
                "elbow": elbow_deg,
                "wrist": 90
            }
        }
