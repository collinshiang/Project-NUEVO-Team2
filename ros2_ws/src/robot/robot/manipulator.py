"""
manipulator.py — cylinder detection and grab sequence for the 3-DOF arm.

Joint mapping (per hardware):
  Yaw   — stepper 1         (swivel)
  J2    — servo 1           (shoulder/elevator: 180°=retracted, 0°=extended)
  J3    — servo 2           (elbow/extension:     0°=retracted, 180°=extended)
  Grip  — servo 3           (180°=open, ~80°=closed)

Sensor reframing (LiDAR origin → yaw-joint origin, mm):
  x_arm = x_lidar + 0
  y_arm = y_lidar - 115
  z_arm = z_lidar + 93.5   (LiDAR is 93.5 mm below the yaw joint)
"""

from __future__ import annotations

import math
import time

import numpy as np

from robot.robot import Robot

# ---------------------------------------------------------------------------
# Arm geometry — tune to physical robot (mm)
# ---------------------------------------------------------------------------

ARM_L2_MM = 200.0   # shoulder pivot → elbow pivot
ARM_L3_MM = 200.0   # elbow pivot → end-effector tip

# ---------------------------------------------------------------------------
# Servo angle conventions
# ---------------------------------------------------------------------------

GRIPPER_OPEN_DEG  = 180.0
GRIPPER_CLOSE_DEG =  80.0

# ---------------------------------------------------------------------------
# Sensor reframing offsets (slide values, mm)
# ---------------------------------------------------------------------------

LIDAR_TO_ARM_X_MM =   0.0
LIDAR_TO_ARM_Y_MM = -115.0
LIDAR_TO_ARM_Z_MM =  93.5    # positive = yaw joint is above the LiDAR plane

# ---------------------------------------------------------------------------
# Yaw stepper conversion
# ---------------------------------------------------------------------------

YAW_STEPS_PER_DEG = 10.0    # tune to gearing / micro-step setting

# ---------------------------------------------------------------------------
# Cylinder detection parameters
# ---------------------------------------------------------------------------

CYLINDER_CLUSTER_MIN_PTS    = 5
CYLINDER_CLUSTER_MAX_RAD_MM = 80.0

# Lift height after gripping (mm)
LIFT_HEIGHT_MM = 30.0


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _find_cylinder(robot: Robot) -> tuple[float, float, float] | None:
    """
    Return (X, Y, Z) of the nearest cylindrical cluster in the manipulator
    frame, or None if no suitable object is found.

    The LiDAR delivers 2-D (x, y) in the robot body frame (mm). We treat the
    scan plane as the horizontal floor (z_lidar = 0) and apply the reframing
    offsets to get manipulator-origin coordinates.
    """
    obstacles = robot._get_obstacles_mm()
    if len(obstacles) < CYLINDER_CLUSTER_MIN_PTS:
        return None

    pts = np.array(obstacles, dtype=float)   # shape (N, 2)

    # Keep only forward-facing points (robot +x direction).
    pts = pts[pts[:, 0] > 0.0]
    if len(pts) < CYLINDER_CLUSTER_MIN_PTS:
        return None

    # Greedy radius clustering — group points within CYLINDER_CLUSTER_MAX_RAD_MM.
    visited  = np.zeros(len(pts), dtype=bool)
    clusters: list[np.ndarray] = []

    for i in range(len(pts)):
        if visited[i]:
            continue
        dists   = np.linalg.norm(pts - pts[i], axis=1)
        members = np.where(dists <= CYLINDER_CLUSTER_MAX_RAD_MM)[0]
        if len(members) >= CYLINDER_CLUSTER_MIN_PTS:
            visited[members] = True
            clusters.append(pts[members])

    if not clusters:
        return None

    # Pick the closest cluster centroid.
    centroids = [c.mean(axis=0) for c in clusters]
    distances = [math.hypot(cx, cy) for cx, cy in centroids]
    cx_body, cy_body = centroids[int(np.argmin(distances))]

    # Apply sensor reframing to convert to manipulator origin.
    x_arm = cx_body + LIDAR_TO_ARM_X_MM
    y_arm = cy_body + LIDAR_TO_ARM_Y_MM
    z_arm = 0.0     + LIDAR_TO_ARM_Z_MM   # yaw joint sits above the LiDAR plane

    return (x_arm, y_arm, z_arm)


def _ik(X: float, Y: float, Z: float) -> tuple[float, float, float]:
    """
    3-DOF inverse kinematics (yaw + shoulder + elbow).

    Args:
      X, Y — horizontal position in the manipulator frame (mm)
      Z    — vertical offset, positive = object is below the yaw joint (mm)

    Returns:
      theta1_deg — yaw angle for stepper 1
      theta2_deg — shoulder servo angle (servo 1), 180°=retracted / 0°=extended
      theta3_deg — elbow servo angle   (servo 2),   0°=retracted / 180°=extended
    """
    # Yaw: rotate to put the arm in the vertical plane containing the target.
    theta1_deg = math.degrees(math.atan2(X, Y))

    # Planar reach and vertical drop from shoulder pivot.
    r      = math.hypot(X, Y)
    z_drop = Z

    # Clamp reach to arm workspace.
    d = math.hypot(r, z_drop)
    d = max(1.0, min(d, ARM_L2_MM + ARM_L3_MM - 1.0))

    # Elbow angle via Law of Cosines.
    cos_t3 = (d**2 - ARM_L2_MM**2 - ARM_L3_MM**2) / (2.0 * ARM_L2_MM * ARM_L3_MM)
    cos_t3 = max(-1.0, min(1.0, cos_t3))
    theta3_rad = math.pi - math.acos(cos_t3)   # elbow-up solution

    # Shoulder angle from complementary angles (atan2 decomposition).
    alpha = math.atan2(z_drop, r)
    beta  = math.atan2(ARM_L3_MM * math.sin(theta3_rad),
                       ARM_L2_MM + ARM_L3_MM * math.cos(theta3_rad))
    theta2_rad = alpha + beta

    # θ3: 0 = retracted (0 rad), 180 = extended (π rad)
    theta3_deg = math.degrees(theta3_rad)

    # θ2: servo 1 is inverted — 180° = arm up (retracted), 0° = arm down (extended)
    theta2_deg = 180.0 - math.degrees(theta2_rad)

    theta2_deg = max(0.0, min(180.0, theta2_deg))
    theta3_deg = max(0.0, min(180.0, theta3_deg))

    return theta1_deg, theta2_deg, theta3_deg


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def grab_cylinder(robot: Robot) -> bool:
    """
    Detect the nearest cylinder in front of the robot, position the arm over
    it using inverse kinematics, grip it, and lift it slightly off the ground.

    Returns True on success, False if no suitable object is detected.
    """
    print("[manipulator] scanning for cylinder...")

    target = _find_cylinder(robot)
    if target is None:
        print("[manipulator] no cylindrical object detected")
        return False

    X, Y, Z = target
    print(f"[manipulator] target at arm-frame (X={X:.1f} Y={Y:.1f} Z={Z:.1f}) mm")

    theta1_deg, theta2_deg, theta3_deg = _ik(X, Y, Z)
    print(f"[manipulator] IK  θ1={theta1_deg:.1f}°  θ2={theta2_deg:.1f}°  θ3={theta3_deg:.1f}°")

    # Open gripper before approaching.
    robot.enable_servo(3)
    robot.set_servo(3, GRIPPER_OPEN_DEG)
    time.sleep(0.4)

    # Yaw arm to face object (stepper 1, relative move).
    yaw_steps = int(round(theta1_deg * YAW_STEPS_PER_DEG))
    robot.step_enable(1)
    robot.step_move(1, yaw_steps, blocking=True, timeout=5.0)

    # Set shoulder angle (servo 1).
    robot.enable_servo(1)
    robot.set_servo(1, theta2_deg)
    time.sleep(0.6)

    # Extend elbow to reach target (servo 2).
    robot.enable_servo(2)
    robot.set_servo(2, theta3_deg)
    time.sleep(0.6)

    # Close gripper.
    print("[manipulator] closing gripper")
    robot.set_servo(3, GRIPPER_CLOSE_DEG)
    time.sleep(0.5)

    # Lift by raising the shoulder slightly.
    lift_delta_deg = math.degrees(math.atan2(LIFT_HEIGHT_MM, math.hypot(X, Y)))
    theta2_lifted  = max(0.0, min(180.0, theta2_deg - lift_delta_deg))
    print(f"[manipulator] lifting  shoulder {theta2_deg:.1f}° → {theta2_lifted:.1f}°")
    robot.set_servo(1, theta2_lifted)
    time.sleep(0.6)

    print("[manipulator] cylinder gripped and lifted")
    return True
