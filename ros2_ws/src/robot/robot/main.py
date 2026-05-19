from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math
import numpy as np


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

TAG_ID = 14 # set aruco tag ID 11 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 79.2
WHEEL_BASE = 358.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = True
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = False


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    robot.set_tracked_tag_id(TAG_ID) # set aruco tag ID as the tracked tag for localization


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def run(robot: Robot) -> None:
    # 1. The delay to fix the firmware overwrite race condition
    time.sleep(0.5) 
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    next_tick = time.monotonic()
    
    # Declare planner and path at this scope so they persist
    planner = None
    remaining_path = []

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            
            # Center lane waypoints
            path_control_points = [ 
                (0.0, 0.0),           # 1st/starting point
                (-50.0, 3431.0),        # 2nd point
                (533.4, 3431.0),      # 3rd point 
                (450.4, 624.8),       # 4th point
                (1524, 624.8),        # 5th point
                (1524, 2630),         # 6th point                        
            ]

            # Densify the path
            remaining_path = densify_polyline(path_control_points, spacing=20.0)

            # 2. Initialize the STANDARD Pure Pursuit planner (No Avoidance!)
            LOOKAHEAD_DIST = 100.0
            planner = PurePursuitPlanner(
                lookahead_dist=LOOKAHEAD_DIST,
                max_angular=1.5, 
                goal_tolerance=20.0,
            )
            
            print("Standard Pure Pursuit initialized. Lidar is completely ignored.")
            print("[FSM] IDLE - Press BTN_1 to enter MOVING state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            if robot.get_button(Button.BTN_1):
                print("Start Moving!")
                state = "MOVING"
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot and saving trajectory.")
                robot.shutdown()

        elif state == "MOVING":
            show_moving_leds(robot)
            
            # Step A: Get current pose
            current_x, current_y, current_theta_deg = robot.get_pose()
            current_theta_rad = math.radians(current_theta_deg)

            # Step B: Advance the path
            remaining_path = robot._advance_remaining_path(
                remaining_path, 
                current_x, 
                current_y, 
                advance_radius_mm=LOOKAHEAD_DIST
            )

            # Step C: Compute pure pursuit velocity (NO OBSTACLES)
            linear_cmd, angular_cmd_rad_s = planner.compute_velocity(
                pose=(current_x, current_y, current_theta_rad),
                waypoints=remaining_path,
                max_linear=160.0,
            )

            # Step D: Send the commands
            # print(f"COMMANDING: Lin={linear_cmd:.1f} mm/s, Ang={math.degrees(angular_cmd_rad_s):.1f} deg/s")
            robot.set_velocity(linear_cmd, math.degrees(angular_cmd_rad_s))

            # Step E: Check if finished
            current_pursuit_x, current_pursuit_y = planner._lookahead_point(
                current_x, current_y, waypoints=remaining_path
            )
            
            if planner.CurrentTargetReached(current_pursuit_x, current_pursuit_y, current_x, current_y):
                print("MOVING: Target reached! Stopping.")
                robot.stop()
                state = "IDLE"

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
