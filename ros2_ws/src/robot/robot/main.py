from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline
from robot.manipulator import grab_cylinder
from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    POSITION_UNIT,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
    LEFT_WHEEL_MOTOR,
    LEFT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    RIGHT_WHEEL_DIR_INVERTED,
)

# ---------------------------------------------------------------------------
# Sensor Toggles
# ---------------------------------------------------------------------------

ENABLE_LIDAR  = True
ENABLE_GPS    = False
ENABLE_VISION = True

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------

VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50     # higher = more skeptical
MIN_STOP_SIGN_CONFIDENCE = 0.50  

# ---------------------------------------------------------------------------
# Navigation & Mission Configuration
# ---------------------------------------------------------------------------

TAG_ID = 13                 # for GPS
GPS_POSITION_ALPHA = 0.01   # GPS contribution weight

# DWA Goal / Path
PATH_CONTROL_POINTS = [
    # (0.0, 0.0),
    # (0.0, 3000.0),

    # testing whole course:
    (0.0, 0.0),
    (0.0, 3500.0),
    (480.0, 3500.0),
    (480.0, 624.8),
    (1374.0, 624.8),
    (1374.0, 3000.0),
    (1600.0, 3000.0),
]
DENSE_PATH = densify_polyline(PATH_CONTROL_POINTS, spacing=20.0)

# ---------------------------------------------------------------------------
# DWA Tuning Parameters - most of them are extremely sensitive so be sure to keep them the same.
# ---------------------------------------------------------------------------

# Rover will stop if it thinks it's going to collide with something.

# These limits ensure the planner doesn't demand impossible speeds/turns
MAX_VEL_MM_S = 160.0
MAX_ACC_MM_S2 = 500.0  
MAX_ANGULAR_RAD_S = 1.5
MAX_ANGULAR_ACC_RAD_S2 = 5.0

# Trajectory generation parameters
PREDICT_TIME_S = 4.0        
VELOCITY_SAMPLES = [7, 15]  # [linear_samples, angular_samples] - num. of calculated paths

# Cost Weights: [heading_gain, clearance_gain, velocity_gain]
# The rover will follow pure pursuit, so the heading will be the lowest. The clearance is 
# important but the velocity needs to be higher for the rover to actually determine that it 
# should go forward instead of avoid the cone by not moving.
COST_GAINS = [1.5, 1.5, 2.0]

LOOKAHEAD_MM = 175.0        # for Pure Pursuit, also affects 
TOLERANCE_MM = 600.0        # Goal point tolerance (large such that rover doesn't think wall at end is an obstacle and freaks out)

# Distance at which an obstacle starts generating a cost. Increase if it doesn't seem to notice 
# a cone, but it is still sensitive.
OBSTACLES_RANGE_MM = 600.0  

# Bounding box for LiDAR to know what the rover is (give slight buffer for each measurement)
ROBOT_FRONT_MM = 400.0       # Distance from rear axle to the front nose, actual 350
ROBOT_REAR_MM = 50.0         # Distance from rear axle to the back tail, actual 40
ROBOT_HALF_WIDTH_MM = 210.0  # Half of the total robot width, actual 162.5


STATUS_PRINT_INTERVAL_S = 0.5


def resolve_dwa_config() -> dict:
    """Consolidates DWA tuning parameters for easy initialization."""
    return {
        "max_vel_mm": float(MAX_VEL_MM_S),
        "max_acc_mm": float(MAX_ACC_MM_S2),
        "max_angular_rad": float(MAX_ANGULAR_RAD_S),
        "max_angular_acc_rad": float(MAX_ANGULAR_ACC_RAD_S2),
        "lookahead_mm": float(LOOKAHEAD_MM),
        "robot_front_mm": float(ROBOT_FRONT_MM),
        "robot_rear_mm": float(ROBOT_REAR_MM),
        "robot_half_width_mm": float(ROBOT_HALF_WIDTH_MM),
        "tolerance_mm": float(TOLERANCE_MM),
        "cost_gains": COST_GAINS,
        "predict_time_s": float(PREDICT_TIME_S),
        "velocity_samples": VELOCITY_SAMPLES,
        "obstacles_range_mm": float(OBSTACLES_RANGE_MM),
    }

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=LIDAR_MOUNT_X_MM,
            y_mm=LIDAR_MOUNT_Y_MM,
            theta_deg=LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=LIDAR_RANGE_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        
        # Added GPS fusion and offset parameters from Pure Pursuit script
        robot.set_gps_offset(-265.6, -111.4) # difference between rover pos and GPS pos
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID} with alpha {GPS_POSITION_ALPHA}")
        
    if ENABLE_VISION:
        robot.enable_vision()
        print("[sensor] Vision enabled — subscribing to YOLO detections")

def find_traffic_light_color(robot: Robot) -> str | None:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color = color_attribute.get("value")
        if color not in ("red", "green"):
            continue

        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)

    return best_color

def is_stop_sign_detected(robot: Robot) -> bool:
    """Return True if a stop sign is detected with sufficient confidence."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False
        
    for sign in robot.get_detections("stop sign"):
        if float(sign["confidence"]) >= MIN_STOP_SIGN_CONFIDENCE:
            return True
            
    return False

def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)

def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)

def print_status(robot: Robot) -> None:
    if ENABLE_GPS and robot.has_fused_pose():
        x, y, theta = robot.get_fused_pose()
        label = "fused"
    else:
        x, y, theta = robot.get_odometry_pose()
        label = "odom "
    print(f"  {label}=({x:6.0f}, {y:6.0f}) mm  θ={theta:5.1f}°")

def init_dwa_goal(robot: Robot, period: float):
    """Injects the DWA parameters into the Robot's planner."""
    cfg = resolve_dwa_config()
    robot._init_dwa_planner(
        max_vel_mm=cfg["max_vel_mm"],
        max_acc_mm=cfg["max_acc_mm"],
        max_angular_rad=cfg["max_angular_rad"],
        max_angular_acc_rad=cfg["max_angular_acc_rad"],
        lookahead_mm=cfg["lookahead_mm"],
        robot_front_mm=cfg["robot_front_mm"],
        robot_rear_mm=cfg["robot_rear_mm"],
        robot_half_width_mm=cfg["robot_half_width_mm"],
        tolerance_mm=cfg["tolerance_mm"],
        gains_of_costs=cfg["cost_gains"],
        period=period,
        predict_time=cfg["predict_time_s"],
        predict_velocity_samples_resolution=cfg["velocity_samples"],
        obstacles_range_mm=cfg["obstacles_range_mm"],
        ttc_weight=0.0
    )

# ---------------------------------------------------------------------------
# FSM run() loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    time.sleep(0.5) 
    configure_robot(robot)

    state = "INIT"
    last_status_print_at = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        if state == "INIT":
            start_robot(robot)
            reset_mission_pose(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 or show GREEN LIGHT to start DWA goal, BTN_2 to cancel")
            print(f"[CFG] velocity={MAX_VEL_MM_S:.0f} mm/s tolerance={TOLERANCE_MM:.0f} mm")
            state = "IDLE"

        elif state == "IDLE":
            button_pressed = robot.was_button_pressed(Button.BTN_1)
            green_light_seen = ENABLE_VISION and find_traffic_light_color(robot) == "green"
            
            if button_pressed or green_light_seen:
                reset_mission_pose(robot)
                show_running_leds(robot)
                init_dwa_goal(robot, period)
                
                if green_light_seen:
                    print("[VISION] Green light detected! Starting DWA Goal!")
                else:
                    print("[UI] BTN_1 pressed. Starting DWA Goal!")
                    
                last_status_print_at = now
                print("[FSM] MOVING — DWA navigation started")
                state = "MOVING"

        elif state == "MOVING":
            button_pressed = robot.was_button_pressed(Button.BTN_2)
            stop_sign_seen = ENABLE_VISION and is_stop_sign_detected(robot)

            if button_pressed or stop_sign_seen:
                robot.stop()
                show_idle_leds(robot)
                
                if stop_sign_seen:
                    print("[VISION] Stop sign detected! Stopping robot.")
                else:
                    print("[FSM] IDLE — DWA goal cancelled")
                
                state = "IDLE"
            else:
                if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now

                # Execute one tick of DWA navigation using the breadcrumb trail
                status = robot._nav_follow_path_loop(DENSE_PATH, period)

                if status == "IDLE":
                    print("[FSM] DONE — goal complete")
                    robot.stop()
                    show_idle_leds(robot)
                    print("[FSM] IDLE — press BTN_1 or show GREEN LIGHT to run again")
                    state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()