from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot
from robot.util import densify_polyline
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

ENABLE_LIDAR  = False
ENABLE_GPS    = False
ENABLE_VISION = True

# ---------------------------------------------------------------------------
# Vision Configuration
# ---------------------------------------------------------------------------

VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
MIN_STOP_SIGN_CONFIDENCE = 0.50

# ---------------------------------------------------------------------------
# Navigation & Mission Configuration
# ---------------------------------------------------------------------------

TAG_ID = 13  # Verify this matches your physical tag (11-18 expected per manual)

VELOCITY_MM_S = 200.0
LOOKAHEAD_DIST = 100.0  
TOLERANCE_MM = 20.0
GPS_POSITION_ALPHA = 0.01

PATH_CONTROL_POINTS = [ 
    (0.0, 0.0),           # 1st/starting point
    (0.0, 3432.0),        # 2nd point
    (533.4, 3432.0),      # 3rd point 
    (533.4, 624.8),       # 4th point
    (1524.0, 624.8),      # 5th point
    (1524.0, 2630.0),     # 6th point
    (2000.0, 2630.0),       
    # (0.0, 0.0),
    # (0.0, 500.0),
    # (500.0, 500.0),             
]

PATH_CONTROL_POINTS = densify_polyline(PATH_CONTROL_POINTS, spacing = 20.0)

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
        robot.set_gps_offset(-265.6, -111.4) # difference between rover pos and GPS pos
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID} with alpha {GPS_POSITION_ALPHA}")

    if ENABLE_VISION:
        robot.enable_vision()
        print("[sensor] Vision enabled — subscribing to YOLO detections")


def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the best recent red/green traffic-light result, or None."""
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


def start_path(robot: Robot):
    return robot.purepursuit_follow_path(
        waypoints=PATH_CONTROL_POINTS,
        velocity=VELOCITY_MM_S,
        lookahead=LOOKAHEAD_DIST,
        tolerance=TOLERANCE_MM,
        advance_radius=LOOKAHEAD_DIST, 
        max_angular_rad_s=1.5,
        blocking=False, 
    )


# ---------------------------------------------------------------------------
# FSM run() loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    time.sleep(0.5) 
    configure_robot(robot)

    state = "INIT"
    drive_handle = None
    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    next_tick = time.monotonic()
    
    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            print(f"[CFG] GPS Pure Pursuit initialized. Lookahead: {LOOKAHEAD_DIST}mm")
            print(f"[CFG] GPS Enabled: {ENABLE_GPS} | LiDAR Enabled: {ENABLE_LIDAR} | Vision Enabled: {ENABLE_VISION}")
            print("[FSM] IDLE - Press BTN_1 or show GREEN LIGHT to enter MOVING state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            
            # Check for physical button press or a green light detection
            button_pressed = robot.get_button(Button.BTN_1)
            green_light_seen = ENABLE_VISION and find_traffic_light_color(robot) == "green"

            if button_pressed or green_light_seen:
                start_robot(robot) 
                if green_light_seen:
                    print("[VISION] Green light detected! Start Moving!")
                else:
                    print("[UI] BTN_1 pressed. Start Moving!")
                    
                drive_handle = start_path(robot)
                state = "MOVING"
            
            # NOTE: BTN_2 shutdown is retained here for emergency shutdown from IDLE
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot.")
                robot.shutdown()

        elif state == "MOVING":
            show_moving_leds(robot)
            
            # Check for emergency stop inputs
            button_pressed = robot.get_button(Button.BTN_2)
            stop_sign_seen = ENABLE_VISION and is_stop_sign_detected(robot)
            
            # Emergency Stop / Cancel
            if button_pressed or stop_sign_seen:
                if stop_sign_seen:
                    print("[VISION] Stop sign detected! Stopping robot.")
                else:
                    print("[UI] BTN_2 pressed. Stopping robot.")
                    
                if drive_handle is not None:
                    drive_handle.cancel()
                    drive_handle.wait(timeout=1.0)
                    drive_handle = None
                robot.stop()
                print("Path cancelled. Returning to IDLE.")
                state = "IDLE"
            else:
                # Background task check
                if drive_handle is not None and drive_handle.is_finished():
                    print("MOVING: Target reached! Stopping.")
                    drive_handle = None
                    robot.stop()
                    state = "IDLE"

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()