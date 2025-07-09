import time
import board
import busio
import sys
import os
import storage
import keypad
import adafruit_vl53l1x
import adafruit_bh1750
import digitalio
import neopixel

# Import the custom display library
from parking_guide_display import ParkingGuideDisplay

# =================================================================
# --- Integrated Hardware and Settings Classes ---
# =================================================================

class Settings:
    FILE_PATH = "/settings.txt"
    TEMP_FILE_PATH = "/settings.tmp.txt"

    def save_parked_distance(self, distance_to_save):
        if distance_to_save is None:
            print("Error: Cannot save None distance.")
            return None
        try:
            storage.remount("/", readonly=False)
            with open(Settings.TEMP_FILE_PATH, "w") as f:
                f.write(str(distance_to_save))
            os.rename(Settings.TEMP_FILE_PATH, Settings.FILE_PATH)
            print(f'Settings saved: {distance_to_save}')
            return distance_to_save
        except (OSError, ValueError) as e:
            print(f'Error saving settings: {e}')
            try:
                os.remove(Settings.TEMP_FILE_PATH)
            except OSError:
                pass
            return None
        finally:
            storage.remount("/", readonly=True)

    def load_parked_distance(self):
        try:
            with open(Settings.FILE_PATH, "r") as f:
                data = f.read().strip()
                parked_distance = float(data)
                print(f'Loaded parked distance: {parked_distance}')
                return parked_distance
        except (OSError, ValueError):
            print('No saved parked distance found or file is invalid.')
            return None

class Sensors:
    def __init__(self, i2c_bus, button_pin):
        # --- ToF Distance Sensor ---
        self.tof_sensor = adafruit_vl53l1x.VL53L1X(i2c_bus)
        self.tof_sensor.distance_mode = 2
        self.tof_sensor.timing_budget = 50
        self.tof_current_mode = 2  # 1 for Short, 2 for Long
        self.tof_switch_threshold_cm = 130.0

        # --- Light Sensor ---
        self.light_sensor = None
        try:
            self.light_sensor = adafruit_bh1750.BH1750(i2c_bus)
            print("BH1750 Light Sensor initialized.")
        except (ValueError, RuntimeError) as e:
            print(f"Could not initialize BH1750 light sensor: {e}")

        # --- Button ---
        self.button = keypad.Keys((button_pin,), value_when_pressed=False, pull=True)

    def start_ranging(self):
        self.tof_sensor.start_ranging()

    def stop_ranging(self):
        self.tof_sensor.stop_ranging()

    def get_distance(self):
        if self.tof_sensor.data_ready:
            distance = self.tof_sensor.distance
            self.tof_sensor.clear_interrupt()
            if distance is not None:
                if distance < self.tof_switch_threshold_cm and self.tof_current_mode == 2:
                    self.tof_sensor.distance_mode = 1
                    self.tof_current_mode = 1
                elif distance >= self.tof_switch_threshold_cm and self.tof_current_mode == 1:
                    self.tof_sensor.distance_mode = 2
                    self.tof_current_mode = 2
                return distance
        return None

    def get_light_level(self):
        if self.light_sensor:
            try:
                return self.light_sensor.lux
            except Exception as e:
                print(f"Could not read light sensor: {e}")
        return None

# =================================================================
# --- Configuration Dictionaries ---
# =================================================================
display_configs = {
    "UPPER_RANGE_CM": 190.0, "MAX_MATRICES": 4,
    "POSSIBLE_ADDRESSES": [0x70, 0x71, 0x72, 0x73], "BRIGHTNESS_LEVEL": 1.0,
    "MATRIX_COLUMNS_PER_UNIT": 8, "NEOPIXEL_NUM_LEDS": 11,
    "NEOPIXEL_PIXEL_ORDER": "GRB", "NEOPIXEL_OE_PIN": board.GP17,
    "NEOPIXEL_USE_OE_PIN": True, "TOF_READING_ERROR": -1.0,
    "FLOAT_COMPARISON_TOLERANCE": 0.0001, "PHASE2_MIN_COLS_PER_ONE_PERCENT": 0.1,
    "ROUNDING_OFFSET": 0.5, "CONFIG_PHASE1_PADDING_COLS": 2,
    "CONFIG_LEFT_STATIC_COLOR": "green", "CONFIG_CENTER_STATIC_COLOR": "off",
    "CONFIG_RIGHT_STATIC_COLOR": "green", 
    "CONFIG_PHASE2_STATIC_MARKER_OFFSET_PCT": 3.0,
    "CONFIG_ZOOM_TRANSITION_THRESHOLD_PERCENT": 75.0,
    "CONFIG_PHASE2_COLS_PER_ONE_PERCENT": 2.0, "CONFIG_PRECISE_INDICATOR_COLOR": "yellow",
    "CONFIG_SLIDER_START_COLOR": "green", "CONFIG_SLIDER_END_COLOR_PHASE1": "yellow",
    "CONFIG_SLIDER_COLOR_CHANGE_POINT_TARGET_PCT": 65.0,
    "CONFIG_RECOLOR_BETWEEN_STATIC_BEHAVIOR": "use_slider_color",
    "CONFIG_SLIDER_COLOR_AFTER_1ST_STATIC_ZOOMED": "red",
    "PHASE2_ALLOW_SLIDER_OVERWRITE_TARGET": True, "PHASE2_SHOW_PRECISE_INDICATOR": True,
}

app_configs = {
    # State Machine Configs
    "ENABLE_SCORING": True, # NEW: Enable/disable the score display
    "SIGNIFICANT_LIGHT_CHANGE_LUX": 5,
    "STABLE_DISTANCE_DURATION_S": 5, # Reduced for easier testing
    "ACTIVE_RANGING_DURATION_S": 180,
    "IDLE_AFTER_PARKING_DURATION_S": 180,
    "LIGHT_MONITOR_INTERVAL_S": 3,
    "CALIBRATION_SAMPLES": 5,
    "CALIBRATION_COUNTDOWN_S": 5,
    "CONSOLE_LOG_INTERVAL_S": 5,
}

# =================================================================
# --- Main Application ---
# =================================================================
def main():
    print("--- Initializing Parking Assistant System ---")

    # --- Initialize Hardware ---
    try:
        i2c_sensor_bus = busio.I2C(board.GP11, board.GP10)
        i2c_matrix_bus = busio.I2C(board.GP21, board.GP20)
        sensor_manager = Sensors(i2c_sensor_bus, button_pin=board.GP15)
        settings_manager = Settings()
        guide_display = ParkingGuideDisplay(
            matrix_i2c_bus=i2c_matrix_bus,
            neopixel_pin_1=board.GP16, neopixel_pin_2=board.GP18,
            configs=display_configs
        )
    except Exception as e:
        print(f"FATAL: Could not initialize hardware. {e}")
        sys.print_exception(e)
        return

    # --- Load Settings & Determine Initial State ---
    target_distance = settings_manager.load_parked_distance()
    
    if target_distance is None:
        state = "AWAITING_CALIBRATION"
    else:
        state = "MONITORING_LIGHT"
        guide_display.set_neopixels("green")
        time.sleep(1)
        guide_display.set_neopixels("off")

    # --- State Machine Variables ---
    last_light_check_time = time.monotonic()
    last_lux_reading = sensor_manager.get_light_level() or 0
    state_enter_time = time.monotonic()
    stable_since_time = time.monotonic()
    last_log_time = time.monotonic()
    previous_state = ""
    last_known_distance = 0

    # --- Calculate and Print Parking Window ---
    def print_parking_window(current_target):
        if current_target is None:
            return
        upper_range = display_configs["UPPER_RANGE_CM"]
        offset_pct = display_configs["CONFIG_PHASE2_STATIC_MARKER_OFFSET_PCT"]
        tolerance_cm = (offset_pct / 100.0) * upper_range
        
        lower_bound = current_target - tolerance_cm
        upper_bound = current_target + tolerance_cm
        
        print("-" * 40)
        print(f"Target Distance: {current_target:.1f} cm")
        print(f"Parking Tolerance: +/- {tolerance_cm:.1f} cm (based on {offset_pct}%)")
        print(f"Valid Parking Window: {lower_bound:.1f} cm to {upper_bound:.1f} cm")
        
        # Print scoring breakdown
        print("Scoring (0 is perfect):")
        print(f"  Score 0: {current_target - 1:.1f} cm to {current_target + 1:.1f} cm")
        for i in range(1, 10):
            step = (tolerance_cm - 1) / 9
            min_err = 1 + (i - 1) * step
            max_err = 1 + i * step
            print(f"  Score {i}: {min_err:.1f} to {max_err:.1f} cm away from target")
        print("-" * 40)

    print_parking_window(target_distance)
    print(f"\n--- System Ready. Initial state: {state} ---")

    # --- Main Loop (State Machine) ---
    try:
        while True:
            now = time.monotonic()
            
            if state != previous_state:
                print(f"\n--- State Change: {previous_state} -> {state} ---")
                previous_state = state
                state_enter_time = now
                last_log_time = now # Reset log timer on state change

            event = sensor_manager.button.events.get()
            if event and event.pressed:
                if state != "CALIBRATING":
                    print("Button override! Starting calibration...")
                    state = "CALIBRATING"
                    sensor_manager.stop_ranging()
                    continue

            if state == "AWAITING_CALIBRATION":
                if (now // 1) % 2 == 0:
                    guide_display.set_neopixels("yellow")
                else:
                    guide_display.set_neopixels("off")
                if now - last_log_time > app_configs["CONSOLE_LOG_INTERVAL_S"]:
                    print("Awaiting calibration... Press button to begin.")
                    last_log_time = now
                
            elif state == "CALIBRATING":
                guide_display.set_neopixels("red")
                for i in range(app_configs["CALIBRATION_COUNTDOWN_S"], 0, -1):
                    guide_display.show_countdown(i)
                    time.sleep(1)
                guide_display.clear()

                print("Taking distance samples...")
                sensor_manager.start_ranging()
                time.sleep(0.5)
                readings = []
                while len(readings) < app_configs["CALIBRATION_SAMPLES"]:
                    dist = sensor_manager.get_distance()
                    if dist is not None:
                        readings.append(dist)
                        print(f"  Sample {len(readings)}: {dist:.1f} cm")
                        time.sleep(0.2)
                sensor_manager.stop_ranging()
                
                if readings:
                    avg_dist = sum(readings) / len(readings)
                    target_distance = settings_manager.save_parked_distance(avg_dist)
                    if target_distance is not None:
                        print_parking_window(target_distance)
                        guide_display.set_neopixels("green")
                        time.sleep(2)
                        state = "MONITORING_LIGHT"
                        last_lux_reading = sensor_manager.get_light_level() or 0
                    else:
                        state = "AWAITING_CALIBRATION"
                else:
                    state = "AWAITING_CALIBRATION"

            elif state == "MONITORING_LIGHT":
                guide_display.clear()
                if now - last_light_check_time > app_configs["LIGHT_MONITOR_INTERVAL_S"]:
                    current_lux = sensor_manager.get_light_level()
                    last_light_check_time = now
                    if current_lux is not None:
                        light_change = abs(current_lux - last_lux_reading)
                        print(f"Light check: Current={current_lux:.1f} lux, Last={last_lux_reading:.1f} lux, Change={light_change:.1f} lux")
                        if light_change > app_configs["SIGNIFICANT_LIGHT_CHANGE_LUX"]:
                            print(">>> Significant light change detected! Checking distance...")
                            sensor_manager.start_ranging()
                            time.sleep(0.5)
                            dist = sensor_manager.get_distance()
                            sensor_manager.stop_ranging()
                            
                            last_lux_reading = current_lux
                            
                            tolerance_cm = (display_configs["CONFIG_PHASE2_STATIC_MARKER_OFFSET_PCT"] / 100.0) * display_configs["UPPER_RANGE_CM"]
                            if dist is None or abs(dist - target_distance) > tolerance_cm:
                                state = "ACTIVE_RANGING"
                            else:
                                state = "IDLE_COOLDOWN"
                    

            elif state == "ACTIVE_RANGING":
                sensor_manager.start_ranging()
                current_distance = sensor_manager.get_distance()
                if current_distance is not None:
                    last_known_distance = current_distance
                guide_display.update(current_distance, target_distance)
                
                if now - last_log_time > app_configs["CONSOLE_LOG_INTERVAL_S"]:
                    print(f"Ranging... Dist: {last_known_distance:.1f} cm. Timeout in {app_configs['ACTIVE_RANGING_DURATION_S'] - (now - state_enter_time):.0f}s")
                    last_log_time = now

                if now - state_enter_time > app_configs["ACTIVE_RANGING_DURATION_S"]:
                    state = "IDLE_COOLDOWN"
                
                tolerance_cm = (display_configs["CONFIG_PHASE2_STATIC_MARKER_OFFSET_PCT"] / 100.0) * display_configs["UPPER_RANGE_CM"]
                if current_distance is not None and abs(current_distance - target_distance) <= tolerance_cm:
                    if now - stable_since_time > app_configs["STABLE_DISTANCE_DURATION_S"]:
                        state = "SHOWING_SCORE"
                else:
                    stable_since_time = now

            elif state == "SHOWING_SCORE":
                sensor_manager.stop_ranging()
                if app_configs["ENABLE_SCORING"]:
                    tolerance_cm = (display_configs["CONFIG_PHASE2_STATIC_MARKER_OFFSET_PCT"] / 100.0) * display_configs["UPPER_RANGE_CM"]
                    error = abs(last_known_distance - target_distance)
                    
                    score = 9
                    if error <= 1.0:
                        score = 0
                    else:
                        step = (tolerance_cm - 1.0) / 9.0
                        if step > 0:
                            score = int((error - 1.0) / step) + 1
                            score = min(9, max(1, score))
                            
                    print(f"Final distance: {last_known_distance:.1f} cm, Error: {error:.1f} cm, Score: {score}")
                    guide_display.show_score(score)
                    time.sleep(5)
                state = "IDLE_COOLDOWN"

            elif state == "IDLE_COOLDOWN":
                guide_display.clear()
                sensor_manager.stop_ranging()
                if now - last_log_time > app_configs["CONSOLE_LOG_INTERVAL_S"]:
                    print(f"In cooldown... Resuming monitoring in {app_configs['IDLE_AFTER_PARKING_DURATION_S'] - (now - state_enter_time):.0f}s")
                    last_log_time = now
                if now - state_enter_time > app_configs["IDLE_AFTER_PARKING_DURATION_S"]:
                    state = "MONITORING_LIGHT"
                    last_lux_reading = sensor_manager.get_light_level() or 0

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nCtrl+C detected.")
    except Exception as e:
        print(f"\nAn unexpected error occurred in the main loop: {e}")
        sys.print_exception(e)
    finally:
        print("Performing cleanup...")
        guide_display.clear()
        sensor_manager.stop_ranging()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()
