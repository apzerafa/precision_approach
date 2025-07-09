import board
import busio
from adafruit_ht16k33.matrix import Matrix8x8x2
import time
import traceback # Retained for development, but sys.print_exception will be used
import sys # For sys.print_exception
import adafruit_vl53l1x
import neopixel
import digitalio

# --- Core Configuration Constants ---

# --- General System Configuration ---
UPPER_RANGE_CM = 190.0  # Max effective ToF sensor range
# IMPORTANT: Set this to your actual 'parked' reference distance in cm.
CONFIG_PARKED_DISTANCE_CM = 25
MAX_MATRICES = 4  # Maximum number of matrix displays to initialize
POSSIBLE_ADDRESSES = [0x70, 0x71, 0x72, 0x73] # Assumed order from left to right
BRIGHTNESS_LEVEL = 1.0 # Set a default brightness (0.0 to 1.0)
MATRIX_COLUMNS_PER_UNIT = 8 # Columns per individual matrix display

# --- NeoPixel Configuration ---
NEOPIXEL_NUM_LEDS = 11         # Number of LEDs in EACH strip
NEOPIXEL_DATA_PIN_1 = board.GP16 # The Pico pin for the first strip
NEOPIXEL_DATA_PIN_2 = board.GP18 # The Pico pin for the second strip
NEOPIXEL_PIXEL_ORDER = neopixel.GRB # Correct order for WS2812B
# Optional Level Shifter OE Pin Control
NEOPIXEL_OE_PIN = board.GP17
NEOPIXEL_USE_OE_PIN = True # Set to False if not using an OE pin

# --- ToF Sensor Error Codes ---
TOF_READING_ERROR = -1.0
TOF_READING_NONE = -2.0
TOF_DATA_NOT_READY = -3.0
TOF_SENSOR_NOT_INIT = -4.0

# --- Numerical & Calculation Constants ---
FLOAT_COMPARISON_TOLERANCE = 0.0001
PHASE2_MIN_COLS_PER_ONE_PERCENT = 0.1
ROUNDING_OFFSET = 0.5

# --- Display Logic Configuration (Direct CircuitPython Colors) ---
CONFIG_PHASE1_PADDING_COLS = 2

CONFIG_LEFT_STATIC_COLOR = "green"
CONFIG_CENTER_STATIC_COLOR = "off" # Default for center if not otherwise specified
CONFIG_RIGHT_STATIC_COLOR = "green"

CONFIG_PHASE2_LEFT_STATIC_PCT_ORIGINAL = 84.5 # This is relative to the 0-100 scale
CONFIG_PHASE2_RIGHT_STATIC_PCT_ORIGINAL = 90.5 # This is relative to the 0-100 scale

CONFIG_ZOOM_TRANSITION_THRESHOLD_PERCENT = 75.0 # For current_progress_percent
CONFIG_PHASE2_COLS_PER_ONE_PERCENT = 2.0

CONFIG_PRECISE_INDICATOR_COLOR = "yellow"

CONFIG_SLIDER_START_COLOR = "green"
CONFIG_SLIDER_END_COLOR_PHASE1 = "yellow"
CONFIG_SLIDER_COLOR_CHANGE_POINT_TARGET_PCT = 65.0 # Percentage of target distance

# Behavior for coloring segments between static markers.
CONFIG_RECOLOR_BETWEEN_STATIC_BEHAVIOR = "use_slider_color"
CONFIG_SLIDER_COLOR_AFTER_1ST_STATIC_ZOOMED = "red"

PHASE2_ALLOW_SLIDER_OVERWRITE_TARGET = True
PHASE2_SHOW_PRECISE_INDICATOR = True

# --- HT16K33 Blink Rates (kept for reference/cleanup) ---
HT16K33_BLINK_OFF = 0

# --- Global Hardware & State Variables ---
i2c_matrices_bus = None
matrices = [] # List of Matrix8x8x2 objects
pixels1 = None # NeoPixel strip 1
pixels2 = None # NeoPixel strip 2
oe_enable = None # Level shifter output enable pin
TOTAL_DISPLAY_COLUMNS = 0
vl53l1x_sensor_global = None # Global for the ToF sensor object

# --- Color Definitions ---
# For LED Matrices
MATRIX_COLOR_MAP = {
    "off": Matrix8x8x2.LED_OFF,
    "red": Matrix8x8x2.LED_RED,
    "green": Matrix8x8x2.LED_GREEN,
    "yellow": Matrix8x8x2.LED_YELLOW
}
# For NeoPixels (using same color names)
NEOPIXEL_COLOR_MAP = {
    "off": (0, 0, 0),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "yellow": (255, 45, 0), # Orange-Yellow color for NeoPixels
    "blue": (0, 0, 255) # Blue for error state
}


# --- Helper Functions ---

def set_single_column_color_by_global_index(global_col_idx, cp_color_name):
    """Sets all LEDs in a globally indexed column to a specified CircuitPython color name."""
    if not matrices or not (0 <= global_col_idx < TOTAL_DISPLAY_COLUMNS):
        return

    numerical_color_value = MATRIX_COLOR_MAP.get(cp_color_name, Matrix8x8x2.LED_OFF)
    if cp_color_name not in MATRIX_COLOR_MAP:
        print(f"Warning: Unknown matrix color name '{cp_color_name}'. Defaulting to 'off'.")

    matrix_idx = global_col_idx // MATRIX_COLUMNS_PER_UNIT
    logical_c_index_in_matrix = global_col_idx % MATRIX_COLUMNS_PER_UNIT
    physical_column_to_target = logical_c_index_in_matrix

    if 0 <= matrix_idx < len(matrices):
        matrix_obj = matrices[matrix_idx]
        for row_index in range(8):
            try:
                matrix_obj[physical_column_to_target, row_index] = numerical_color_value
            except IndexError:
                print(f"Error: IndexError in set_single_column_color: col={physical_column_to_target}, row={row_index} for matrix {matrix_idx}")

def calculate_current_tof_percentage(current_tof_reading_cm, upper_range_cm_config):
    """Calculates progress percentage based on ToF reading."""
    if current_tof_reading_cm < 0:
        return TOF_READING_ERROR
    if current_tof_reading_cm > upper_range_cm_config:
        return TOF_READING_ERROR
    if upper_range_cm_config == 0:
        return 0.0 if current_tof_reading_cm > 0 else 100.0

    raw_percentage = ((upper_range_cm_config - current_tof_reading_cm) / upper_range_cm_config) * 100.0
    return max(0.0, min(100.0, raw_percentage))

def calculate_derived_target_percentage(parked_distance_cm_config, upper_range_cm_config):
    """Calculates the target percentage based on the configured parked distance."""
    if not (0 <= parked_distance_cm_config <= upper_range_cm_config):
        print(f"Error: Invalid CONFIG_PARKED_DISTANCE_CM ({parked_distance_cm_config}).")
        return 50.0
    if upper_range_cm_config == 0:
        return 0.0

    raw_percentage = ((upper_range_cm_config - parked_distance_cm_config) / upper_range_cm_config) * 100.0
    return max(0.0, min(100.0, raw_percentage))


# --- Pattern Details Calculation ---
def _get_phase1_pattern_details(target_original_pct, total_display_cols, config_padding_cols,
                               left_static_color, center_static_color, right_static_color):
    """Helper to calculate pattern details for Phase 1."""
    pattern = {
        'left_display_col_start': -1, 'left_display_col_end': -1, 'left_color': left_static_color,
        'center_display_col_start': -1, 'center_display_col_end': -1, 'center_color': center_static_color,
        'right_display_col_start': -1, 'right_display_col_end': -1, 'right_color': right_static_color,
        'left_original_pct': -1.0, 'center_original_pct': target_original_pct, 'right_original_pct': -1.0
    }
    if total_display_cols == 0: return pattern

    center_display_col = min(total_display_cols - 1, max(0, int((target_original_pct / 100.0) * total_display_cols)))
    pattern['center_display_col_start'] = center_display_col
    pattern['center_display_col_end'] = center_display_col

    left_col = center_display_col - config_padding_cols - 1
    if left_col >= 0:
        pattern['left_display_col_start'] = left_col
        pattern['left_display_col_end'] = left_col
        pattern['left_original_pct'] = (float(left_col) / total_display_cols) * 100.0

    right_col = center_display_col + config_padding_cols + 1
    if right_col < total_display_cols:
        pattern['right_display_col_start'] = right_col
        pattern['right_display_col_end'] = right_col
        pattern['right_original_pct'] = (float(right_col) / total_display_cols) * 100.0
    return pattern

def _get_phase2_static_marker_details(static_marker_original_pct, target_original_pct_for_zoom_center,
                                     total_display_cols, phase2_cols_per_one_percent_config,
                                     viewport_start_original_pct, phase2_view_coverage_pct,
                                     is_left_marker, is_right_marker, center_marker_details):
    """Helper to calculate display columns for a single static marker in Phase 2."""
    display_col_start, display_col_end = -1, -1
    if total_display_cols == 0 or phase2_cols_per_one_percent_config <= 0:
        return display_col_start, display_col_end

    original_pct_per_display_col = 1.0 / phase2_cols_per_one_percent_config
    num_display_cols_for_static_marker = max(1, int(phase2_cols_per_one_percent_config * 1.0))

    if static_marker_original_pct >= viewport_start_original_pct - FLOAT_COMPARISON_TOLERANCE and \
       static_marker_original_pct < viewport_start_original_pct + phase2_view_coverage_pct + FLOAT_COMPARISON_TOLERANCE:
        
        offset_from_viewport_start_pct = static_marker_original_pct - viewport_start_original_pct
        
        if is_left_marker:
            ideal_right_edge_fractional = offset_from_viewport_start_pct / original_pct_per_display_col
            display_col_end = int(ideal_right_edge_fractional)
            display_col_start = display_col_end - num_display_cols_for_static_marker + 1
        elif is_right_marker:
            ideal_left_edge_fractional = offset_from_viewport_start_pct / original_pct_per_display_col
            display_col_start = int(ideal_left_edge_fractional)
            display_col_end = display_col_start + num_display_cols_for_static_marker - 1
        else: # Center marker
            ideal_center_fractional = offset_from_viewport_start_pct / original_pct_per_display_col
            start_col_raw = ideal_center_fractional - (num_display_cols_for_static_marker / 2.0)
            display_col_start = int(start_col_raw + ROUNDING_OFFSET)
            display_col_end = display_col_start + num_display_cols_for_static_marker - 1

        display_col_start = max(0, display_col_start)
        display_col_end = min(total_display_cols - 1, display_col_end)

        if display_col_start > display_col_end:
             display_col_start, display_col_end = -1, -1
        elif center_marker_details and not (is_left_marker or is_right_marker):
            pass
        elif center_marker_details and center_marker_details['start'] != -1:
            center_start = center_marker_details['start']
            center_end = center_marker_details['end']
            overlap = (display_col_start <= center_end) and (display_col_end >= center_start)
            if overlap and (is_left_marker or is_right_marker):
                display_col_start, display_col_end = -1, -1

    return display_col_start, display_col_end


def get_pattern_display_details(current_slider_original_pct, is_phase2_active, derived_target_pct,
                                total_display_cols, display_configs):
    """Calculates the display column ranges and colors for static markers."""
    cfg_p1_pad = display_configs['phase1_padding_cols']
    cfg_left_color = display_configs['left_static_color']
    cfg_center_color = display_configs['center_static_color']
    cfg_right_color = display_configs['right_static_color']
    cfg_p2_left_pct = display_configs['phase2_left_static_pct']
    cfg_p2_right_pct = display_configs['phase2_right_static_pct']
    cfg_p2_cols_per_pct = display_configs['phase2_cols_per_one_percent']

    pattern = {
        'left_display_col_start': -1, 'left_display_col_end': -1, 'left_color': cfg_left_color,
        'center_display_col_start': -1, 'center_display_col_end': -1, 'center_color': cfg_center_color,
        'right_display_col_start': -1, 'right_display_col_end': -1, 'right_color': cfg_right_color,
        'left_original_pct': -1.0, 'center_original_pct': derived_target_pct, 'right_original_pct': -1.0,
        'viewport_original_start_pct': 0.0, 'original_pct_per_display_col': 0.0, 'phase2_view_coverage_pct': 0.0
    }
    if total_display_cols == 0: return pattern

    if is_phase2_active:
        effective_cols_per_original_percent = max(PHASE2_MIN_COLS_PER_ONE_PERCENT, cfg_p2_cols_per_pct)
        original_pct_per_display_col = 1.0 / effective_cols_per_original_percent
        phase2_view_coverage_pct = total_display_cols * original_pct_per_display_col
        viewport_original_start_pct = max(0.0, derived_target_pct - (phase2_view_coverage_pct / 2.0))
        
        pattern['viewport_original_start_pct'] = viewport_original_start_pct
        pattern['original_pct_per_display_col'] = original_pct_per_display_col
        pattern['phase2_view_coverage_pct'] = phase2_view_coverage_pct

        c_start, c_end = _get_phase2_static_marker_details(
            derived_target_pct, derived_target_pct, total_display_cols,
            effective_cols_per_original_percent, viewport_original_start_pct, phase2_view_coverage_pct,
            False, False, None
        )
        pattern['center_display_col_start'] = c_start
        pattern['center_display_col_end'] = c_end
        center_marker_details_for_overlap_check = {'start': c_start, 'end': c_end}

        pattern['left_original_pct'] = cfg_p2_left_pct
        l_start, l_end = _get_phase2_static_marker_details(
            cfg_p2_left_pct, derived_target_pct, total_display_cols,
            effective_cols_per_original_percent, viewport_original_start_pct, phase2_view_coverage_pct,
            True, False, center_marker_details_for_overlap_check
        )
        pattern['left_display_col_start'] = l_start
        pattern['left_display_col_end'] = l_end
        
        pattern['right_original_pct'] = cfg_p2_right_pct
        r_start, r_end = _get_phase2_static_marker_details(
            cfg_p2_right_pct, derived_target_pct, total_display_cols,
            effective_cols_per_original_percent, viewport_original_start_pct, phase2_view_coverage_pct,
            False, True, center_marker_details_for_overlap_check
        )
        pattern['right_display_col_start'] = r_start
        pattern['right_display_col_end'] = r_end

    else: # Phase 1
        phase1_details = _get_phase1_pattern_details(derived_target_pct, total_display_cols, cfg_p1_pad,
                                                    cfg_left_color, cfg_center_color, cfg_right_color)
        pattern.update(phase1_details)
        if total_display_cols > 0:
            pattern['original_pct_per_display_col'] = 100.0 / total_display_cols
    return pattern


def calculate_phase1_slider_color_change_threshold_original_pct(center_marker_original_pct,
                                                                slider_color_change_point_target_pct_config,
                                                                derived_target_pct_overall):
    """Calculates the original percentage at which slider color changes in Phase 1."""
    if center_marker_original_pct != -1 and not isinstance(slider_color_change_point_target_pct_config, str):
        return max(0.0, (slider_color_change_point_target_pct_config / 100.0) * center_marker_original_pct)
    return (50.0 / 100.0) * derived_target_pct_overall


# --- Main Rendering Logic ---

def get_current_progress_color(current_slider_original_pct, is_phase2_active,
                               phase1_color_change_thresh_pct, pattern_details, render_configs):
    """Determines the effective color of the progress bar at its current tip."""
    # This logic mirrors _determine_slider_fill_color but is for a single point in time.
    cfg_slider_start_color = render_configs['slider_start_color']
    cfg_slider_end_color_p1 = render_configs['slider_end_color_phase1']
    cfg_slider_color_after_1st_static_zoomed = render_configs['slider_color_after_1st_static_zoomed']

    if is_phase2_active:
        phase2_initial_color = cfg_slider_end_color_p1 \
            if current_slider_original_pct >= phase1_color_change_thresh_pct \
            else cfg_slider_start_color

        first_static_original_pct_phase2 = pattern_details.get('left_original_pct', -1.0)
        if first_static_original_pct_phase2 != -1.0 and current_slider_original_pct > first_static_original_pct_phase2:
            return cfg_slider_color_after_1st_static_zoomed
        else:
            return phase2_initial_color
    else: # Phase 1
        if current_slider_original_pct > phase1_color_change_thresh_pct:
            return cfg_slider_end_color_p1
        else:
            return cfg_slider_start_color

def _determine_slider_fill_color(current_slider_original_pct, original_pct_for_this_col_start,
                                is_phase2_active, phase1_color_change_thresh_pct,
                                pattern_details, render_configs):
    """Determines the color of a column if it's part of the 'slider' progress."""
    cfg_slider_start_color = render_configs['slider_start_color']
    cfg_slider_end_color_p1 = render_configs['slider_end_color_phase1']
    
    # The current color of the slider is determined by the function above.
    # We pass the start of the column to determine its color based on phase 1 rules.
    slider_tip_color = get_current_progress_color(current_slider_original_pct, is_phase2_active, phase1_color_change_thresh_pct, pattern_details, render_configs)
    
    if is_phase2_active:
        # In phase 2, the whole slider before the tip has the same color.
        return slider_tip_color
    else: # Phase 1
        # In phase 1, color can change mid-bar.
        if original_pct_for_this_col_start >= phase1_color_change_thresh_pct:
            return cfg_slider_end_color_p1
        else:
            return cfg_slider_start_color


def render_display(current_slider_original_pct, derived_target_pct, total_display_cols,
                   pattern_details, render_configs):
    """Renders the LED matrix display based on current progress and pattern details."""
    if total_display_cols == 0 or not pattern_details:
        return

    cfg_recolor_between_static = render_configs['recolor_between_static_behavior']
    cfg_precise_indicator_color = render_configs['precise_indicator_color']
    cfg_phase2_allow_slider_overwrite = render_configs['phase2_allow_slider_overwrite_target']
    cfg_phase2_show_precise_indicator = render_configs['phase2_show_precise_indicator']
    
    is_phase2_active = current_slider_original_pct >= render_configs['zoom_transition_threshold_percent']

    viewport_original_start_pct = pattern_details['viewport_original_start_pct']
    original_pct_per_display_col = pattern_details['original_pct_per_display_col']
    phase2_view_coverage_pct = pattern_details['phase2_view_coverage_pct']

    display_column_colors = ["off"] * total_display_cols

    phase1_color_change_threshold_original_pct = calculate_phase1_slider_color_change_threshold_original_pct(
        pattern_details['center_original_pct'],
        render_configs['slider_color_change_point_target_pct'],
        derived_target_pct
    )

    for display_col_idx in range(total_display_cols):
        original_pct_for_this_col_start = viewport_original_start_pct + (display_col_idx * original_pct_per_display_col)
        center_original_pct_of_display_col = original_pct_for_this_col_start + (original_pct_per_display_col / 2.0)
        current_col_color = "off"

        if is_phase2_active:
            viewport_effective_end_pct = viewport_original_start_pct + phase2_view_coverage_pct
            if center_original_pct_of_display_col < viewport_original_start_pct - FLOAT_COMPARISON_TOLERANCE or \
               center_original_pct_of_display_col > viewport_effective_end_pct + FLOAT_COMPARISON_TOLERANCE:
                display_column_colors[display_col_idx] = "off"
                continue

        is_filled_by_slider = False
        if is_phase2_active:
            if current_slider_original_pct > viewport_original_start_pct and original_pct_per_display_col > 0:
                progress_into_viewport_pct = current_slider_original_pct - viewport_original_start_pct
                display_cols_to_fill_in_zoom = int(progress_into_viewport_pct / original_pct_per_display_col)
                if display_col_idx < display_cols_to_fill_in_zoom:
                    is_filled_by_slider = True
        else: # Phase 1
            if original_pct_for_this_col_start < current_slider_original_pct:
                is_filled_by_slider = True
        
        if is_filled_by_slider:
            current_col_color = _determine_slider_fill_color(
                current_slider_original_pct, original_pct_for_this_col_start,
                is_phase2_active, phase1_color_change_threshold_original_pct,
                pattern_details, render_configs
            )
            if cfg_recolor_between_static != "use_slider_color":
                is_between_left_and_center = (
                    pattern_details['left_display_col_start'] != -1 and
                    display_col_idx > pattern_details['left_display_col_end'] and
                    pattern_details['center_display_col_start'] != -1 and
                    display_col_idx < pattern_details['center_display_col_start']
                )
                is_between_center_and_right = (
                    pattern_details['right_display_col_start'] != -1 and
                    pattern_details['center_display_col_start'] != -1 and
                    display_col_idx > pattern_details['center_display_col_end'] and
                    display_col_idx < pattern_details['right_display_col_start']
                )
                if is_between_left_and_center or is_between_center_and_right:
                    current_col_color = cfg_recolor_between_static
        
        display_column_colors[display_col_idx] = current_col_color

    markers_to_apply = [
        ('left', pattern_details['left_display_col_start'], pattern_details['left_display_col_end'], pattern_details['left_color']),
        ('right', pattern_details['right_display_col_start'], pattern_details['right_display_col_end'], pattern_details['right_color']),
        ('center', pattern_details['center_display_col_start'], pattern_details['center_display_col_end'], pattern_details['center_color'])
    ]

    for marker_name, start_col, end_col, color in markers_to_apply:
        if start_col != -1 and end_col != -1:
            for col_idx in range(start_col, end_col + 1):
                if 0 <= col_idx < total_display_cols:
                    if marker_name == 'center' and is_phase2_active and cfg_phase2_allow_slider_overwrite:
                        if display_column_colors[col_idx] == "off":
                             display_column_colors[col_idx] = color
                    else:
                        display_column_colors[col_idx] = color
    
    if is_phase2_active and cfg_phase2_show_precise_indicator:
        precise_indicator_display_col_idx = -1
        if current_slider_original_pct >= viewport_original_start_pct - FLOAT_COMPARISON_TOLERANCE and \
           current_slider_original_pct < viewport_original_start_pct + phase2_view_coverage_pct + FLOAT_COMPARISON_TOLERANCE:
            progress_into_viewport_pct = current_slider_original_pct - viewport_original_start_pct
            if original_pct_per_display_col > 0:
                precise_indicator_display_col_idx = int(progress_into_viewport_pct / original_pct_per_display_col)
                precise_indicator_display_col_idx = max(0, min(total_display_cols - 1, precise_indicator_display_col_idx))
        
        if precise_indicator_display_col_idx != -1:
            display_column_colors[precise_indicator_display_col_idx] = cfg_precise_indicator_color

    for display_col_idx in range(total_display_cols):
        set_single_column_color_by_global_index(display_col_idx, display_column_colors[display_col_idx])
    
    for m_obj in matrices:
        try:
            m_obj.show()
        except OSError as e:
            print(f"I2C Error on matrix {hex(m_obj.address)} during show(): {e}. Check connections.")

def update_neopixel_strips(color_name="off"):
    """Sets the color of both NeoPixel strips."""
    if not pixels1 or not pixels2:
        return # Do nothing if NeoPixels are not initialized

    # Get the RGB color tuple from the map, defaulting to black (off)
    rgb_color = NEOPIXEL_COLOR_MAP.get(color_name, (0, 0, 0))
    if color_name not in NEOPIXEL_COLOR_MAP:
        print(f"Warning: Unknown NeoPixel color name '{color_name}'. Defaulting to 'off'.")

    try:
        pixels1.fill(rgb_color)
        pixels2.fill(rgb_color)
        pixels1.show()
        pixels2.show()
    except Exception as e:
        print(f"Error updating NeoPixels: {e}")

def initialize_system():
    """Initializes I2C buses, ToF sensor, LED matrices, and NeoPixels."""
    global i2c_matrices_bus, matrices, TOTAL_DISPLAY_COLUMNS, vl53l1x_sensor_global
    global pixels1, pixels2, oe_enable
    
    # --- Initialize NeoPixels ---
    if NEOPIXEL_USE_OE_PIN:
        try:
            oe_enable = digitalio.DigitalInOut(NEOPIXEL_OE_PIN)
            oe_enable.direction = digitalio.Direction.OUTPUT
            oe_enable.value = True # Enable level shifter outputs
            print("NeoPixel level shifter enabled.")
        except Exception as e:
            print(f"Could not initialize NeoPixel OE pin: {e}")

    try:
        pixels1 = neopixel.NeoPixel(
            NEOPIXEL_DATA_PIN_1, NEOPIXEL_NUM_LEDS,
            pixel_order=NEOPIXEL_PIXEL_ORDER, auto_write=False
        )
        pixels2 = neopixel.NeoPixel(
            NEOPIXEL_DATA_PIN_2, NEOPIXEL_NUM_LEDS,
            pixel_order=NEOPIXEL_PIXEL_ORDER, auto_write=False
        )
        print("NeoPixel strips initialized.")
    except Exception as e:
        print(f"Error initializing NeoPixel strips: {e}")
        pixels1 = None
        pixels2 = None

    # --- Initialize ToF Sensor ---
    try:
        tof_i2c_bus = busio.I2C(board.GP11, board.GP10)
        vl53l1x_sensor_global = adafruit_vl53l1x.VL53L1X(tof_i2c_bus)
        vl53l1x_sensor_global.distance_mode = 2
        vl53l1x_sensor_global.timing_budget = 50
        vl53l1x_sensor_global.start_ranging()
        print("VL53L1X ToF sensor initialized and ranging started.")
    except Exception as e:
        print(f"Error initializing VL53L1X ToF sensor: {e}")
        sys.print_exception(e)
        vl53l1x_sensor_global = None

    # --- Initialize Matrices ---
    scl_pin_name_matrices = 'GP21'
    sda_pin_name_matrices = 'GP20'
    try:
        if not (hasattr(board, scl_pin_name_matrices) and hasattr(board, sda_pin_name_matrices)):
            print(f"Error: Matrix SCL/SDA pins not defined in 'board'.")
            return False
        scl_pin_m = getattr(board, scl_pin_name_matrices)
        sda_pin_m = getattr(board, sda_pin_name_matrices)
        i2c_matrices_bus = busio.I2C(scl_pin_m, sda_pin_m)
    except Exception as e:
        print(f"Error initializing I2C bus for matrices: {e}")
        sys.print_exception(e)
        return False
    
    found_addresses = []
    bus_locked_for_scan = False
    try:
        while not i2c_matrices_bus.try_lock():
            time.sleep(0.01)
        bus_locked_for_scan = True
        found_addresses = i2c_matrices_bus.scan()
    except Exception as e:
        print(f"Error during matrix I2C scan: {e}")
        sys.print_exception(e)
    finally:
        if bus_locked_for_scan:
            i2c_matrices_bus.unlock()

    temp_matrices = []
    for addr in POSSIBLE_ADDRESSES:
        if addr in found_addresses:
            if len(temp_matrices) < MAX_MATRICES:
                try:
                    matrix_instance = Matrix8x8x2(i2c_matrices_bus, address=addr)
                    matrix_instance.auto_write = False
                    matrix_instance.brightness = BRIGHTNESS_LEVEL
                    matrix_instance.fill(MATRIX_COLOR_MAP["off"])
                    matrix_instance.show()
                    temp_matrices.append(matrix_instance)
                except Exception as e:
                    print(f"Failed to initialize matrix at {hex(addr)}: {e}")
            else:
                break
    matrices = temp_matrices
    TOTAL_DISPLAY_COLUMNS = len(matrices) * MATRIX_COLUMNS_PER_UNIT

    if not matrices:
        print("Warning: No matrix displays were successfully initialized.")
    
    print(f"Initialized {len(matrices)} matrices, total {TOTAL_DISPLAY_COLUMNS} display columns.")
    
    if vl53l1x_sensor_global is None and not matrices:
        print("Critical: Neither ToF sensor nor any matrix could be initialized.")
        return False
    return True


def fetch_tof_sensor_distance_cm():
    """Fetches distance from ToF sensor, returns value in cm or an error code."""
    if vl53l1x_sensor_global is None:
        return TOF_SENSOR_NOT_INIT
    try:
        if vl53l1x_sensor_global.data_ready:
            distance_val = vl53l1x_sensor_global.distance
            vl53l1x_sensor_global.clear_interrupt()
            if distance_val is not None:
                return float(distance_val)
            else:
                return TOF_READING_NONE
        else:
            return TOF_DATA_NOT_READY
    except Exception as e:
        print(f"Error reading ToF sensor: {e}")
        return TOF_READING_ERROR


def main():
    if not initialize_system():
        print("System initialization failed. Exiting.")
        return

    derived_target_pct_original = calculate_derived_target_percentage(CONFIG_PARKED_DISTANCE_CM, UPPER_RANGE_CM)

    pattern_display_configs = {
        'phase1_padding_cols': CONFIG_PHASE1_PADDING_COLS,
        'left_static_color': CONFIG_LEFT_STATIC_COLOR,
        'center_static_color': CONFIG_CENTER_STATIC_COLOR,
        'right_static_color': CONFIG_RIGHT_STATIC_COLOR,
        'phase2_left_static_pct': CONFIG_PHASE2_LEFT_STATIC_PCT_ORIGINAL,
        'phase2_right_static_pct': CONFIG_PHASE2_RIGHT_STATIC_PCT_ORIGINAL,
        'phase2_cols_per_one_percent': CONFIG_PHASE2_COLS_PER_ONE_PERCENT,
    }

    render_configs = {
        'slider_start_color': CONFIG_SLIDER_START_COLOR,
        'slider_end_color_phase1': CONFIG_SLIDER_END_COLOR_PHASE1,
        'slider_color_change_point_target_pct': CONFIG_SLIDER_COLOR_CHANGE_POINT_TARGET_PCT,
        'recolor_between_static_behavior': CONFIG_RECOLOR_BETWEEN_STATIC_BEHAVIOR,
        'slider_color_after_1st_static_zoomed': CONFIG_SLIDER_COLOR_AFTER_1ST_STATIC_ZOOMED,
        'precise_indicator_color': CONFIG_PRECISE_INDICATOR_COLOR,
        'phase2_allow_slider_overwrite_target': PHASE2_ALLOW_SLIDER_OVERWRITE_TARGET,
        'phase2_show_precise_indicator': PHASE2_SHOW_PRECISE_INDICATOR,
        'zoom_transition_threshold_percent': CONFIG_ZOOM_TRANSITION_THRESHOLD_PERCENT,
    }

    print(f"\nStarting ToF-driven Matrix Simulation...")
    print(f"Parked distance: {CONFIG_PARKED_DISTANCE_CM} cm -> Target: {derived_target_pct_original:.2f}%")
    print(f"ToF Upper Range: {UPPER_RANGE_CM} cm")
    if TOTAL_DISPLAY_COLUMNS == 0 and vl53l1x_sensor_global is not None:
        print("ToF sensor initialized, but no display columns. Will print readings only.")

    try:
        while True:
            current_tof_reading_cm = fetch_tof_sensor_distance_cm()
            
            current_progress_percent = TOF_READING_ERROR
            if current_tof_reading_cm > TOF_READING_ERROR:
                current_progress_percent = calculate_current_tof_percentage(current_tof_reading_cm, UPPER_RANGE_CM)

            tof_is_error_or_out_of_range = (current_progress_percent <= TOF_READING_ERROR)
            effective_progress_for_pattern = 0.0 if tof_is_error_or_out_of_range else current_progress_percent
            
            if TOTAL_DISPLAY_COLUMNS == 0:
                if vl53l1x_sensor_global is not None:
                    status_msg = f"ToF Reading: {current_tof_reading_cm:.1f} cm"
                    if tof_is_error_or_out_of_range:
                        status_msg += f" -> Status Code: {current_progress_percent}"
                    else:
                        status_msg += f" -> Progress: {current_progress_percent:.2f}%"
                    print(status_msg)
                time.sleep(0.1)
                continue

            # --- Display Logic ---
            try:
                is_phase2_active = effective_progress_for_pattern >= CONFIG_ZOOM_TRANSITION_THRESHOLD_PERCENT
                
                pattern_details = get_pattern_display_details(
                    effective_progress_for_pattern, is_phase2_active, derived_target_pct_original,
                    TOTAL_DISPLAY_COLUMNS, pattern_display_configs
                )
                
                # --- NeoPixel Update Logic ---
                if tof_is_error_or_out_of_range:
                    update_neopixel_strips("blue")
                else:
                    phase1_color_change_thresh = calculate_phase1_slider_color_change_threshold_original_pct(
                        pattern_details['center_original_pct'],
                        render_configs['slider_color_change_point_target_pct'],
                        derived_target_pct_original
                    )
                    current_color = get_current_progress_color(
                        effective_progress_for_pattern, is_phase2_active,
                        phase1_color_change_thresh, pattern_details, render_configs
                    )
                    update_neopixel_strips(current_color)

                # --- Matrix Display Update ---
                render_display(effective_progress_for_pattern, derived_target_pct_original,
                               TOTAL_DISPLAY_COLUMNS, pattern_details, render_configs)

            except OSError as e:
                # Catch hardware errors during the rendering process and print a warning
                print(f"I2C Error during display update: {e}. Check connections.")
            
            time.sleep(0.05) # Main loop delay

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user (Ctrl+C).")
    except Exception as e:
        print(f"An unexpected error occurred: {type(e).__name__}: {e}")
        sys.print_exception(e)
    finally:
        print("Performing final cleanup...")
        if pixels1 and pixels2:
            print("Turning off NeoPixel strips.")
            update_neopixel_strips("off")
        if vl53l1x_sensor_global is not None:
            try:
                vl53l1x_sensor_global.stop_ranging()
                print("VL53L1X ToF sensor ranging stopped.")
            except Exception as e_stop_tof:
                print(f"Error stopping ToF sensor: {e_stop_tof}")
        if matrices:
            try:
                print("Turning off blink and clearing displays...")
                for matrix_obj in matrices:
                    try:
                        matrix_obj.blink_rate = HT16K33_BLINK_OFF
                        matrix_obj.fill(MATRIX_COLOR_MAP["off"])
                        matrix_obj.show()
                    except Exception: pass
            except Exception as e_final_clear:
                print(f"Error during final display clear: {e_final_clear}")
        
        if i2c_matrices_bus and hasattr(i2c_matrices_bus, 'deinit'):
            try:
                i2c_matrices_bus.deinit()
                print("Matrices I2C deinitialized.")
            except Exception as e_deinit_matrices:
                print(f"Error deinitializing matrices I2C: {e_deinit_matrices}")
        
        print("Program ended.")

if __name__ == "__main__":
    main()
