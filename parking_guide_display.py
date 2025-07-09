import time
import sys
from adafruit_ht16k33.matrix import Matrix8x8x2
import neopixel
import digitalio

class ParkingGuideDisplay:
    """
    Manages the LED Matrix and NeoPixel displays for the parking assistant.
    This class encapsulates all drawing logic, calculations, and hardware control
    for the visual feedback system.
    """

    def __init__(self, matrix_i2c_bus, neopixel_pin_1, neopixel_pin_2, configs):
        """
        Initializes the display hardware and configurations.
        :param matrix_i2c_bus: The configured busio.I2C object for the matrices.
        :param neopixel_pin_1: The board pin for the first NeoPixel strip.
        :param neopixel_pin_2: The board pin for the second NeoPixel strip.
        :param configs: A dictionary containing all necessary configuration constants.
        """
        self.config = configs
        self.matrices = []
        self.pixels1 = None
        self.pixels2 = None
        self.oe_enable = None
        self.total_display_columns = 0

        # Unpack error codes for internal use
        self.TOF_READING_ERROR = self.config.get('TOF_READING_ERROR', -1.0)

        # Color maps
        self._MATRIX_COLOR_MAP = {
            "off": Matrix8x8x2.LED_OFF, "red": Matrix8x8x2.LED_RED,
            "green": Matrix8x8x2.LED_GREEN, "yellow": Matrix8x8x2.LED_YELLOW
        }
        self._NEOPIXEL_COLOR_MAP = {
            "off": (0, 0, 0), "red": (255, 0, 0), "green": (0, 255, 0),
            "yellow": (255, 45, 0), "blue": (0, 0, 255)
        }
        
        # The standard 5x7 font that we confirmed works correctly.
        self._FONT = {
            '0': (" 111 ", "1   1", "1   1", "1   1", " 111 "),
            '1': ("  1  ", " 11  ", "  1  ", "  1  ", " 111 "),
            '2': (" 111 ", "1   1", "   1 ", "  1  ", "11111"),
            '3': ("1111 ", "    1", " 111 ", "    1", "1111 "),
            '4': ("1  1 ", "1  1 ", "11111", "   1 ", "   1 "),
            '5': ("11111", "1    ", "1111 ", "    1", " 111 "),
            '6': (" 111 ", "1    ", "1111 ", "1   1", " 111 "),
            '7': ("11111", "   1 ", "  1  ", " 1   ", " 1   "),
            '8': (" 111 ", "1   1", " 111 ", "1   1", " 111 "),
            '9': (" 111 ", "1   1", " 1111", "    1", " 111 "),
        }


        self._initialize_neopixels(neopixel_pin_1, neopixel_pin_2)
        self._initialize_matrices(matrix_i2c_bus)

    def update(self, current_distance_cm, target_distance_cm):
        """
        The main public method to update the entire display based on sensor readings.
        :param current_distance_cm: The current distance from the ToF sensor. Can be an error code.
        :param target_distance_cm: The desired 'parked' distance.
        """
        if current_distance_cm is None:
            current_distance_cm = self.TOF_READING_ERROR

        # --- Percentage Calculations ---
        upper_range = self.config.get('UPPER_RANGE_CM', 190.0)
        current_progress_percent = self._calculate_current_tof_percentage(current_distance_cm, upper_range)
        derived_target_pct = self._calculate_derived_target_percentage(target_distance_cm, upper_range)

        tof_is_error = current_progress_percent <= self.TOF_READING_ERROR
        effective_progress = 0.0 if tof_is_error else current_progress_percent

        # --- Determine Display State ---
        is_phase2_active = effective_progress >= self.config.get('CONFIG_ZOOM_TRANSITION_THRESHOLD_PERCENT', 75.0)

        pattern_details = self._get_pattern_display_details(
            effective_progress, is_phase2_active, derived_target_pct
        )

        # --- Update Hardware ---
        self._update_neopixel_strips(effective_progress, is_phase2_active, tof_is_error, pattern_details, derived_target_pct)
        if self.total_display_columns > 0:
            self._render_matrix_display(effective_progress, derived_target_pct, pattern_details)

    def clear(self):
        """Turns off all LEDs on both matrices and NeoPixel strips."""
        if self.matrices:
            for matrix_obj in self.matrices:
                try:
                    matrix_obj.fill(self._MATRIX_COLOR_MAP["off"])
                    matrix_obj.show()
                except OSError:
                    pass # Ignore I2C errors on clear
        if self.pixels1 and self.pixels2:
            self._set_neopixel_color("off")

    def show_error_state(self):
        """Sets the display to a predefined error state (e.g., blue NeoPixels)."""
        self.clear()
        self._set_neopixel_color("blue")

    def set_neopixels(self, color_name):
        """Directly sets the neopixel color. Publicly accessible."""
        self._set_neopixel_color(color_name)

    def show_countdown(self, number):
        """
        Displays a number on the first matrix for a countdown.
        :param number: The integer to display.
        """
        self._draw_digit(number, "red")

    def show_score(self, number):
        """
        Displays a score number on the first matrix.
        :param number: The integer score to display.
        """
        self._draw_digit(number, "green")

    def _draw_digit(self, number, color_name):
        """
        Internal helper to draw a single digit to the first matrix.
        :param number: The integer to display.
        :param color_name: The string name of the color to use.
        """
        if not self.matrices:
            return
        
        for m in self.matrices:
            m.fill(0)

        matrix = self.matrices[0]
        char_to_draw = str(number)
        
        if char_to_draw in self._FONT:
            font_char = self._FONT[char_to_draw]
            color_value = self._MATRIX_COLOR_MAP.get(color_name, self._MATRIX_COLOR_MAP["off"])
            
            for y, row_str in enumerate(font_char):
                for x, pixel in enumerate(row_str):
                    if pixel == '1':
                        # This is the confirmed working transformation from the test program.
                        matrix[7 - y, 7 - (x + 1)] = color_value
        
        for m in self.matrices:
            try:
                m.show()
            except OSError:
                pass


    # --- Private Initialization Methods ---

    def _initialize_neopixels(self, pin1, pin2):
        """Initializes the NeoPixel strips."""
        if self.config.get('NEOPIXEL_USE_OE_PIN', False):
            try:
                self.oe_enable = digitalio.DigitalInOut(self.config['NEOPIXEL_OE_PIN'])
                self.oe_enable.direction = digitalio.Direction.OUTPUT
                self.oe_enable.value = True
                print("NeoPixel level shifter enabled.")
            except Exception as e:
                print(f"Could not initialize NeoPixel OE pin: {e}")

        try:
            num_leds = self.config.get('NEOPIXEL_NUM_LEDS', 11)
            order = self.config.get('NEOPIXEL_PIXEL_ORDER', 'GRB')
            self.pixels1 = neopixel.NeoPixel(pin1, num_leds, pixel_order=order, auto_write=False)
            self.pixels2 = neopixel.NeoPixel(pin2, num_leds, pixel_order=order, auto_write=False)
            print("NeoPixel strips initialized.")
        except Exception as e:
            print(f"Error initializing NeoPixel strips: {e}")
            self.pixels1, self.pixels2 = None, None

    def _initialize_matrices(self, i2c_bus):
        """Scans for and initializes the LED matrix displays."""
        if not i2c_bus:
            print("Warning: No I2C bus provided for matrices.")
            return

        found_addresses = []
        bus_locked = False # Flag to track if the lock was acquired
        try:
            while not i2c_bus.try_lock():
                time.sleep(0.01)
            bus_locked = True # Lock acquired
            found_addresses = i2c_bus.scan()
        except Exception as e:
            print(f"Error during matrix I2C scan: {e}")
        finally:
            if bus_locked: # Only unlock if the lock was successfully acquired
                i2c_bus.unlock()

        temp_matrices = []
        possible_addrs = self.config.get('POSSIBLE_ADDRESSES', [])
        max_matrices = self.config.get('MAX_MATRICES', 4)
        brightness = self.config.get('BRIGHTNESS_LEVEL', 1.0)

        for addr in possible_addrs:
            if addr in found_addresses and len(temp_matrices) < max_matrices:
                try:
                    matrix_instance = Matrix8x8x2(i2c_bus, address=addr)
                    matrix_instance.auto_write = False
                    matrix_instance.brightness = brightness
                    matrix_instance.fill(self._MATRIX_COLOR_MAP["off"])
                    matrix_instance.show()
                    temp_matrices.append(matrix_instance)
                except Exception as e:
                    print(f"Failed to initialize matrix at {hex(addr)}: {e}")

        self.matrices = temp_matrices
        self.total_display_columns = len(self.matrices) * self.config.get('MATRIX_COLUMNS_PER_UNIT', 8)
        print(f"Initialized {len(self.matrices)} matrices, total {self.total_display_columns} display columns.")


    # --- Private Calculation Helpers (Directly from original script) ---

    def _calculate_current_tof_percentage(self, current_tof_reading_cm, upper_range_cm_config):
        if current_tof_reading_cm < 0: return self.TOF_READING_ERROR
        if current_tof_reading_cm > upper_range_cm_config: return self.TOF_READING_ERROR
        if upper_range_cm_config == 0: return 0.0 if current_tof_reading_cm > 0 else 100.0
        raw_percentage = ((upper_range_cm_config - current_tof_reading_cm) / upper_range_cm_config) * 100.0
        return max(0.0, min(100.0, raw_percentage))

    def _calculate_derived_target_percentage(self, parked_distance_cm_config, upper_range_cm_config):
        if parked_distance_cm_config is None or not (0 <= parked_distance_cm_config <= upper_range_cm_config): return 50.0
        if upper_range_cm_config == 0: return 0.0
        raw_percentage = ((upper_range_cm_config - parked_distance_cm_config) / upper_range_cm_config) * 100.0
        return max(0.0, min(100.0, raw_percentage))

    def _get_pattern_display_details(self, current_slider_original_pct, is_phase2_active, derived_target_pct):
        # This function remains complex, but is now fully encapsulated.
        cfg = self.config
        total_cols = self.total_display_columns
        pattern = {
            'left_display_col_start': -1, 'left_display_col_end': -1, 'left_color': cfg['CONFIG_LEFT_STATIC_COLOR'],
            'center_display_col_start': -1, 'center_display_col_end': -1, 'center_color': cfg['CONFIG_CENTER_STATIC_COLOR'],
            'right_display_col_start': -1, 'right_display_col_end': -1, 'right_color': cfg['CONFIG_RIGHT_STATIC_COLOR'],
            'left_original_pct': -1.0, 'center_original_pct': derived_target_pct, 'right_original_pct': -1.0,
            'viewport_original_start_pct': 0.0, 'original_pct_per_display_col': 0.0, 'phase2_view_coverage_pct': 0.0
        }
        if total_cols == 0: return pattern

        if is_phase2_active:
            # Phase 2 "Zoomed" calculations
            effective_cols_per_pct = max(cfg['PHASE2_MIN_COLS_PER_ONE_PERCENT'], cfg['CONFIG_PHASE2_COLS_PER_ONE_PERCENT'])
            pct_per_col = 1.0 / effective_cols_per_pct
            view_coverage_pct = total_cols * pct_per_col
            viewport_start_pct = max(0.0, derived_target_pct - (view_coverage_pct / 2.0))
            
            pattern.update({
                'viewport_original_start_pct': viewport_start_pct,
                'original_pct_per_display_col': pct_per_col,
                'phase2_view_coverage_pct': view_coverage_pct
            })

            c_start, c_end = self._get_phase2_static_marker_details(derived_target_pct, derived_target_pct, effective_cols_per_pct, viewport_start_pct, view_coverage_pct, False, False, None)
            pattern['center_display_col_start'], pattern['center_display_col_end'] = c_start, c_end
            center_marker_details = {'start': c_start, 'end': c_end}

            # DYNAMIC MARKER CALCULATION
            offset_pct = cfg.get('CONFIG_PHASE2_STATIC_MARKER_OFFSET_PCT', 3.0)
            left_pct = derived_target_pct - offset_pct
            right_pct = derived_target_pct + offset_pct
            
            pattern['left_original_pct'] = left_pct
            l_start, l_end = self._get_phase2_static_marker_details(left_pct, derived_target_pct, effective_cols_per_pct, viewport_start_pct, view_coverage_pct, True, False, center_marker_details)
            pattern['left_display_col_start'], pattern['left_display_col_end'] = l_start, l_end
            
            pattern['right_original_pct'] = right_pct
            r_start, r_end = self._get_phase2_static_marker_details(right_pct, derived_target_pct, effective_cols_per_pct, viewport_start_pct, view_coverage_pct, False, True, center_marker_details)
            pattern['right_display_col_start'], pattern['right_display_col_end'] = r_start, r_end
        else:
            # Phase 1 "Overview" calculations
            center_col = min(total_cols - 1, max(0, int((derived_target_pct / 100.0) * total_cols)))
            pattern['center_display_col_start'] = pattern['center_display_col_end'] = center_col
            
            padding = cfg['CONFIG_PHASE1_PADDING_COLS']
            left_col = center_col - padding - 1
            if left_col >= 0:
                pattern['left_display_col_start'] = pattern['left_display_col_end'] = left_col
                pattern['left_original_pct'] = (float(left_col) / total_cols) * 100.0

            right_col = center_col + padding + 1
            if right_col < total_cols:
                pattern['right_display_col_start'] = pattern['right_display_col_end'] = right_col
                pattern['right_original_pct'] = (float(right_col) / total_cols) * 100.0
            
            if total_cols > 0:
                pattern['original_pct_per_display_col'] = 100.0 / total_cols
        return pattern

    def _get_phase2_static_marker_details(self, static_marker_pct, target_pct, cols_per_pct, viewport_start_pct, view_coverage_pct, is_left, is_right, center_details):
        # This helper remains mostly unchanged, using instance properties for calculations.
        total_cols = self.total_display_columns
        display_col_start, display_col_end = -1, -1
        if total_cols == 0 or cols_per_pct <= 0: return -1, -1

        pct_per_col = 1.0 / cols_per_pct
        num_cols_for_marker = max(1, int(cols_per_pct * 1.0))
        
        tolerance = self.config.get('FLOAT_COMPARISON_TOLERANCE', 0.0001)

        if viewport_start_pct - tolerance <= static_marker_pct < viewport_start_pct + view_coverage_pct + tolerance:
            offset_pct = static_marker_pct - viewport_start_pct
            if is_left:
                ideal_end = offset_pct / pct_per_col
                display_col_end = int(ideal_end)
                display_col_start = display_col_end - num_cols_for_marker + 1
            elif is_right:
                ideal_start = offset_pct / pct_per_col
                display_col_start = int(ideal_start)
                display_col_end = display_col_start + num_cols_for_marker - 1
            else: # Center
                ideal_center = offset_pct / pct_per_col
                start_raw = ideal_center - (num_cols_for_marker / 2.0)
                display_col_start = int(start_raw + self.config.get('ROUNDING_OFFSET', 0.5))
                display_col_end = display_col_start + num_cols_for_marker - 1

            display_col_start = max(0, display_col_start)
            display_col_end = min(total_cols - 1, display_col_end)

            if display_col_start > display_col_end: return -1, -1
            if center_details and center_details['start'] != -1 and (is_left or is_right):
                overlap = (display_col_start <= center_details['end']) and (display_col_end >= center_details['start'])
                if overlap: return -1, -1
        return display_col_start, display_col_end

    # --- Private Rendering Methods ---

    def _render_matrix_display(self, current_slider_pct, derived_target_pct, pattern_details):
        """Internal method to draw the current state to the LED matrices."""
        total_cols = self.total_display_columns
        if total_cols == 0 or not pattern_details: return

        cfg = self.config
        is_phase2 = current_slider_pct >= cfg['CONFIG_ZOOM_TRANSITION_THRESHOLD_PERCENT']
        
        colors = ["off"] * total_cols
        
        p1_color_thresh_pct = self._calculate_phase1_slider_color_change_threshold(pattern_details['center_original_pct'], derived_target_pct)

        for idx in range(total_cols):
            col_start_pct = pattern_details['viewport_original_start_pct'] + (idx * pattern_details['original_pct_per_display_col'])
            
            is_filled = False
            if is_phase2:
                if current_slider_pct > pattern_details['viewport_original_start_pct'] and pattern_details['original_pct_per_display_col'] > 0:
                    cols_to_fill = int((current_slider_pct - pattern_details['viewport_original_start_pct']) / pattern_details['original_pct_per_display_col'])
                    if idx < cols_to_fill: is_filled = True
            elif col_start_pct < current_slider_pct:
                is_filled = True

            if is_filled:
                colors[idx] = self._get_slider_fill_color(current_slider_pct, col_start_pct, is_phase2, p1_color_thresh_pct, pattern_details)

        # Apply static markers over the slider bar
        markers = [
            ('left', pattern_details['left_display_col_start'], pattern_details['left_display_col_end'], pattern_details['left_color']),
            ('right', pattern_details['right_display_col_start'], pattern_details['right_display_col_end'], pattern_details['right_color']),
            ('center', pattern_details['center_display_col_start'], pattern_details['center_display_col_end'], pattern_details['center_color'])
        ]
        for name, start, end, color in markers:
            if start != -1:
                for i in range(start, end + 1):
                    if 0 <= i < total_cols:
                        if name == 'center' and is_phase2 and cfg['PHASE2_ALLOW_SLIDER_OVERWRITE_TARGET']:
                            if colors[i] == "off": colors[i] = color
                        else:
                            colors[i] = color

        # Apply precise indicator in Phase 2
        if is_phase2 and cfg['PHASE2_SHOW_PRECISE_INDICATOR']:
            if pattern_details['original_pct_per_display_col'] > 0:
                progress_into_vp = current_slider_pct - pattern_details['viewport_original_start_pct']
                indicator_idx = int(progress_into_vp / pattern_details['original_pct_per_display_col'])
                if 0 <= indicator_idx < total_cols:
                    colors[indicator_idx] = cfg['CONFIG_PRECISE_INDICATOR_COLOR']

        # Draw to hardware
        for i, color_name in enumerate(colors):
            self._set_single_column_color(i, color_name)
        
        for m in self.matrices:
            try:
                m.show()
            except OSError as e:
                print(f"I2C Error on matrix {hex(m.address)} during show(): {e}")

    def _update_neopixel_strips(self, current_slider_pct, is_phase2, is_error, pattern_details, derived_target_pct):
        """Internal method to update the NeoPixel strips based on the current state."""
        if is_error:
            self._set_neopixel_color("blue")
            return
            
        p1_color_thresh = self._calculate_phase1_slider_color_change_threshold(pattern_details['center_original_pct'], derived_target_pct)
        
        current_color = self._get_current_progress_color(current_slider_pct, is_phase2, p1_color_thresh, pattern_details)
        self._set_neopixel_color(current_color)

    def _set_single_column_color(self, global_col_idx, color_name):
        """Sets all LEDs in a globally indexed column to a specified color."""
        if not (0 <= global_col_idx < self.total_display_columns): return

        color_val = self._MATRIX_COLOR_MAP.get(color_name, self._MATRIX_COLOR_MAP["off"])
        matrix_idx = global_col_idx // self.config['MATRIX_COLUMNS_PER_UNIT']
        col_in_matrix = global_col_idx % self.config['MATRIX_COLUMNS_PER_UNIT']
        
        if 0 <= matrix_idx < len(self.matrices):
            matrix_obj = self.matrices[matrix_idx]
            for row in range(8):
                try:
                    matrix_obj[col_in_matrix, row] = color_val
                except IndexError:
                    pass # Should not happen with guards, but for safety

    def _set_neopixel_color(self, color_name):
        """Sets both NeoPixel strips to a specified color."""
        if not self.pixels1 or not self.pixels2: return
        
        rgb_color = self._NEOPIXEL_COLOR_MAP.get(color_name, self._NEOPIXEL_COLOR_MAP["off"])
        try:
            self.pixels1.fill(rgb_color)
            self.pixels2.fill(rgb_color)
            self.pixels1.show()
            self.pixels2.show()
        except Exception as e:
            print(f"Error updating NeoPixels: {e}")
            
    # --- Private Color Logic Helpers ---
    
    def _calculate_phase1_slider_color_change_threshold(self, center_marker_pct, derived_target_pct):
        change_point_pct = self.config['CONFIG_SLIDER_COLOR_CHANGE_POINT_TARGET_PCT']
        if center_marker_pct != -1 and not isinstance(change_point_pct, str):
            return max(0.0, (change_point_pct / 100.0) * center_marker_pct)
        return (50.0 / 100.0) * derived_target_pct

    def _get_current_progress_color(self, current_slider_pct, is_phase2, p1_color_thresh_pct, pattern_details):
        cfg = self.config
        if is_phase2:
            initial_color = cfg['CONFIG_SLIDER_END_COLOR_PHASE1'] if current_slider_pct >= p1_color_thresh_pct else cfg['CONFIG_SLIDER_START_COLOR']
            first_static_pct = pattern_details.get('left_original_pct', -1.0)
            if first_static_pct != -1.0 and current_slider_pct > first_static_pct:
                return cfg['CONFIG_SLIDER_COLOR_AFTER_1ST_STATIC_ZOOMED']
            return initial_color
        else:
            return cfg['CONFIG_SLIDER_END_COLOR_PHASE1'] if current_slider_pct > p1_color_thresh_pct else cfg['CONFIG_SLIDER_START_COLOR']

    def _get_slider_fill_color(self, current_slider_pct, col_start_pct, is_phase2, p1_color_thresh_pct, pattern_details):
        cfg = self.config
        if is_phase2:
            return self._get_current_progress_color(current_slider_pct, is_phase2, p1_color_thresh_pct, pattern_details)
        else:
            return cfg['CONFIG_SLIDER_END_COLOR_PHASE1'] if col_start_pct >= p1_color_thresh_pct else cfg['CONFIG_SLIDER_START_COLOR']
