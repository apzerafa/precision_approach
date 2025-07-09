#imports for settings
import os

#imports for light sensor
import analogio

#imports for button pushes
import keypad

#imports for distancing
import time
import board
import adafruit_vl53l1x
import busio
import digitalio
import storage

#imports for LCD
from lcd.lcd import LCD
from lcd.i2c_pcf8574_interface import I2CPCF8574Interface
from lcd.lcd import CursorMode


class Sensors:
    def __init__(self, i2c_bus, button=None, light_sensor=None, address=0x27):
        
        self.sensor = adafruit_vl53l1x.VL53L1X(i2c_bus)
        
        if light_sensor:
            self.light_sensor = analogio.AnalogIn(light_sensor)
            self.last_light_level = None
            self.light_rate_of_change = None
            
        self.button = keypad.Keys((button,), value_when_pressed=False,pull=True)

        
    def set_distance_mode(self, distance_mode=2):
        self.sensor.distance_mode = distance_mode
        
    def set_timing_budget(self, timing_budget=100):
        self.sensor.timing_budget = timing_budget
        
    def start_ranging(self):
        self.sensor.start_ranging()
    
    def stop_ranging(self):
        self.sensor.stop_ranging()
        
    def get_distance(self):
        if self.sensor.data_ready:
            distance = self.sensor.distance
            self.sensor.clear_interrupt()
            return distance
        
    def get_light_reading(self):
        light_reading = self.light_sensor.value
        return light_reading

class DisplayManager:
    def __init__(self, backlight=None, red_led=None, yellow_led=None, green_led=None, array_i2c_bus=None, array_i2c_address = None, screen_i2c_bus=None, screen_i2c_address=0x27,):
        
        self.backlight = None
        self.red_led = None
        self.yellow_led = None
        self.green_led = None
        
        if backlight:
            self.backlight = digitalio.DigitalInOut(backlight)
            self.backlight.direction = digitalio.Directiond.OUTPUT
        
        if red_led:
            self.red_led = digitalio.DigitalInOut(red_led)
            self.red_led.direction = digitalio.Direction.OUTPUT
            #set self.red_led.value = False to turn off
        
        if yellow_led:
            self.yellow_led = digitalio.DigitalInOut(yellow_led)
            self.yellow_led.direction = digitalio.Direction.OUTPUT
            
        if green_led:
            self.green_led = digitalio.DigitalInOut(green_led)
            self.green_led.direction = digitalio.Direction.OUTPUT
        
        self.all_lights = [light for light in [self.backlight, self.red_led, self.yellow_led, self.green_led] if light is not None]            
    
    
    def all_lights_off(self):        
        for light in self.all_lights:
            light.value = False

    def all_lights_on(self):
        for light in self.all_lights:
            light.value = True
            
    def led_on(self, led):
        led.value = True

    def led_off(self, led):
        led.value = False
        
        
class Settings:
    FILE_PATH = "/settings.txt"
    TEMP_FILE_PATH = "/settings.tmp.txt"
    
    def save_parked_distance(self, distance_to_save):
        if distance_to_save is None:
            print("Error: Cannot save None distance.")
            return None
        
        #ensure filesystem is writable, it is not when mounted as usb.
        
        try:
            storage.remount("/", readonly=False)
            
            #Write to temp file
            
            with open(Settings.TEMP_FILE_PATH, "w") as f:
                f.write(str(distance_to_save))
                
            #Rename temp file to target file
            
            os.rename(Settings.TEMP_FILE_PATH, Settings.FILE_PATH)
            
            
            print(f'Settings saved: {distance_to_save}')
            return distance_to_save
        
        except (OSError, ValueError) as e:
            print(f' Error saving settigns: {e}')
            
            #if an error happened cleam up the temp file
            
            try:
                os.remove(Settings.TEMP_FILE_PATH)
                
            except OSError:
                pass #ignore if the temp file doesn't exist
            
            return None
        finally:
            #remount as readonly every time
            
            storage.remount("/", readonly=True)
    
    def load_parked_distance(self):
        try:
            with open(Settings.FILE_PATH, "r") as f:
                data = f.read().strip()
                parked_distance = float(data)
                print(f'Loaded parked distance: {parked_distance}')
                return parked_distance
        except (OSError, ValueError) as e:
            print(f' Error loading data:{e}')
            return None 
        
#     def load_parked_distance(self):

class Precision_Approach:
    def __init__(self, sensor_obj, display_obj, settings_objc, tolerance=10):
        
        self.sensor = sensor_obj
        self.display = display_obj
        self.settings = sensor_obj
        
        self.tolerance = tolerance
        
        self.parked_distance = self.settings.load_parked_distance()
        
                
        
        
i2c_sensor_bus = busio.I2C(board.GP17, board.GP16)

sensor = Sensors(i2c_sensor_bus, board.GP12, board.A2,)

sensor.set_timing_budget(200)
sensor.set_distance_mode(2)

sensor.start_ranging()

display = DisplayManager(red_led=board.GP13, yellow_led=board.GP14, green_led=board.GP15)




# while True:
#     distance = sensor.get_distance()
#     light_reading = sensor.get_light_reading()
#     if distance:
#         display.all_lights_on()
#         print(distance, light_reading, sensor.last_light_level)
#         time.sleep(.1)
#         display.all_lights_off()
