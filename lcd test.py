import time
import board
import busio

from lcd.lcd import LCD
from lcd.i2c_pcf8574_interface import I2CPCF8574Interface
from lcd.lcd import CursorMode

def print_slowly(lcd, text, delay=0.1):
    for char in text:
        lcd.print(char)
        time.sleep(delay)

# Setup I2C and LCD
i2c = busio.I2C(board.GP19, board.GP18)
lcd = LCD(I2CPCF8574Interface(i2c, 0x27), num_rows=2, num_cols=16)

# Print a short message normally
lcd.print("abc ")

# Print the longer message slowly
print_slowly(lcd, "This is quite long and will wrap onto the next line automatically.")

time.sleep(2)  # Wait so you can read the message

lcd.clear()

# Set cursor to second line, fifth column (0-based)
lcd.set_cursor_pos(1, 4)
lcd.print("Here I am")


