import board
import keypad
import time

# Create a Key object on GP15 with internal pull-up
keys = keypad.Keys((board.GP12,), value_when_pressed=False, pull=True)

while True:
    event = keys.events.get()
    if event:
        if event.pressed:
            print("Button pressed!")
        elif event.released:
            print("Button released!")
    time.sleep(0.01)
