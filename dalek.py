# Need floating point division of integers
from __future__ import division
import time
import board
import busio
import adafruit_mcp4728
import gpiozero

from signal import pause

pins = {}

for (pin) in range(4,18):
    pins[pin] = gpiozero.LED(pin);
    pins[pin].on();

from approxeng.input.selectbinder import ControllerResource

buttonMappings = {
    "square": 4,
    "triangle": 5,
    "circle": 6,
    "cross": 7,
    "l1": 8,
    "l2": 9,
    "r1": 10,
    "r2": 11,
    "ddown": 12,
    "dleft": 13,
    "dright": 14,
    "dup": 15,
    "start": 16,
    "select": 17,
    "home": 18
}

i2c = busio.I2C(board.SCL, board.SDA)
mcp4728 =  adafruit_mcp4728.MCP4728(i2c)

# Use internal voltage reference
mcp4728.channel_a.vref = adafruit_mcp4728.Vref.INTERNAL
mcp4728.channel_b.vref = adafruit_mcp4728.Vref.INTERNAL
mcp4728.channel_c.vref = adafruit_mcp4728.Vref.INTERNAL
mcp4728.channel_d.vref = adafruit_mcp4728.Vref.INTERNAL

mcp4728.channel_a.gain = 2
mcp4728.channel_b.gain = 2
mcp4728.channel_c.gain = 2
mcp4728.channel_d.gain = 2

mcp4728.channel_a.value = 0
mcp4728.channel_b.value = 0
mcp4728.channel_c.value = 0
mcp4728.channel_d.value = 0

mcp4728.save_settings()

def mapStickToDacValue(stickValue):
    # stickValue should be between -1 and 1
    # That should map on to 1.11 to 3.89 volts
    # 0 volts is 0, 4.096 is 4096

    #Add 1 to stickvalue. range is now 0 to 2
    stickValue += 1

    #Multiply by 2048. range is now 0 to 4096
    stickValue *= 2048

    #How mucht through the range are we?
    stickValue = stickValue / 4096

    return int((1.11 + stickValue * (3.89-1.11)) * 1000)

with ControllerResource(dead_zone=0.1, hot_zone=0) as joystick:
    while joystick.connected:
        presses = joystick.check_presses()
        for button in buttonMappings.keys():
            if joystick.presses[button]:
                print("Button pressed: " + button);
                pins[buttonMappings[button]].off();
            if joystick.releases[button]:
                print("Button released: " + button);
                pins[buttonMappings[button]].on();
        
        # Get the x, y values of the left stick
        x_axis = joystick['lx']
        y_axis = joystick['ly']
        print("x: " + str(x_axis) + " y: " + str(y_axis))
        mcp4728.channel_b.raw_value = mapStickToDacValue(x_axis)
        mcp4728.channel_a.raw_value = mapStickToDacValue(y_axis)
        x_axis = joystick['rx']
        y_axis = joystick['ry']
        mcp4728.channel_d.raw_value = mapStickToDacValue(x_axis)
        mcp4728.channel_c.raw_value = mapStickToDacValue(y_axis)
        
