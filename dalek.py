# Need floating point division of integers

from __future__ import division
import RPi.GPIO as GPIO
import time
import board
import busio
#import adafruit_mcp4728 - DAC used for old joystick
from GP8XXX_IIC import GP8413
import gpiozero
import subprocess
import random
from gpiozero import Device, PWMOutputDevice, OutputDevice, Button
from gpiozero.pins.native import NativeFactory
from approxeng.input.selectbinder import ControllerResource
from signal import pause
from webcam import Webcam
import asyncio

#import logging
from audio import init_pygame, monitor_audio_output
from ears import Ears
from motor import Motor

#time.sleep(5)
#webcam = Webcam()
try:
    webcam = Webcam()
    webcam_available = True
except Exception as e:
    print(f"Webcam initialization failed: {e}")
    webcam_available = False

# Set the default pin factory to native (BCM numbering, not physical pin number)
#Device.pin_factory = NativeFactory() # seems not to work

def mapStickToDacValue(stickValue):
    # stickValue should be between -1 and 1
    # That should map on to 1.11 to 3.89 volts
    # 0 volts is 0, 4.096 is 4096

    #Add 1 to stickvalue. range is now 0 to 2
    stickValue += 1

    #Multiply by 2048. range is now 0 to 4096
    stickValue *= 2048

    #How much through the range are we?
    stickValue = stickValue / 4096
    if (stickValue == 0.5):
        print('wiggle')
        v = int((1.11 + stickValue * (3.89-1.11)) * 1000)
        # print(v)
        # time.sleep(5)
        return v
    else:
        return int((1.11 + stickValue * (3.89-1.11)) * 1000)


def mapStickToGP8413Value(stickValue):
    # stickValue should be between -1 and 1
    # That should map on to 4.75 to 6.25 volts
    # 0  is stick central, which should equate to 5.5 volts
    base_voltage = 5.8
    stick_scaling = 1.1 # multiplier for stick value -
    # do not exceed 1.1 to avoid over-volt on joystick pcb

    #Calc voltage to 3 DP
    GP8413_value = round(base_voltage + (stickValue * stick_scaling), 3)

    #print("stickValue: " + str(stickValue) + " GP8413_value: " + str(GP8413_value))
    return GP8413_value


# Set pins for controller pad
pins = {}

for (pin) in [4,5,6,7,8,9,10,11,14,15,16,19,20]: #range(4,21):
    pins[pin] = gpiozero.LED(pin)#;
    #pins[pin].on(); Only if using low level relay trigger

# Set up input pins for buttons with external pull-up resistors (hence pull_up=False)
button17 = Button(17, pull_up=False)
button18 = Button(18, pull_up=False)
button22 = Button(22, pull_up=False)
button23 = Button(23, pull_up=False)
button24 = Button(24, pull_up=False) # Gin dispense button on gun

buttonMappings = {
    "square": 4,
    "triangle": 10,
    #"circle": 7, # used to pump gin
    "cross": 6,
    "l1": 8,
    "l2": 9,
    "r1": 5,  # water pistol
    #"r2": 11, # ears
    "ddown": 14, # skip 12 and 13 as these are pwm pins used elsewhere
    "dleft": 15,
    "dright": 16,
    #"dup": 17,
    #"start": 18,
    # "select": 19, # this is 'select' on pihut controller - to toggle joustmania
    # "home": 20 # this is 'analog' on pihut controller - to toggle disco mode
}

i2c = busio.I2C(board.SCL, board.SDA)

# Set up the GP8413 DAC
GP8413 = GP8413(i2c_addr=0x59)

while GP8413.begin():
    print("init error")
    time.sleep(1)

#Set range to 10V
#GP8413.set_dac_outrange(GP8413.OUTPUT_RANGE_10V)


# Old DAC for original joystick emulation
# mcp4728 =  adafruit_mcp4728.MCP4728(i2c)

# # Set up mcp4728 and use internal voltage reference
# mcp4728.channel_a.vref = adafruit_mcp4728.Vref.INTERNAL
# mcp4728.channel_b.vref = adafruit_mcp4728.Vref.INTERNAL
# mcp4728.channel_c.vref = adafruit_mcp4728.Vref.INTERNAL
# mcp4728.channel_d.vref = adafruit_mcp4728.Vref.INTERNAL

# mcp4728.channel_a.gain = 2
# mcp4728.channel_b.gain = 2
# mcp4728.channel_c.gain = 2
# mcp4728.channel_d.gain = 2

# mcp4728.channel_a.value = 0
# mcp4728.channel_b.value = 0
# mcp4728.channel_c.value = 0
# mcp4728.channel_d.value = 0

# mcp4728.save_settings()

# Set PWM and Dir pins for head and eye via Cytron MOD10A Rev2
head_motor = Motor(
    gpiozero.PWMOutputDevice(12),  # GPIO pin for motor 1 PWM0
    gpiozero.OutputDevice(25),  # GPIO pin for motor 1 direction
)
eye_stalk_motor = Motor(
    gpiozero.PWMOutputDevice(13),  # GPIO pin for motor 2 PWM1
    gpiozero.OutputDevice(26),  # GPIO pin for motor 2 direction
)


async def core():
    pygame = await init_pygame()

    # Flags to track previous button states so that only rumble once per time limit switch reached
    prev_button17_state = False
    prev_button18_state = False
    prev_button22_state = True
    prev_button23_state = True

    # Initialize variables for the time-delay gin dispensary
    button24_not_pressed_start = None  # Tracks when button24 was first not pressed
    gpio_pin_on = False  # Tracks the state of the GPIO pin dispensing gin
    inebriated = False  # Tracks whether the inebriated sound clip has been played

    with ControllerResource(dead_zone=0.1, hot_zone=0) as joystick:
        while joystick.connected:
            presses = joystick.check_presses()
            for button in buttonMappings.keys():
                if joystick.presses[button]:
                    print("Button pressed: " + button);
                    if buttonMappings[button] is not None:
                        pins[buttonMappings[button]].on(); # off(); if using low level trigger
                if joystick.releases[button]:
                    print("Button released: " + button);
                    if buttonMappings[button] is not None:
                        pins[buttonMappings[button]].off(); # on();

            if joystick.presses["select"]:
                print("Select pushed");
                #If joust not running, start it
                res = subprocess.run("sudo supervisorctl status joustmania".split(" "), capture_output=True)
                print(res.stdout);

                if ("STOPPED" in res.stdout.decode("utf-8")):
                    subprocess.run("sudo supervisorctl start joustmania".split(" "))
                else:
                    subprocess.run("sudo /home/davros/JoustMania/kill_processes.sh".split(" "))
                joystick.rumble(2000)  # Rumble for 2.0 seconds

            if joystick.presses["home"]:
                pins[20].toggle()
                joystick.rumble(4000)  # Rumble for 4.0 seconds

            if webcam_available:
                if joystick.presses["rs"]:
                    #Toggle face tracking
                    webcam.face_tracking = not webcam.face_tracking
                    print("Face tracking status: " + str(webcam.face_tracking))

            # Get the x, y values of the left stick
            lx_axis = joystick['lx']
            ly_axis = joystick['ly']
            #print(" lx: " + str(lx_axis) + "  ly: " + str(ly_axis))
            GP8413.set_dac_out_voltage(mapStickToGP8413Value(-lx_axis), channel=0)#l/r -ve because chair reversed
            GP8413.set_dac_out_voltage(mapStickToGP8413Value(ly_axis), channel=1) #f/r
            # Original DAC for original joystick emulation
            #mcp4728.channel_b.raw_value = mapStickToDacValue(-lx_axis) # -ve because chair reversed
            #mcp4728.channel_a.raw_value = mapStickToDacValue(ly_axis)


            # TODO - implement feedback from limit switches to stop motors

            # Right stick controls head left right, eye up down, but currently only in a binary sense
            rx_axis = joystick['rx']
            ry_axis = joystick['ry']

            #print("rx: " + str(rx_axis) + " ry: " + str(ry_axis))
            # If right stick being used, set face tracking to false
            if rx_axis != 0 or ry_axis != 0:
                webcam.face_tracking = False
                print("Face tracking disabled via stick override")

            if (webcam.face_tracking):
                try:
                    x, y = webcam.get_direction()
                    print("face tracking, setting eye position")
                    print("x: " + str(x) + " y: " + str(y))
                    ry_axis = -y * 1.5
                    rx_axis = x * 1.7

                    MIN_TRACKING_DRIVE = 0.6

                    if (abs(ry_axis) < 0.1):
                        ry_axis = 0
                    elif (abs(ry_axis) < MIN_TRACKING_DRIVE):
                        ry_axis = MIN_TRACKING_DRIVE * ry_axis / abs(ry_axis)

                    if (abs(rx_axis) < 0.1):
                        rx_axis = 0
                    elif (abs(rx_axis) < MIN_TRACKING_DRIVE):
                        rx_axis = MIN_TRACKING_DRIVE * rx_axis / abs(rx_axis)

                except Exception as e:
                        print(f"Error during face tracking: {e}")


            ry_axis /= 2
            rx_axis /= 1.5

            # Scale one direction to counteract preference of moter
            if (rx_axis < 0):
                rx_axis *= 1.25

            if rx_axis > 1:
                rx_axis = 1
            if rx_axis < -1:
                rx_axis = -1

        # Check button states before setting motor speed on head LR axis
        # switches are normally open so circuit is closed when pressed. Not ideal
        # as no failsafe in case of circuit failure, but limited by hardware options
        # for toggle switches
            if button17.is_pressed and button18.is_pressed:
                # Control motor speed and direction based on joystick input
                head_motor.set_velocity(-rx_axis)
                #print('head limit not reached')
            elif not button17.is_pressed:
                #print('CCW head limit reached')
                # Prevent motor rotation anticlockwise when button 17 is pressed
                if rx_axis < 0:
                    head_motor.set_velocity(0)  # don't allow ccw
                    if not prev_button17_state:
                        joystick.rumble(500)  # Rumble for 0.5 seconds
                        prev_button17_state = True
                else:
                    head_motor.set_velocity(-rx_axis)  # allow cw

            elif not button18.is_pressed:
                # Prevent motor rotation clockwise when button 18 is pressed
                #print('CW head limit reached')
                if rx_axis > 0:
                    head_motor.set_velocity(0)
                    if not prev_button18_state:
                        joystick.rumble(500)  # Rumble for 0.5 seconds
                        prev_button18_state = True
                else:
                    head_motor.set_velocity(-rx_axis)

            # Reset flags if buttons are not pressed
            if  button17.is_pressed:
                prev_button17_state = False
            if  button18.is_pressed:
                prev_button18_state = False

        # Check button states before setting motor speed on eye up/down axis
        # These are the opposite of the head rotation as switches wired normally closed
        # as failsafe in case of circuit failure
            if not button22.is_pressed and not button23.is_pressed:
                # Control motor speed and direction based on joystick input
                eye_stalk_motor.set_velocity(-ry_axis)

            elif button22.is_pressed:
                # Prevent eye lifting when 22 is (not) pressed and circuit is open
                if ry_axis > 0:
                    eye_stalk_motor.set_velocity(0)  # Don't allow lift
                    if prev_button22_state:
                        joystick.rumble(500)  # Rumble for 0.5 seconds
                        prev_button22_state = False
                else:
                    eye_stalk_motor.set_velocity(-ry_axis)  # allow cw

            elif button23.is_pressed:
                # Prevent lowering when button 23 is (not) pressed and circuit is open
                if ry_axis < 0:
                    eye_stalk_motor.set_velocity(0)
                    if prev_button23_state:
                        joystick.rumble(500)  # Rumble for 0.5 seconds
                        prev_button23_state = False
                else:
                    eye_stalk_motor.set_velocity(-ry_axis)

            # Reset flags if buttons are pressed
            if not button22.is_pressed:
                prev_button22_state = True
            if not button23.is_pressed:
                prev_button23_state = True


            # Legacy as initially used for head controls before Cytron MOD10A Rev2
            # implemented for speed and direction
            # This pin should control two relays for each axis, and will control direction
            # mcp4728.channel_d.raw_value = max(mapStickToDacValue(x_axis)-2000, 0)
            # mcp4728.channel_c.raw_value = max(mapStickToDacValue(y_axis)-2000, 0)

            # These pins control power.
            # The limit is here such that the direction indication above is on, if it's coming on,
            # Before power is supplied.
            # if (abs(joystick['rx']) > 0.8):
            #     pins[19].on() # off()  if using low level trigger
            # else:
            #     pins[19].off() # on()

            # if (abs(joystick['ry']) > 0.8):
            #     pins[20].on() # off()
            # else:
            #     pins[20].off() # on()

            # time.sleep(0.3) # slow loop for debugging

            # Check if button24 is not pressed
            if not button24.is_pressed: # or (joystick.presses["cross"] and joystick.presses["triangle"]):
                if not inebriated:
                    print("INEBRIATE")
                    pygame.mixer.music.load('/home/davros/inebriate.mp3')
                    pygame.mixer.music.play()
                    inebriated = True  # Set the flag to prevent replaying

                if button24_not_pressed_start is None:
                    # Start the timer when button24 is first detected as not pressed
                    button24_not_pressed_start = time.time()
                elif time.time() - button24_not_pressed_start >= 3 and not gpio_pin_on:
                    # If button24 has been not pressed for 3 seconds, turn the GPIO pin on
                    print("Gin button pressed for 3 seconds, turning pump on")
                    pins[7].on()
                    gpio_pin_on = True  # Update the state of the GPIO pin
            else:
                # Reset the timer and turn the GPIO pin off when button24 is pressed
                if gpio_pin_on:
                    print("Gin button released, turning pump off")
                    pins[7].off()  # Turn the GPIO pin off
                    gpio_pin_on = False  # Update the state of the GPIO pin
                button24_not_pressed_start = None  # Reset the timer
                inebriated = False  # Reset the inebriated flag to allow replay on the next unpress

            await asyncio.sleep(0)


ears = Ears(pins[11])


async def main():
    tasks = [core(), monitor_audio_output(ears)]
    if webcam_available:
        tasks.append(webcam.start())
    await asyncio.gather(*tasks)


asyncio.run(main())

