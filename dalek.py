# Need floating point division of integers
from __future__ import division
import time
import board
import busio
import adafruit_mcp4728
import gpiozero
import subprocess
import random
import RPi.GPIO as GPIO
from gpiozero import Device, PWMOutputDevice, OutputDevice
from gpiozero.pins.native import NativeFactory
from approxeng.input.selectbinder import ControllerResource
import cv2
from signal import pause

# Set the default pin factory to native (BCM numbering, not physical pin number)
#Device.pin_factory = NativeFactory()

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

def setMotorSpeed(motor_pwm, motor_dir, speed):
    if speed >= 0:
        motor_dir.on()
        motor_pwm.value = speed
        print("forwards: " + str(speed))
    else:
        motor_dir.off()
        motor_pwm.value = -speed
        print("backwards: " + str(speed))

# try:
#     cam = cv2.VideoCapture(0)
# except:
#     face_classifier = cv2.CascadeClassifier(
#         cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
#     )

# Set pins for controller pad
pins = {}

for (pin) in [4,5,6,7,8,9,10,11,14,15,16,17,18,19,20]: #range(4,21):
    pins[pin] = gpiozero.LED(pin);
    #pins[pin].on(); Only if using low level relay trigger

buttonMappings = {
    "square": 4,
    "triangle": 5,
    "circle": 6,
    "cross": 7,
    "l1": 8,
    "l2": 9,
    "r1": 10,
    "r2": 11,
    "ddown": 14, # skip 12 and 13 as these are pwm pins used elsewhere
    "dleft": 15,
    "dright": 16,
    "dup": 17,
    "start": 18,
    # "select": 19,
    "home": 20
}

i2c = busio.I2C(board.SCL, board.SDA)
mcp4728 =  adafruit_mcp4728.MCP4728(i2c)

# Set up mcp4728 and use internal voltage reference
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

# Set PWM and Dir pins for head and eye via Cytron MOD10A Rev2
motor1_pwm = gpiozero.PWMOutputDevice(12)  # GPIO pin for motor 1 PWM0
motor1_dir = gpiozero.OutputDevice(25)     # GPIO pin for motor 1 direction
motor2_pwm = gpiozero.PWMOutputDevice(13)  # GPIO pin for motor 2 PWM1
motor2_dir = gpiozero.OutputDevice(26)     # GPIO pin for motor 2 direction




with ControllerResource(dead_zone=0.1, hot_zone=0) as joystick:
    while joystick.connected:
        presses = joystick.check_presses()
        for button in buttonMappings.keys():
            if joystick.presses[button]:
                print("Button pressed: " + button);
                pins[buttonMappings[button]].on(); # off(); if using low level trigger
            if joystick.releases[button]:
                print("Button released: " + button);
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

        # Get the x, y values of the left stick
        lx_axis = joystick['lx']
        ly_axis = joystick['ly']
        print(" lx: " + str(lx_axis) + "  ly: " + str(ly_axis))
        mcp4728.channel_b.raw_value = mapStickToDacValue(-lx_axis) # -ve because chair reversed
        mcp4728.channel_a.raw_value = mapStickToDacValue(ly_axis)


        # TODO - implement feedback from limit switches to stop motors
        
        # Right stick controls head left right, eye up down, but currently only in a binary sense
        rx_axis = joystick['rx']
        ry_axis = joystick['ry']
        print("rx: " + str(rx_axis) + " ry: " + str(ry_axis))

        # Control motor speed and direction based on joystick input
        setMotorSpeed(motor1_pwm, motor1_dir, rx_axis)
        setMotorSpeed(motor2_pwm, motor2_dir, ry_axis)

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

        time.sleep(0.1) # slow loop for debugging

        # if (cam):
        #     ret, image = cam.read()
        #     gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #     face = face_classifier.detectMultiScale(
        #         gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
        #     )

        #     for (x, y, w, h) in face:
        #         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 4)

        #     #save image
        #     # cv2.imwrite("face.jpg", image)
        #     # exit()

        
