# Need floating point division of integers
from __future__ import division
import asyncio
import time

import board
import busio
import gpiozero
from approxeng.input.selectbinder import ControllerResource
from GP8XXX_IIC import GP8413

from dalek.audio import init_pygame, monitor_audio_output
from dalek.ears import Ears
from dalek.motor import TravelLimitedMotor
from dalek.utils import throttle
from dalek.mock_joystick import MockJoystick
from dalek.lights import ArduinoLedController
from dalek.webcam_fdlite import Webcam

# Game imports
from dalek import snake
from dalek import invaders
from dalek import runner

## usage: PYTHONPATH=src ./dalek_venv_fdlite/bin/python -u -m dalek.dalek

DEBUG_MODE = False  # Toggle to False when running on controller rather than keyboard

try:
    webcam = Webcam()
    # Double check that the webcam object actually bound a hardware stream 
    if hasattr(webcam, 'cap') and webcam.cap is not None and webcam.cap.isOpened():
        webcam_available = True
    else:
        print("Webcam initialized but no active video stream found.")
        webcam_available = False
except Exception as e:
    print(f"Webcam initialization failed: {e}")
    webcam = None
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
        v = int((1.11 + stickValue * (3.89-1.11)) * 1000)
        return v
    else:
        return int((1.11 + stickValue * (3.89-1.11)) * 1000)

def mapStickToGP8413Value(stickValue):
    # stickValue should be between -1 and 1
    # That should map on to 4.75 to 6.25 volts
    # 0  is stick central, which should equate to 5.5 volts
    base_voltage = 5.8
    stick_scaling = 1.1 # multiplier for stick value
    # do not exceed 1.1 to avoid over-volt on joystick pcb
    #Calc voltage to 3 DP
    GP8413_value = round(base_voltage + (stickValue * stick_scaling), 3)
    #print("stickValue: " + str(stickValue) + " GP8413_value: " + str(GP8413_value))
    return GP8413_value

PIN_EAR = 11
## Other pins used: 
# pin 4 provides power for 24v LEDS for snake/Joust

# Set pins for controller pad
pins = {}
for pin in [4,5,6,7,8,9,10,11,14,15,16,19,20, 21]:
    pins[pin] = gpiozero.LED(pin)
    #pins[pin].on(); Only if using low level relay trigger

# Set up input pins for buttons with external pull-up resistors (hence pull_up=False)
# these are the limit switches for the head roatation and eye stalk up/down travel limits
button17 = gpiozero.Button(17, pull_up=False)
button18 = gpiozero.Button(18, pull_up=False)
button22 = gpiozero.Button(22, pull_up=False)
button23 = gpiozero.Button(23, pull_up=False)
gin_button = gpiozero.Button(24, pull_up=False) # Gin dispense button on gun

buttonMappings = {
    "triangle": 10,
    "cross": 6,
    "l1": 8,
    "r1": 5,  # water pistol
    "ddown": 14,  # skip 12 and 13 as these are pwm pins used elsewhere
    "dleft": 15,
    "dright": 16,
}   #"dup": 19,
    #"start": 21, # second relay - toggle the wheelchair controller on and off
    # "select": , # this is 'select' on pihut controller - to toggle joustmania
    # "home": 20 # this is 'analog' on pihut controller - to toggle disco mode

i2c = busio.I2C(board.SCL, board.SDA)


# Set up the GP8413 DAC
try:
    GP8413 = GP8413(i2c_addr=0x59)
except:
    print("GP8413 not found at address 0x59")
    print("Will mock joystick outputs - no movement possible")
    class MockGP8413:
        def begin(self): return False
        def set_dac_out_voltage(self, voltage, channel): pass
    GP8413 = MockGP8413()

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

head_motor = TravelLimitedMotor(
    pwm_pin=gpiozero.PWMOutputDevice(12),   # GPIO pin for motor 1 PWM0
    direction_pin=gpiozero.OutputDevice(25),  # GPIO pin for motor 1 direction
    velocity_scaling=(0.45, 0.35),
)
eye_stalk_motor = TravelLimitedMotor(
    pwm_pin=gpiozero.PWMOutputDevice(13),  # GPIO pin for motor 2 PWM1
    direction_pin=gpiozero.OutputDevice(26),  # GPIO pin for motor 2 direction,
    velocity_scaling=(0.65, 0.65),
)

@throttle
def rumble(duration_s: float, joystick):
    joystick.rumble(duration_s / 1000)

async def pump_gin(pygame):
    # Make sure they really want it.
    await asyncio.sleep(2)
    if not pygame.mixer.music.get_busy():
        pygame.mixer.music.play()
    pins[7].on()

async def core():
    # Instantiate arduino light module over its USB connection string
    lights = ArduinoLedController(vendor_id="1a86", product_id="7523")
    # Enforce a safe, predictable blackout on boot
    await lights.global_blackout()

    # Load standalone sounds safely
    pygame = await init_pygame()
    try:
        pygame.mixer.music.load('./media/inebriate.mp3')
    except Exception as e:
        print(f"Error loading music: {e}")

    try: sound_irrigated = pygame.mixer.Sound('./media/irrigated.wav')
    except Exception as e: sound_irrigated = None

    try: sound_irrigate = pygame.mixer.Sound('./media/irrigate.wav')
    except Exception as e: sound_irrigate = None

    # Load Snake sounds safely into the dictionary
    snake_sounds = {}
    try: snake_sounds["inferior"] = pygame.mixer.Sound('./media/inferior.wav')
    except Exception: pass
        
    try: snake_sounds["superior"] = pygame.mixer.Sound('./media/superior.wav')
    except Exception: pass

    
    print(f"AUDIO INIT CHECK - snake_sounds dict contains: {list(snake_sounds.keys())}")
    

    r1_window_start_time = 0.0
    irrigated_channel = None

    gin_task: asyncio.Task | None = None
    gin_joystick_button_states = [False, False]
    stick_override_active = False

    # --- STATE MACHINE SETUP ---
    MODE_DALEK = 0
    MODE_ARCADE = 1
    MODE_JOUST = 2
    
    current_mode = MODE_DALEK
    
    # Arcade Menu Variables
    games = [snake, invaders, runner]
    selected_game_idx = 0
    game_task = None
    preview_needs_render = True

    #Fall back to keybaord if in debug mode. left joy=wasd, right on arrow keys
    if DEBUG_MODE:
        controller_context = MockJoystick(button_mappings=buttonMappings)
        print("--- RUNNING IN KEYBOARD DEBUG MODE ---")
    else:
        controller_context = ControllerResource(dead_zone=0.1, hot_zone=0)

    with controller_context as joystick:
        while joystick.connected:
            
            # ==========================================
            # 1. READ INPUTS GLOBALLY (Must be first!)
            # ==========================================
            joystick.check_presses()
            
            # ==========================================
            # 2. ACTIVE MINI-GAME OVERRIDE & ABORT
            # ==========================================
            if game_task is not None:
                if game_task.done():
                    print("Game finished. Returning to Arcade Menu.")
                    exc = game_task.exception()
                    # Cancelled errors are intentional aborts, don't flag them as crashes
                    if exc and not isinstance(exc, asyncio.CancelledError): 
                        print(f"Game error: {exc}")
                    game_task = None
                    preview_needs_render = True # Force menu redraw
                else:
                    # Allow user to instantly abort the game using Select
                    if joystick.presses.get("select"): #["select"]:
                        print("Aborting game and switching modes...")
                        game_task.cancel()
                        game_task = None
                        # DO NOT continue here! Allow execution to fall through 
                        # to the Global Mode Selector below to instantly switch to Joust.
                    else:
                        # feed in the queue to let the game consume joystick inputs and run its loop
                        # Only track keys relevant to the games to save processing
                        for key in ["dleft", "dright", "dup", "ddown", "cross"]:
                            if key in joystick.presses and joystick.presses[key]:
                                # Limit buffer to 3 moves to prevent runaway snake lag
                                if game_queue.qsize() < 3: 
                                    game_queue.put_nowait(key)
                        await asyncio.sleep(0.01)
                        continue

            # ==========================================
            # 3. GLOBAL MODE SELECTOR
            # ==========================================
            if joystick.presses["select"]:
                # CRITICAL SAFETY INTERLOCK: Stop motors before changing modes
                if current_mode == MODE_DALEK:
                    GP8413.set_dac_out_voltage(mapStickToGP8413Value(0), channel=0)
                    GP8413.set_dac_out_voltage(mapStickToGP8413Value(0), channel=1)
                    head_motor.set_velocity(0)
                    eye_stalk_motor.set_velocity(0)

                # Cycle Mode
                current_mode = (current_mode + 1) % 3
                print(f"Switched to Mode: {current_mode}")
                
                # Cleanup previous mode hardware/processes safely
                proc = await asyncio.create_subprocess_exec("sudo", "/home/davros/JoustMania/kill_processes.sh")
                await proc.wait()
                
                await lights.global_blackout()
                
                
                if current_mode == MODE_ARCADE:
                    print("Starting ARCADE mode")
                    pins[4].on()
                    await asyncio.sleep(0.5)
                    preview_needs_render = True
                    joystick.rumble(1.0)
                elif current_mode == MODE_JOUST:
                    print("Starting JOUSTMANIA mode")
                    await lights.set_strip_color(channel=0, r=0, g=0, b=255)
                    proc = await asyncio.create_subprocess_exec("sudo", "supervisorctl", "start", "joustmania")
                    await proc.wait()
                    joystick.rumble(2.0)
                elif current_mode == MODE_DALEK:
                    pins[4].off()
                    print("Returning to DALEK mode")
                    joystick.rumble(0.5)

            # ==========================================
            # 4. ARCADE MENU LOGIC
            # ==========================================
            if current_mode == MODE_ARCADE:
                # D-Pad scrolling
                if joystick.presses["dleft"]:
                    selected_game_idx = (selected_game_idx - 1) % len(games)
                    preview_needs_render = True
                elif joystick.presses["dright"]:
                    selected_game_idx = (selected_game_idx + 1) % len(games)
                    preview_needs_render = True
                
                # Render preview frame only when selection changes
                if preview_needs_render:
                    preview_data = games[selected_game_idx].get_preview()
                    await lights.send_frame(channel=0, pixel_dict=preview_data)
                    preview_needs_render = False

                # Start Button -> Launch Game
                if joystick.presses["start"]:  
                    print(f"Launching Game: {games[selected_game_idx].__name__}")
                    await lights.set_strip_color(channel=0, r=0, g=255, b=0) # Flash green start
                    #Create a fresh queue for this specific game session
                    game_queue = asyncio.Queue()
                    game_task = asyncio.create_task(
                        games[selected_game_idx].run(joystick, lights, snake_sounds, game_queue)
                    )
                
                await asyncio.sleep(0.01)
                continue # Skip all Dalek logic below!

            # ==========================================
            # 5. JOUSTMANIA LOGIC
            # ==========================================
            elif current_mode == MODE_JOUST:
                await asyncio.sleep(0.01)
                continue # Skip all Dalek logic below!

            # ==========================================
            # 5. NORMAL DALEK LOGIC
            # ==========================================
            # The code below ONLY executes if current_mode == MODE_DALEK

            for button in buttonMappings.keys():
                if joystick.presses[button]:
                    print("Button pressed: " + button)
                    if buttonMappings[button] is not None:
                        pins[buttonMappings[button]].on()

                    # Soundbites for water pistol
                    if button == "r1":
                        current_time = time.time()
                        if current_time - r1_window_start_time >= 60.0:
                            r1_window_start_time = current_time
                            if sound_irrigated:
                                irrigated_channel = sound_irrigated.play()
                        else:
                            is_irrigated_playing = irrigated_channel is not None and irrigated_channel.get_busy()
                            if not is_irrigated_playing:
                                if sound_irrigate:
                                    sound_irrigate.play()

                if joystick.releases[button]:
                    print("Button released: " + button)
                    if buttonMappings[button] is not None:
                        pins[buttonMappings[button]].off()

            if joystick.presses["home"]: # This is 'analog' on the pihut controller - to toggle disco mode
                pins[20].toggle()
                joystick.rumble(4.0)

            # Handle "Start" button for momentary control of pin 21 - turn wheelchair hardware on and off
            if joystick.presses["start"]: # controls the second relay - toggle the wheelchair controller on and off
                pins[21].on()  # Turn on only while the button is pressed
            elif joystick.releases["start"]:
                pins[21].off()

            if webcam_available and webcam:
                if joystick.presses["rs"]:
                    webcam.face_tracking = not webcam.face_tracking
                    print("Face tracking status: " + str(webcam.face_tracking))

            # Wheelchair Drive Motors
            # Get the x, y values of the left stick
            lx_axis = joystick['lx']
            ly_axis = joystick['ly']
            #print(" lx: " + str(lx_axis) + "  ly: " + str(ly_axis))
            GP8413.set_dac_out_voltage(mapStickToGP8413Value(-lx_axis), channel=0) #l/r -ve because chair physically reversed
            GP8413.set_dac_out_voltage(mapStickToGP8413Value(ly_axis), channel=1) #f/r

            # Original DAC for original joystick emulation
            #mcp4728.channel_b.raw_value = mapStickToDacValue(-lx_axis) # -ve because chair reversed
            #mcp4728.channel_a.raw_value = mapStickToDacValue(ly_axis)

            # Right stick controls head left right, eye up down, but currently only in a binary sense
            # Disable stick/motor processing while snake is running

            rx_axis = joystick['rx']
            ry_axis = joystick['ry']

            # Face tracking / Head Override
            if webcam and (rx_axis != 0 or ry_axis != 0):
                if not stick_override_active:
                    print("Face tracking disabled via stick override")
                    stick_override_active = True
                webcam.face_tracking = False
            else:
                stick_override_active = False

            if webcam and webcam.face_tracking:
                try:
                    x, y = webcam.get_direction()
                    # Clamp face-tracking drive values to the allowed motor range.
                    ry_axis = max(-1.0, min(1.0, -y * 1.5))
                    rx_axis = max(-1.0, min(1.0, x * 1.7))

                    MIN_TRACKING_DRIVE = 0.6
                    if abs(ry_axis) < 0.1: ry_axis = 0
                    elif abs(ry_axis) < MIN_TRACKING_DRIVE: ry_axis = MIN_TRACKING_DRIVE * ry_axis / abs(ry_axis)

                    if abs(rx_axis) < 0.1: rx_axis = 0
                    elif abs(rx_axis) < MIN_TRACKING_DRIVE: rx_axis = MIN_TRACKING_DRIVE * rx_axis / abs(rx_axis)
                except Exception as e:
                        print(f"Error during face tracking: {e}")

            # Head/Eye Motors
            # Check button states before setting motor speed on head LR axis
            # switches are normally open so circuit is closed when pressed. Not ideal
            # as no failsafe in case of circuit failure, but limited by hardware options
            # for toggle switches

            head_motor.set_travel_limit(not button17.is_pressed, not button18.is_pressed)
            # Control motor speed and direction based on joystick input
            #print(f"Head request: {-rx_axis} (dir pin before={head_motor._direction_pin.value}, pwm before={head_motor._pwm_pin.value})")
            head_motor.set_velocity(-rx_axis)
            #print(f"Head actual: {head_motor.velocity} (dir pin after={head_motor._direction_pin.value}, pwm after={head_motor._pwm_pin.value})")
            if rx_axis and not head_motor.velocity:
                # Velocity not set due to travel limit reached.
                rumble(0.5, joystick)

            # Check button states before setting motor speed on eye up/down axis
            # These are the opposite of the head rotation as switches wired normally closed
            # as failsafe in case of circuit failure
            eye_stalk_motor.set_travel_limit(button23.is_pressed, button22.is_pressed)
            # Control motor speed and direction based on joystick input
            eye_stalk_motor.set_velocity(-ry_axis)
            if ry_axis and not eye_stalk_motor.velocity:
                # Velocity not set due to travel limit reached.
                rumble(0.5, joystick)

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

            # The joystick "presses since last check" loops too quickly
            # to catch both button presses at the same time, so we track
            # the joint state ourselves.

            # Gin Pump Logic
            if joystick.presses["l2"]: gin_joystick_button_states[0] = True
            elif joystick.releases["l2"]: gin_joystick_button_states[0] = False
            
            if joystick.presses["r2"]: gin_joystick_button_states[1] = True
            elif joystick.releases["r2"]: gin_joystick_button_states[1] = False
            
            if all(gin_joystick_button_states) or not gin_button.is_pressed:
                if not gin_task or gin_task.done():
                    gin_task = asyncio.create_task(pump_gin(pygame))
            elif gin_task:
                pins[7].off()
                gin_task.cancel()

            if DEBUG_MODE:
                joystick.clear_buffered_states()

            await asyncio.sleep(0.01)

ears = Ears(pins[PIN_EAR])

async def main():
    tasks = [core(), monitor_audio_output(ears)]
    if webcam_available and webcam:
        tasks.append(webcam.start())
    await asyncio.gather(*tasks)

asyncio.run(main())