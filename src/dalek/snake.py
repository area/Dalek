"""
snake.py — Snake game on a 14×4 WS2811 LED grid routed via Arduino packets.

Grid coordinates: (x, y) where x=0 is left, y=0 is top.
Both axes wrap (left↔right, top↔bottom).
"""

import asyncio
import random
import os

# ---------------------------------------------------------------------------
# Hardware & Routing configuration
# ---------------------------------------------------------------------------
# Specify which Arduino channel strip controls your 56-LED snake grid matrix
SNAKE_CHANNEL = 0  
LED_COUNT = 56     

GRID_WIDTH = 14
GRID_HEIGHT = 4

# Lookup table: GRID_MAP[row][col] → chain index matching physical wiring layout
GRID_MAP = [
    [3, 4, 11, 12, 19, 20, 27, 55, 48, 47, 40, 39, 32, 31],  # Row 3 (top)
    [2, 5, 10, 13, 18, 21, 26, 54, 49, 46, 41, 38, 33, 30],  # Row 2 
    [1, 6,  9, 14, 17, 22, 25, 53, 50, 45, 42, 37, 34, 29],  # Row 1 
    [0, 7,  8, 15, 16, 23, 24, 52, 51, 44, 43, 36, 35, 28],  # Row 0 (bottom)
]

# ---------------------------------------------------------------------------
# Colours (Converted to raw RGB tuples for direct packet integration)
# ---------------------------------------------------------------------------
COLOR_OFF        = (0, 0, 0)
COLOR_SNAKE_HEAD = (255, 0, 0)    # Bright red
COLOR_SNAKE_BODY = (60, 0, 0)     # Dim red
COLOR_FOOD       = (0, 0, 255)     # Blue

TICK_RATE = 0.25    
LOOP_SLEEP = 0.02   
STICK_THRESHOLD = 0.2

# --- NEW TWEAK: Vertical speed scaling ---
VERTICAL_MULTIPLIER = 1.8  # 1.8x slower when moving up or down

HIGH_SCORE_FILE = "snake_highscore.txt"

# ---------------------------------------------------------------------------
# Storage Helpers
# ---------------------------------------------------------------------------

def get_high_score() -> int:
    if os.path.exists(HIGH_SCORE_FILE):
        try:
            with open(HIGH_SCORE_FILE, "r") as f:
                return int(f.read().strip())
        except ValueError:
            return 0
    return 0

def save_high_score(score: int):
    with open(HIGH_SCORE_FILE, "w") as f:
        f.write(str(score))

# ---------------------------------------------------------------------------
# Async Packet Helpers
# ---------------------------------------------------------------------------

def _chain_index(x: int, y: int) -> int:
    """Return the chain index for grid position (x, y) using GRID_MAP."""
    return GRID_MAP[y % GRID_HEIGHT][x % GRID_WIDTH]


async def _render(lights, snake: list, food: tuple) -> None:
    if lights is None:
        return

    # Build the complete picture layout for this frame frame in memory
    frame_data = {}

    # 1. Add food position
    fx, fy = food
    frame_data[_chain_index(fx, fy)] = COLOR_FOOD

    # 2. Add snake body layout
    for seg in reversed(snake):
        frame_data[_chain_index(seg[0], seg[1])] = COLOR_SNAKE_BODY

    # 3. Layer the head on top
    hx, hy = snake[0]
    frame_data[_chain_index(hx, hy)] = COLOR_SNAKE_HEAD
    
    # 4. EXPLICIT LATCH: Add the magic index 254 to force the hardware render.
    frame_data[254] = (0, 0, 0)
    
    # Debug print to console
    print(f"Frame Rendered -> Head: {snake[0]} | Food: {food} | Snake Length: {len(snake)} | Packets Sent: {len(frame_data)}")

    # Send the whole dictionary data map down the pipe in a single step
    await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=frame_data)

def _spawn_food(snake: list) -> tuple:
    """Pick a random cell not occupied by the snake body layer."""
    occupied = set(snake)
    candidates = [
        (x, y)
        for y in range(GRID_HEIGHT)
        for x in range(GRID_WIDTH)
        if (x, y) not in occupied
    ]
    return random.choice(candidates) if candidates else (0, 0)


def _direction_from_joystick(joystick, current_dir: tuple) -> tuple:
    dx, dy = current_dir

    if joystick.presses["dleft"]:
        dx, dy = -1, 0
    elif joystick.presses["dright"]:
        dx, dy = 1, 0
    elif joystick.presses["dup"]:
        dx, dy = 0, 1
    elif joystick.presses["ddown"]:
        dx, dy = 0, -1
    else:
        lx = joystick["lx"]
        ly = joystick["ly"]
        if abs(lx) > abs(ly):
            if lx > STICK_THRESHOLD:
                dx, dy = 1, 0
            elif lx < -STICK_THRESHOLD:
                dx, dy = -1, 0
        else:
            if ly > STICK_THRESHOLD:
                dx, dy = 0, -1
            elif ly < -STICK_THRESHOLD:
                dx, dy = 0, 1

    return dx, dy

# ---------------------------------------------------------------------------
# Main Game Loop Entry Point
# ---------------------------------------------------------------------------

async def run(joystick=None, lights=None, sounds=None) -> int:
    """Plays one game of Snake using the external dynamic lights module."""
    tick_rate = TICK_RATE

    # Initial state initialization
    snake = [(GRID_WIDTH // 2, GRID_HEIGHT // 2)]
    direction = (1, 0)
    
    # Track the actual movement direction of the LAST executed step
    last_executed_direction = (1, 0) 
    
    food = _spawn_food(snake)
    score = 0
    alive = True

    # Draw the initial game board frame
    await _render(lights, snake, food)

    loop = asyncio.get_event_loop()
    last_tick = loop.time()

    try:
        while alive:
            now = loop.time()

            # --- Process Movement Input ---
            if joystick is not None:
                new_dir = _direction_from_joystick(joystick, direction)
                
                # Compare against last_executed_direction instead of current input direction
                if not (new_dir[0] == -last_executed_direction[0] and new_dir[1] == -last_executed_direction[1]):
                    direction = new_dir

            # --- Apply Axis-Aware Speed Scaling ---
            current_tick_delay = tick_rate * VERTICAL_MULTIPLIER if direction[0] == 0 else tick_rate

            # --- Process Game Mechanics Step ---
            if now - last_tick >= current_tick_delay:
                last_tick = now

                # Lock in the direction we are actively moving for this step
                last_executed_direction = direction

                head = snake[0]
                new_head = (
                    (head[0] + direction[0]) % GRID_WIDTH,
                    (head[1] + direction[1]) % GRID_HEIGHT,
                    )

                if new_head in snake:
                    alive = False
                    break

                snake.insert(0, new_head)

                if new_head == food:
                    score += 1
                    tick_rate *= 0.95  # Gradually accelerate base frame timing
                    print(f"Score: {score}")
                    if len(snake) == LED_COUNT:
                        print("Perfect Game! You win!")
                        alive = False
                        break
                    food = _spawn_food(snake)
                else:
                    snake.pop()

                # Refresh display frame
                await _render(lights, snake, food)

            await asyncio.sleep(LOOP_SLEEP)

    finally:
        # --- ENDGAME HIGH SCORE SEQUENCE ---
        if lights is not None:
            high_score = get_high_score()
            is_new_record = score > high_score

            # 1. Clear the board for dramatic effect
            await lights.clear_strip(channel=SNAKE_CHANNEL)
            await asyncio.sleep(0.5)

            # Draw High Score columns in Green
            score_render_data = {}
            for i in range(high_score):
                x = i // GRID_HEIGHT
                y = i % GRID_HEIGHT
                score_render_data[_chain_index(x, y)] = (150, 0, 0) # Green (Base Record)
            
            score_render_data[254] = (0, 0, 0) # Latch
            await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
            
            # Pause to show the target to beat
            await asyncio.sleep(1.0) 

            #Current Score fills in Red
            for i in range(score):
                x = i // GRID_HEIGHT
                y = i % GRID_HEIGHT
                score_render_data[_chain_index(x, y)] = (0, 255, 0) # Red (Player Score)
                score_render_data[254] = (0, 0, 0)
                await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
                await asyncio.sleep(0.08) # Satisfying fast-fill effect

            # judge them
            if sounds:
                if is_new_record:
                    if sounds.get("superior"): 
                        sounds["superior"].play()
                    save_high_score(score)
                    
                    # Flash the winning score
                    for _ in range(4):
                        await lights.clear_strip(channel=SNAKE_CHANNEL)
                        await asyncio.sleep(0.15)
                        await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
                        await asyncio.sleep(0.15)
                else:
                    if sounds.get("inferior"): 
                        sounds["inferior"].play()
            
            # Leave score up for a bit
            await asyncio.sleep(3.0)
            await lights.clear_strip(channel=SNAKE_CHANNEL)

    print(f"Game over. Final score: {score} | High Score: {get_high_score()}")
    return score