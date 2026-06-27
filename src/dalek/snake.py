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
SNAKE_CHANNEL = 0  
LED_COUNT = 56     

GRID_WIDTH = 14
GRID_HEIGHT = 4

GRID_MAP = [
    [3, 4, 11, 12, 19, 20, 27, 55, 48, 47, 40, 39, 32, 31],  # Row 3 (top)
    [2, 5, 10, 13, 18, 21, 26, 54, 49, 46, 41, 38, 33, 30],  # Row 2 
    [1, 6,  9, 14, 17, 22, 25, 53, 50, 45, 42, 37, 34, 29],  # Row 1 
    [0, 7,  8, 15, 16, 23, 24, 52, 51, 44, 43, 36, 35, 28],  # Row 0 (bottom)
]

# ---------------------------------------------------------------------------
# Colours (Hardware is GRB: Component 1 = Green, Component 2 = Red)
# ---------------------------------------------------------------------------
COLOR_OFF        = (0, 0, 0)
COLOR_SNAKE_HEAD = (255, 0, 0)    # Bright Green
COLOR_SNAKE_BODY = (60, 0, 0)     # Dim Green
COLOR_FOOD       = (0, 0, 255)    # Blue

TICK_RATE = 0.4    
LOOP_SLEEP = 0.02   
STICK_THRESHOLD = 0.1

VERTICAL_MULTIPLIER = 1.8  

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
    return GRID_MAP[y % GRID_HEIGHT][x % GRID_WIDTH]

async def _render(lights, snake: list, food: tuple) -> None:
    if lights is None:
        return

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
    
    # 4. LATCH
    frame_data[254] = (0, 0, 0)
    
    await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=frame_data)

def _spawn_food(snake: list) -> tuple:
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
    print(f"AUDIO INIT CHECK - snake_sounds dict contains: {list(sounds.keys())}")
    tick_rate = TICK_RATE

    snake = [(GRID_WIDTH // 2, GRID_HEIGHT // 2)]
    direction = (1, 0)
    last_executed_direction = (1, 0) 
    
    food = _spawn_food(snake)
    score = 0
    alive = True

    await _render(lights, snake, food)

    loop = asyncio.get_event_loop()
    last_tick = loop.time()

    try:
        while alive:
            now = loop.time()

            # --- Process Movement Input ---
            if joystick is not None:
                new_dir = _direction_from_joystick(joystick, direction)
                if not (new_dir[0] == -last_executed_direction[0] and new_dir[1] == -last_executed_direction[1]):
                    direction = new_dir

            # --- Apply Axis-Aware Speed Scaling ---
            current_tick_delay = tick_rate * VERTICAL_MULTIPLIER if direction[0] == 0 else tick_rate

            # --- Process Game Mechanics Step ---
            if now - last_tick >= current_tick_delay:
                last_tick = now
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
                    tick_rate *= 0.95  
                    print(f"Score: {score}")
                    if len(snake) == LED_COUNT:
                        print("Perfect Game! You win!")
                        alive = False
                        break
                    food = _spawn_food(snake)
                else:
                    snake.pop()

                await _render(lights, snake, food)

            await asyncio.sleep(LOOP_SLEEP)

    finally:
        # --- UNCONDITIONAL LOGIC (Runs regardless of hardware presence) ---
        file_exists = os.path.exists(HIGH_SCORE_FILE)
        high_score = get_high_score()
        
        # Determine if it's a record or if the file needs initialization
        is_new_record = score > high_score or not file_exists

        if is_new_record:
            save_high_score(score)
            high_score = score  # Sync for console log display

        # Play audio alerts immediately before hardware updates
        # Play audio alerts immediately before hardware updates
        if sounds:
            print(f"[DEBUG] Sounds dictionary detected! Available keys: {list(sounds.keys())}")
            if score > 0 and is_new_record:
                print("[DEBUG] Logic check: New Record! Attempting to play 'superior'...")
                if sounds.get("superior"):
                    sounds["superior"].play()
                    print("[DEBUG] Success: 'superior' play() command triggered.")
                else:
                    print("[DEBUG] Error: 'superior' key missing from sounds dictionary!")
            else:
                print("[DEBUG] Logic check: No record. Attempting to play 'inferior'...")
                if sounds.get("inferior"):
                    sounds["inferior"].play()
                    print("[DEBUG] Success: 'inferior' play() command triggered.")
                else:
                    print("[DEBUG] Error: 'inferior' key missing from sounds dictionary!")
        else:
            print("[DEBUG] Warning: No 'sounds' dictionary was passed into the run() function.")

        # --- CONDITIONAL VISUAL PRESENTATION ---
        if lights is not None:
            try:
                # 1. Clear the board for dramatic effect
                await lights.clear_strip(channel=SNAKE_CHANNEL)
                await asyncio.sleep(0.5)

                # 2. Draw High Score columns in green, unless it is a new high score, in which case red (GRB: 0, 150, 0)
                score_render_data = {}
                for i in range(high_score):
                    x = i // GRID_HEIGHT
                    y = i % GRID_HEIGHT
                    if is_new_record: 
                        score_render_data[_chain_index(x, y)] = (0, 150, 0) 
                    else:
                        score_render_data[_chain_index(x, y)] = (150, 0, 0)    
                
                score_render_data[254] = (0, 0, 0) # Latch
                await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
                await asyncio.sleep(1.0) 

                # 3. Current Score fills in Red (GRB: 0, 255, 0)
                for i in range(score):
                    x = i // GRID_HEIGHT
                    y = i % GRID_HEIGHT
                    if is_new_record: # render in green
                        score_render_data[_chain_index(x, y)] = (255, 0, 0) 
                    else: # red
                        score_render_data[_chain_index(x, y)] = (0, 255, 0)
                    
                    score_render_data[254] = (0, 0, 0)
                    await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
                    await asyncio.sleep(0.08)

                # 4. If they won, flash the scoreboard
                if is_new_record and score > 0:
                    for _ in range(4):
                        await lights.clear_strip(channel=SNAKE_CHANNEL)
                        await asyncio.sleep(0.15)
                        await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
                        await asyncio.sleep(0.15)
                
                await asyncio.sleep(3.0)
                await lights.clear_strip(channel=SNAKE_CHANNEL)
            except asyncio.CancelledError:
                # Catch cancellations cleanly so the loop doesn't raise raw stack traces
                pass

        print(f"Game over. Final score: {score} | High Score: {high_score}")
        return score