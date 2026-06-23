"""
snake.py — Snake game on a 14×4 WS2811 LED grid routed via Arduino packets.

Grid coordinates: (x, y) where x=0 is left, y=0 is top.
Both axes wrap (left↔right, top↔bottom).
"""

import asyncio
import random

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
COLOR_SNAKE_HEAD = (0, 255, 0)    # Bright green
COLOR_SNAKE_BODY = (0, 80, 0)     # Dim green
COLOR_FOOD       = (255, 0, 0)     # Red

TICK_RATE = 0.25    
LOOP_SLEEP = 0.02   
STICK_THRESHOLD = 0.5

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
    
    # Debug print to console
    print(f"Frame Rendered -> Head: {snake[0]} | Food: {food} | Snake Length: {len(snake)} | Packets Sent: {len(frame_data) + 1}")

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
        dx, dy = 0, -1
    elif joystick.presses["ddown"]:
        dx, dy = 0, 1
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
                dx, dy = 0, 1
            elif ly < -STICK_THRESHOLD:
                dx, dy = 0, -1

    return dx, dy

# ---------------------------------------------------------------------------
# Main Game Loop Entry Point
# ---------------------------------------------------------------------------

async def run(joystick=None, lights=None) -> int:
    """Plays one game of Snake using the external dynamic lights module."""
    tick_rate = TICK_RATE

    # Initial state initialization
    snake = [(GRID_WIDTH // 2, GRID_HEIGHT // 2)]
    direction = (1, 0)
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
                if not (new_dir[0] == -direction[0] and new_dir[1] == -direction[1]):
                    direction = new_dir

            # --- Process Game Mechanics Step ---
            if now - last_tick >= tick_rate:
                last_tick = now

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
                    tick_rate *= 0.95  # Gradually accelerate frame timing
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
        # Game over blinking sequence
        if lights is not None:
            for _ in range(4):
                await lights.clear_strip(channel=SNAKE_CHANNEL)
                await asyncio.sleep(0.15)
                await _render(lights, snake, food)
                await asyncio.sleep(0.15)
            await lights.clear_strip(channel=SNAKE_CHANNEL)

    print(f"Game over. Final score: {score}")
    return score
