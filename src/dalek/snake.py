"""
snake.py  —  Snake game on a 14×4 WS2811 LED grid.

Grid coordinates: (x, y) where x=0 is left, y=0 is top.
Both axes wrap (left↔right, top↔bottom).

GRID_MAP[row][col] gives the chain index for that LED.
Edit it to match your physical wiring.
"""

import argparse
import asyncio
import random

from rpi_ws281x import Color, PixelStrip

# ---------------------------------------------------------------------------
# Hardware configuration
# ---------------------------------------------------------------------------

LED_COUNT = 56        # Total LEDs in chain
LED_PIN = 21          # GPIO pin (18 = PWM0, 21 = PCM, 10 = SPI MOSI)
LED_FREQ_HZ = 800000  # WS2811 data rate
LED_DMA = 10          # DMA channel
LED_BRIGHTNESS =255   # 0–255
LED_INVERT = False    # True if using NPN transistor level-shift
LED_CHANNEL = 0       # 0 for GPIO 18/10, 1 for GPIO 13/19

GRID_WIDTH = 14
GRID_HEIGHT = 4

# ---------------------------------------------------------------------------
# Lookup table: GRID_MAP[row][col] → chain index
#
# Default assumes a zig-zag wiring pattern:
#   Row 0: L→R  (indices  0–13)
#   Row 1: R→L  (indices 27–14)
#   Row 2: L→R  (indices 28–41)
#   Row 3: R→L  (indices 55–42)
#
# Replace any value to match your actual wiring.
# ---------------------------------------------------------------------------

GRID_MAP = [
    [3, 4, 11, 12, 19, 20, 27, 55, 48, 47, 40, 39, 32, 31],  # Row 3 (top)
    [2, 5, 10, 13, 18, 21, 26, 54, 49, 46, 41, 38, 33, 30],  # Row 2 
    [1, 6,  9, 14, 17, 22, 25, 53, 50, 45, 42, 37, 34, 29],  # Row 1 
    [0, 7,  8, 15, 16, 23, 24, 52, 51, 44, 43, 36, 35, 28],  # Row 0 (bottom)
]
# LEDs are linked through columns 1-7 along left side , then back to front middle, and around right side
# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------

COLOR_OFF        = Color(0,   0,   0)
COLOR_SNAKE_HEAD = Color(0,   255, 0)    # Bright green
COLOR_SNAKE_BODY = Color(0,   80,  0)    # Dim green
COLOR_FOOD       = Color(255, 0,   0)    # Red

# ---------------------------------------------------------------------------
# Game settings
# ---------------------------------------------------------------------------

TICK_RATE = 0.25    # Seconds per game step (lower = faster)
LOOP_SLEEP = 0.02   # Asyncio sleep between input polls (seconds)

# Dead-zone for analogue stick direction input
STICK_THRESHOLD = 0.5

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _chain_index(x: int, y: int) -> int:
    """Return the chain index for grid position (x, y) using GRID_MAP."""
    return GRID_MAP[y % GRID_HEIGHT][x % GRID_WIDTH]


def _set_pixel(strip: PixelStrip, x: int, y: int, color: Color) -> None:
    strip.setPixelColor(_chain_index(x, y), color)


def _clear(strip: PixelStrip) -> None:
    for i in range(LED_COUNT):
        strip.setPixelColor(i, COLOR_OFF)


def _render(strip: PixelStrip, snake: list, food: tuple) -> None:
    """Render snake and food to the strip (does not call show())."""
    _clear(strip)
    _set_pixel(strip, food[0], food[1], COLOR_FOOD)
    for seg in reversed(snake):
        _set_pixel(strip, seg[0], seg[1], COLOR_SNAKE_BODY)
    _set_pixel(strip, snake[0][0], snake[0][1], COLOR_SNAKE_HEAD)
    strip.show()


class DummyPixelStrip:
    def __init__(self, led_count, *args, **kwargs):
        self.led_count = led_count
        self.pixels = [COLOR_OFF] * led_count

    def begin(self):
        return True

    def setPixelColor(self, index, color):
        if 0 <= index < self.led_count:
            self.pixels[index] = color

    def show(self):
        # No hardware attached; keep this quiet or print a minimal status.
        print(f"DummyStrip show: head={self.pixels.count(COLOR_SNAKE_HEAD)}, food={self.pixels.count(COLOR_FOOD)}")

    def numPixels(self):
        return self.led_count


def _spawn_food(snake: list) -> tuple:
    """Pick a random cell not occupied by the snake."""
    occupied = set(snake)
    candidates = [
        (x, y)
        for y in range(GRID_HEIGHT)
        for x in range(GRID_WIDTH)
        if (x, y) not in occupied
    ]
    return random.choice(candidates) if candidates else (0, 0)


def _direction_from_joystick(joystick, current_dir: tuple) -> tuple:
    """
    Read direction from the joystick.

    Priority:
      1. D-pad presses  (dleft, dright, dup, ddown)
      2. Left analogue stick if past STICK_THRESHOLD
      3. Current direction unchanged
    """
    dx, dy = current_dir

    # D-pad (discrete presses)
    if joystick.presses.get("dleft"):
        dx, dy = -1, 0
    elif joystick.presses.get("dright"):
        dx, dy = 1, 0
    elif joystick.presses.get("dup"):
        dx, dy = 0, -1
    elif joystick.presses.get("ddown"):
        dx, dy = 0, 1
    else:
        # Analogue left stick
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
# Public entry point
# ---------------------------------------------------------------------------

async def run(joystick=None, use_dummy_strip=False) -> int:
    """
    Play one game of Snake.

    Args:
        joystick: An approxeng joystick resource (already open).
                  If None the snake moves automatically (demo / test mode).
        use_dummy_strip: If True, run without WS2811 hardware attached.

    Returns:
        Final score (number of food items eaten).
    """
    if use_dummy_strip:
        strip = DummyPixelStrip(LED_COUNT)
    else:
        strip = PixelStrip(
            LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA,
            LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL,
        )
    strip.begin()

    # Initial state: snake starts in the middle of the grid, moving right
    snake = [(GRID_WIDTH // 2, GRID_HEIGHT // 2)]
    direction = (1, 0)
    food = _spawn_food(snake)
    score = 0
    alive = True

    _render(strip, snake, food)

    loop = asyncio.get_event_loop()
    last_tick = loop.time()

    try:
        while alive:
            now = loop.time()

            # --- Input --------------------------------------------------
            if joystick is not None:
                joystick.check_presses()
                new_dir = _direction_from_joystick(joystick, direction)
                # Prevent 180° reversal
                if not (new_dir[0] == -direction[0] and new_dir[1] == -direction[1]):
                    direction = new_dir

            # --- Game tick ----------------------------------------------
            if now - last_tick >= TICK_RATE:
                last_tick = now

                head = snake[0]
                new_head = (
                    (head[0] + direction[0]) % GRID_WIDTH,
                    (head[1] + direction[1]) % GRID_HEIGHT,
                )

                # Self-collision → game over
                if new_head in snake:
                    alive = False
                    break

                snake.insert(0, new_head)

                if new_head == food:
                    score += 1
                    print(f"Score: {score}")
                    if len(snake) == LED_COUNT:
                        # Filled the whole grid — you win!
                        print("You win!")
                        alive = False
                        break
                    food = _spawn_food(snake)
                else:
                    snake.pop()

                _render(strip, snake, food)

            await asyncio.sleep(LOOP_SLEEP)

    finally:
        # Game-over flash
        for _ in range(4):
            _clear(strip)
            strip.show()
            await asyncio.sleep(0.15)
            _render(strip, snake, food)
            await asyncio.sleep(0.15)
        _clear(strip)
        strip.show()

    print(f"Game over. Final score: {score}")
    return score


# ---------------------------------------------------------------------------
# Standalone entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    from approxeng.input.selectbinder import ControllerResource

    parser = argparse.ArgumentParser(description="Run snake on WS2811 LEDs")
    parser.add_argument("--dummy", action="store_true", help="Run without LED hardware")
    parser.add_argument("--no-joystick", action="store_true", help="Run demo mode without joystick")
    args = parser.parse_args()

    async def _main():
        joystick = None
        if not args.no_joystick and not args.dummy:
            try:
                with ControllerResource(dead_zone=0.1, hot_zone=0) as js:
                    joystick = js
                    print("Controller found. Starting Snake...")
                    await run(joystick, use_dummy_strip=args.dummy)
                    return
            except Exception:
                print("No controller found. Running in demo mode...")

        await run(joystick=None, use_dummy_strip=args.dummy)

    asyncio.run(_main())
