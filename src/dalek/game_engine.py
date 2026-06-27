"""
game_engine.py — Shared resources and hardware logic for Dalek mini-games.
"""
import asyncio
import os

# --- Hardware Constants ---
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

STICK_THRESHOLD = 0.1

def get_chain_index(x: int, y: int) -> int:
    return GRID_MAP[y % GRID_HEIGHT][x % GRID_WIDTH]

def parse_joystick_direction(joystick, current_dir: tuple) -> tuple:
    """Standardized D-Pad and Left Stick movement."""
    dx, dy = current_dir
    if joystick.presses["dleft"]: return -1, 0
    if joystick.presses["dright"]: return 1, 0
    if joystick.presses["dup"]: return 0, 1
    if joystick.presses["ddown"]: return 0, -1
    
    lx, ly = joystick["lx"], joystick["ly"]
    if abs(lx) > abs(ly):
        if lx > STICK_THRESHOLD: return 1, 0
        elif lx < -STICK_THRESHOLD: return -1, 0
    else:
        if ly > STICK_THRESHOLD: return 0, -1
        elif ly < -STICK_THRESHOLD: return 0, 1
    return dx, dy

# --- Data & Endgame Sequence ---
def get_high_score(game_name: str) -> int:
    filename = f"{game_name}_highscore.txt"
    if os.path.exists(filename):
        try:
            with open(filename, "r") as f:
                return int(f.read().strip())
        except ValueError:
            pass
    return 0

def save_high_score(game_name: str, score: int):
    with open(f"{game_name}_highscore.txt", "w") as f:
        f.write(str(score))

async def run_endgame_sequence(score: int, game_name: str, lights, sounds):
    """The universal dramatic finale. Handles scores > 56 dynamically."""
    high_score = get_high_score(game_name)
    file_exists = os.path.exists(f"{game_name}_highscore.txt")
    is_new_record = score > high_score or not file_exists

    if is_new_record:
        save_high_score(game_name, score)
        high_score = score

    if sounds:
        if score > 0 and is_new_record and sounds.get("superior"):
            sounds["superior"].play()
        elif sounds.get("inferior"):
            sounds["inferior"].play()

    if lights is None:
        return

    try:
        # Dynamic Scaling (Prevents IndexError if score > 56)
        max_val = max(56, max(score, high_score))
        scale = 56 / max_val
        display_high = min(56, int(high_score * scale))
        display_score = min(56, int(score * scale))
        
        # Pity pixels
        if score > 0 and display_score == 0: display_score = 1
        if high_score > 0 and display_high == 0: display_high = 1

        await lights.clear_strip(channel=SNAKE_CHANNEL)
        await asyncio.sleep(0.5)

        # Draw Target in Green/Red depending on win
        score_data = {}
        for i in range(display_high):
            x, y = i // GRID_HEIGHT, i % GRID_HEIGHT
            score_data[get_chain_index(x, y)] = (0, 150, 0) if is_new_record else (150, 0, 0)
        
        score_data[254] = (0, 0, 0)
        await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_data)
        await asyncio.sleep(1.0)

        # Fill Player Score
        for i in range(display_score):
            x, y = i // GRID_HEIGHT, i % GRID_HEIGHT
            score_data[get_chain_index(x, y)] = (255, 0, 0) if is_new_record else (0, 255, 0)
            score_data[254] = (0, 0, 0)
            await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_data)
            await asyncio.sleep(max(0.02, 0.08 - (score * 0.0005)))

        if is_new_record and score > 0:
            for _ in range(4):
                await lights.clear_strip(channel=SNAKE_CHANNEL)
                await asyncio.sleep(0.15)
                await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_data)
                await asyncio.sleep(0.15)

        await asyncio.sleep(3.0)
        await lights.clear_strip(channel=SNAKE_CHANNEL)
    except asyncio.CancelledError:
        pass