"""
invaders.py — Horizontal Space Invaders on a 14×4 WS2811 LED grid.

Player operates in Column 0 (left).
Invaders spawn in Column 13 (right) and move left.
Bullets move right, wrap around, and can kill the player.
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

# Lookup table: GRID_MAP[row][col] → chain index matching physical wiring layout
GRID_MAP = [
    [3, 4, 11, 12, 19, 20, 27, 55, 48, 47, 40, 39, 32, 31],  # Row 3 (top)
    [2, 5, 10, 13, 18, 21, 26, 54, 49, 46, 41, 38, 33, 30],  # Row 2 
    [1, 6,  9, 14, 17, 22, 25, 53, 50, 45, 42, 37, 34, 29],  # Row 1 
    [0, 7,  8, 15, 16, 23, 24, 52, 51, 44, 43, 36, 35, 28],  # Row 0 (bottom)
]

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
COLOR_OFF      = (0, 0, 0)
COLOR_PLAYER   = (0, 255, 0)    # Green
COLOR_INVADER  = (0, 0, 255)    # Blue
COLOR_BULLET   = (255, 200, 0)  # Yellow-ish Orange

# ---------------------------------------------------------------------------
# Game Timings & Settings
# ---------------------------------------------------------------------------
LOOP_SLEEP = 0.01          # Fast main loop sleep
PLAYER_MOVE_TICK = 0.12    # Prevent moving too fast when holding the stick
FIRE_COOLDOWN = 0.25       # Delay between shots
BULLET_TICK = 0.04         # How fast bullets travel
STICK_THRESHOLD = 0.2

HIGH_SCORE_FILE = "invaders_highscore.txt"

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

async def _render(lights, player_y: int, invaders: list, bullets: list) -> None:
    if lights is None:
        return

    frame_data = {}

    # 1. Draw player
    frame_data[_chain_index(0, player_y)] = COLOR_PLAYER

    # 2. Draw invaders
    for inv in invaders:
        frame_data[_chain_index(inv["x"], inv["y"])] = COLOR_INVADER

    # 3. Draw bullets
    for b in bullets:
        frame_data[_chain_index(b["x"], b["y"])] = COLOR_BULLET
    
    # 4. Latch
    frame_data[254] = (0, 0, 0)
    
    await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=frame_data)

# ---------------------------------------------------------------------------
# Main Game Loop
# ---------------------------------------------------------------------------
async def run(joystick=None, lights=None, sounds=None) -> int:
    player_y = GRID_HEIGHT // 2
    
    invaders = []  # List of dicts: {"x": 13, "y": random, "old_x": 13}
    bullets = []   # List of dicts: {"x": 1, "y": player_y, "old_x": 1}
    
    score = 0
    alive = True

    # Timers
    loop = asyncio.get_event_loop()
    last_move_tick = 0
    last_fire_tick = 0
    last_bullet_tick = loop.time()
    last_invader_tick = loop.time()
    last_spawn_tick = loop.time()

    # Dynamic Difficulty Variables
    invader_speed = 0.6   # Seconds between invader movements
    spawn_rate = 2.5      # Seconds between new enemy spawns

    await _render(lights, player_y, invaders, bullets)

    try:
        while alive:
            now = loop.time()

            # --- 1. Process Input ---
            if joystick is not None:
                # Movement
                dy = 0
                if joystick.presses["dup"]: dy = -1
                elif joystick.presses["ddown"]: dy = 1
                else:
                    ly = joystick["ly"]
                    if ly > STICK_THRESHOLD: dy = -1
                    elif ly < -STICK_THRESHOLD: dy = 1

                if dy != 0 and (now - last_move_tick > PLAYER_MOVE_TICK):
                    player_y = max(0, min(GRID_HEIGHT - 1, player_y + dy))
                    last_move_tick = now

                # Firing (Triangle button)
                if joystick.presses["triangle"] and (now - last_fire_tick > FIRE_COOLDOWN):
                    # Spawn bullet just in front of the player
                    bullets.append({"x": 1, "y": player_y, "old_x": 1})
                    last_fire_tick = now

            # --- 2. Update Bullets ---
            if now - last_bullet_tick >= BULLET_TICK:
                last_bullet_tick = now
                for b in bullets:
                    b["old_x"] = b["x"]
                    b["x"] += 1
                    # Wrap around logic
                    if b["x"] >= GRID_WIDTH:
                        b["x"] = 0
                        # Reset old_x to 0 so we don't accidentally register a 
                        # crossover hit with an invader across the entire screen!
                        b["old_x"] = 0 

            # --- 3. Update Invaders ---
            if now - last_invader_tick >= invader_speed:
                last_invader_tick = now
                for inv in invaders:
                    inv["old_x"] = inv["x"]
                    inv["x"] -= 1

            # --- 4. Spawn Invaders ---
            if now - last_spawn_tick >= spawn_rate:
                last_spawn_tick = now
                y_spawn = random.randint(0, GRID_HEIGHT - 1)
                
                # Make sure we don't spawn two directly on top of each other
                if not any(inv["x"] == 13 and inv["y"] == y_spawn for inv in invaders):
                    invaders.append({"x": 13, "y": y_spawn, "old_x": 13})

            # --- 5. Collision Detection ---
            bullets_to_remove = []
            invaders_to_remove = []

            # Bullets vs Invaders
            for b in bullets:
                for inv in invaders:
                    if b["y"] == inv["y"]:
                        # Check exact overlap OR if they physically crossed each other in the same tick
                        if b["x"] == inv["x"] or (b["old_x"] <= inv["old_x"] and b["x"] >= inv["x"]):
                            if b not in bullets_to_remove: bullets_to_remove.append(b)
                            if inv not in invaders_to_remove: invaders_to_remove.append(inv)
                            score += 1
                            print(f"Hit! Score: {score}")
                            
                            # Speed up the game slowly with every kill!
                            invader_speed = max(0.1, invader_speed * 0.98)
                            spawn_rate = max(0.5, spawn_rate * 0.98)

            # Process destruction
            bullets = [b for b in bullets if b not in bullets_to_remove]
            invaders = [inv for inv in invaders if inv not in invaders_to_remove]

            # Friendly Fire (Did a wrapped bullet hit the player?)
            for b in bullets:
                if b["x"] == 0 and b["y"] == player_y:
                    print("FRIENDLY FIRE!")
                    alive = False

            # Invader breached defenses
            for inv in invaders:
                if inv["x"] == 0:
                    print("INVADER BREACHED!")
                    alive = False

            # --- 6. Render Frame ---
            await _render(lights, player_y, invaders, bullets)
            await asyncio.sleep(LOOP_SLEEP)

    finally:
        # --- ENDGAME HIGH SCORE SEQUENCE ---
        if lights is not None:
            high_score = get_high_score()
            is_new_record = score > high_score

            # 1. Clear board
            await lights.clear_strip(channel=SNAKE_CHANNEL)
            await asyncio.sleep(0.5)

            # 2. Draw High Score in Green
            score_render_data = {}
            for i in range(high_score):
                x = i // GRID_HEIGHT
                y = i % GRID_HEIGHT
                score_render_data[_chain_index(x, y)] = (0, 150, 0) 
            
            score_render_data[254] = (0, 0, 0) 
            await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
            await asyncio.sleep(1.0) 

            # 3. Animate Current Score in Red
            for i in range(score):
                x = i // GRID_HEIGHT
                y = i % GRID_HEIGHT
                score_render_data[_chain_index(x, y)] = (255, 0, 0)
                score_render_data[254] = (0, 0, 0)
                await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
                await asyncio.sleep(0.08)

            # 4. Judgement
            if sounds:
                if is_new_record:
                    if sounds.get("superior"): sounds["superior"].play()
                    save_high_score(score)
                    
                    for _ in range(4):
                        await lights.clear_strip(channel=SNAKE_CHANNEL)
                        await asyncio.sleep(0.15)
                        await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=score_render_data)
                        await asyncio.sleep(0.15)
                else:
                    if sounds.get("inferior"): sounds["inferior"].play()
            
            await asyncio.sleep(3.0)
            await lights.clear_strip(channel=SNAKE_CHANNEL)

    print(f"Invaders Game Over. Final score: {score} | High Score: {get_high_score()}")
    return score
