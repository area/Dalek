"""
invaders.py — Horizontal Space Invaders on a 14×4 WS2811 LED grid.

Player operates in Column 0 (left).
Invaders spawn in Column 13 (right) and move left.
Bullets move right, wrap around, and can kill the player.
"""
import asyncio
import random
from dalek.game_engine import (
    SNAKE_CHANNEL, GRID_WIDTH, GRID_HEIGHT,
    get_chain_index, run_endgame_sequence
)

COLOR_PLAYER  = (0, 255, 0)   # GBR
COLOR_INVADER = (0, 0, 255)   # GBR
COLOR_BULLET  = (255, 200, 0) # GBR

def get_preview():
    frame = {}
    frame[get_chain_index(0, 2)] = COLOR_PLAYER
    frame[get_chain_index(3, 2)] = COLOR_BULLET
    frame[get_chain_index(12, 1)] = COLOR_INVADER
    frame[get_chain_index(13, 2)] = COLOR_INVADER
    frame[get_chain_index(12, 3)] = COLOR_INVADER
    frame[254] = (0, 0, 0)
    return frame

async def run(joystick, lights, sounds, input_queue):
    player_y = GRID_HEIGHT // 2
    invaders = []  
    bullets = []   
    score = 0
    alive = True

    loop = asyncio.get_event_loop()
    last_move, last_fire = 0, 0
    last_b_tick, last_i_tick, last_spawn = loop.time(), loop.time(), loop.time()
    
    i_speed = 0.2
    s_rate = 2.5

    try:
        while alive:
            now = loop.time()
            dy = 0
            
            # 1. Drain the queue for discrete events
            while not input_queue.empty():
                cmd = input_queue.get_nowait()
                if cmd == "cross" and (now - last_fire > 0.1):
                    bullets.append({"x": 1, "y": player_y, "old_x": 1})
                    last_fire = now
                elif cmd == "dup": dy = -1
                elif cmd == "ddown": dy = 1

            # 2. Fallback to analog stick for movement
            if dy == 0 and joystick:
                ly = joystick["ly"]
                if ly > 0.2: dy = -1
                elif ly < -0.2: dy = 1

            # 3. Apply vertical movement
            if dy != 0 and (now - last_move > 0.12):
                player_y = max(0, min(GRID_HEIGHT - 1, player_y + dy))
                last_move = now

                # Firing logic (Cross mapped to fire)
                if joystick.presses["cross"] and (now - last_fire > 0.1):
                    bullets.append({"x": 1, "y": player_y, "old_x": 1})
                    last_fire = now

            if now - last_b_tick >= 0.04:
                last_b_tick = now
                for b in bullets:
                    b["old_x"] = b["x"]
                    b["x"] += 1
                    if b["x"] >= GRID_WIDTH:
                        b["x"], b["old_x"] = 0, 0

            if now - last_i_tick >= i_speed:
                last_i_tick = now
                for inv in invaders:
                    inv["old_x"] = inv["x"]
                    inv["x"] -= 1

            if now - last_spawn >= s_rate:
                last_spawn = now
                y_spawn = random.randint(0, GRID_HEIGHT - 1)
                if not any(inv["x"] == 13 and inv["y"] == y_spawn for inv in invaders):
                    invaders.append({"x": 13, "y": y_spawn, "old_x": 13})

            b_rem, i_rem = [], []
            for b in bullets:
                for inv in invaders:
                    if b["y"] == inv["y"]:
                        if b["x"] == inv["x"] or (b["old_x"] <= inv["old_x"] and b["x"] >= inv["x"]):
                            if b not in b_rem: b_rem.append(b)
                            if inv not in i_rem: i_rem.append(inv)
                            score += 1
                            i_speed = max(0.05, i_speed * 0.98)
                            s_rate = max(0.5, s_rate * 0.98)

            bullets = [b for b in bullets if b not in b_rem]
            invaders = [inv for inv in invaders if inv not in i_rem]

            for b in bullets:
                if b["x"] == 0 and b["y"] == player_y: alive = False
            for inv in invaders:
                if inv["x"] == 0: alive = False

            if lights:
                frame = {get_chain_index(0, player_y): COLOR_PLAYER}
                for i in invaders: frame[get_chain_index(i["x"], i["y"])] = COLOR_INVADER
                for b in bullets: frame[get_chain_index(b["x"], b["y"])] = COLOR_BULLET
                frame[254] = (0, 0, 0)
                await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=frame)

            await asyncio.sleep(0.01)
    finally:
        await run_endgame_sequence(score, "invaders", lights, sounds)
    return score