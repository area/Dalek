"""
snake.py — Snake game on a 14×4 WS2811 LED grid routed via Arduino packets.

Grid coordinates: (x, y) where x=0 is left, y=0 is top.
Both axes wrap (left↔right, top↔bottom).
"""
import asyncio
import random
from dalek.game_engine import (
    SNAKE_CHANNEL, GRID_WIDTH, GRID_HEIGHT, LED_COUNT,
    get_chain_index, parse_joystick_direction, run_endgame_sequence
)

COLOR_HEAD = (255, 0, 0) # GRB Green
COLOR_BODY = (60, 0, 0)
COLOR_FOOD = (0, 0, 255)

def get_preview():
    """Returns a static frame to display in the game menu."""
    frame = {}
    # Draw a little snake shape
    for x in range(3, 7): frame[get_chain_index(x, 1)] = COLOR_BODY
    frame[get_chain_index(7, 1)] = COLOR_HEAD
    frame[get_chain_index(10, 1)] = COLOR_FOOD
    frame[254] = (0, 0, 0)
    return frame

async def run(joystick, lights, sounds, input_queue):
    snake = [(GRID_WIDTH // 2, GRID_HEIGHT // 2)]
    direction = (1, 0)
    last_exec_dir = (1, 0)
    
    def spawn_food():
        c = [(x, y) for y in range(GRID_HEIGHT) for x in range(GRID_WIDTH) if (x, y) not in set(snake)]
        return random.choice(c) if c else (0, 0)
        
    food = spawn_food()
    score = 0
    tick_rate = 0.4
    alive = True
    loop = asyncio.get_event_loop()
    last_tick = loop.time()

    try:
        while alive:
            now = loop.time()
            delay = tick_rate * 1.8 if direction[0] == 0 else tick_rate

            # Tick here
            if now - last_tick >= delay:
                # Pop one discrete command from the queue
                if not input_queue.empty():
                    cmd = input_queue.get_nowait()
                    new_dir = direction
                    if cmd == "dleft": new_dir = (-1, 0)
                    elif cmd == "dright": new_dir = (1, 0)
                    elif cmd == "dup": new_dir = (0, 1)
                    elif cmd == "ddown": new_dir = (0, -1)
                    
                    # Validate it's not a 180-degree suicide turn
                    if not (new_dir[0] == -last_exec_dir[0] and new_dir[1] == -last_exec_dir[1]):
                        direction = new_dir
                        
                # Fall back to analogue stick if queue is empty
                elif joystick:
                    lx, ly = joystick["lx"], joystick["ly"]
                    if abs(lx) > 0.2 or abs(ly) > 0.2:
                        new_dir = direction
                        if abs(lx) > abs(ly):
                            new_dir = (1, 0) if lx > 0 else (-1, 0)
                        else:
                            new_dir = (0, -1) if ly > 0 else (0, 1)
                            
                        if not (new_dir[0] == -last_exec_dir[0] and new_dir[1] == -last_exec_dir[1]):
                            direction = new_dir

                # move
                last_tick = now
                last_exec_dir = direction
                head = snake[0]
                new_head = ((head[0] + direction[0]) % GRID_WIDTH, (head[1] + direction[1]) % GRID_HEIGHT)
                
                if new_head in snake:
                    alive = False
                    break

                snake.insert(0, new_head)
                if new_head == food:
                    score += 1
                    tick_rate *= 0.95
                    if len(snake) == LED_COUNT:
                        alive = False
                        break
                    food = spawn_food()
                else:
                    snake.pop()

                if lights:
                    frame = {get_chain_index(fx, fy): COLOR_BODY for fx, fy in snake}
                    frame[get_chain_index(new_head[0], new_head[1])] = COLOR_HEAD
                    frame[get_chain_index(food[0], food[1])] = COLOR_FOOD
                    frame[254] = (0, 0, 0)
                    await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=frame)

            await asyncio.sleep(0.02)
    finally:
        await run_endgame_sequence(score, "snake", lights, sounds)
    return score