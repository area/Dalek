"""
runner.py — Endless Runner on a 14×4 WS2811 LED grid.

The player (Green LED) moves continuously forward (wrapping around the skirt).
Obstacles (Red LEDs) spawn ahead. The player must move up and down to find the gap.
Speed increases every lap, and obstacle density increases every 5 laps.
"""
import asyncio
import random
from dalek.game_engine import (
    SNAKE_CHANNEL, GRID_WIDTH, GRID_HEIGHT,
    get_chain_index, run_endgame_sequence
)

# --- Visuals ---
COLOR_PLAYER   = (0, 255, 0)  # Green
COLOR_OBSTACLE = (255, 0, 0)  # Red

# --- Game Configuration ---
# Easily tweak these parameters to balance the difficulty
INITIAL_SPACING = 4          # Spawn an obstacle every N columns initially
SPACING_DECREASE_RATE = 5    # Decrease spacing every N complete laps
MIN_SPACING = 2              # Minimum spacing (2 = exactly 1 empty column between obstacles)
INITIAL_SPEED = 0.4          # Seconds per horizontal step
SPEED_MULTIPLIER = 0.95      # Speed increases by 5% per lap (time * 0.95)

def get_preview():
    """Returns a static frame to display in the Arcade menu."""
    frame = {}
    frame[get_chain_index(1, 2)] = COLOR_PLAYER
    
    # Static Obstacle 1 (Gap at y=2)
    frame[get_chain_index(5, 0)] = COLOR_OBSTACLE
    frame[get_chain_index(5, 1)] = COLOR_OBSTACLE
    frame[get_chain_index(5, 3)] = COLOR_OBSTACLE
    
    # Static Obstacle 2 (Gap at y=1)
    frame[get_chain_index(9, 0)] = COLOR_OBSTACLE
    frame[get_chain_index(9, 2)] = COLOR_OBSTACLE
    frame[get_chain_index(9, 3)] = COLOR_OBSTACLE
    
    frame[254] = (0, 0, 0)
    return frame

async def run(joystick, lights, sounds, input_queue):
    player_y = GRID_HEIGHT // 2
    abs_x = 0  # Absolute distance traveled
    laps = 0
    score = 0
    
    current_spacing = INITIAL_SPACING
    current_speed = INITIAL_SPEED
    
    active_obstacles = []
    next_spawn_abs_x = 4  # First obstacle spawns a few columns ahead
    
    alive = True
    loop = asyncio.get_event_loop()
    last_move_tick = loop.time()
    last_analog_move = loop.time()
    
    def spawn_obstacle():
        """Generates an obstacle ahead of the player."""
        nonlocal next_spawn_abs_x
        # Obstacles can be 1, 2, or 3 pixels tall (leaving at least a 1 pixel gap)
        blocked_count = random.choice([1, 2, 3])
        blocked_ys = random.sample(range(GRID_HEIGHT), blocked_count)
        
        active_obstacles.append({
            "abs_x": next_spawn_abs_x,
            "blocked": blocked_ys
        })
        # Advance the spawn cursor by the current spacing rules
        next_spawn_abs_x += current_spacing

    # Pre-populate the initial track ahead of the player
    # abs_x + 12 covers the entire visible wrap-around of the 14-column skirt
    while next_spawn_abs_x <= abs_x + 12:
        spawn_obstacle()
        
    try:
        while alive:
            now = loop.time()
            dy = 0
            
            # 1. PROCESS QUEUED INPUTS (Instant Vertical Movement)
            # Drains the entire buffer allowing multiple moves in a single visual tick
            while not input_queue.empty():
                cmd = input_queue.get_nowait()
                if cmd == "dup":
                    dy -= 1
                elif cmd == "ddown":
                    dy += 1
            
            # Analog Stick Fallback (Throttle it so it doesn't instantly jump to the edge)
            if dy == 0 and joystick:
                ly = joystick["ly"]
                if ly > 0.4 and (now - last_analog_move > 0.15):
                    dy -= 1
                    last_analog_move = now
                elif ly < -0.4 and (now - last_analog_move > 0.15):
                    dy += 1
                    last_analog_move = now
            
            # Apply vertical bounds
            if dy != 0:
                player_y = max(0, min(GRID_HEIGHT - 1, player_y + dy))
                
            # 2. HORIZONTAL FORWARD TICK
            if now - last_move_tick >= current_speed:
                last_move_tick = now
                abs_x += 1
                score += 1
                
                # Check for Lap Completion
                if abs_x % GRID_WIDTH == 0:
                    laps += 1
                    current_speed *= SPEED_MULTIPLIER # Increase speed
                    
                    # Check for Density Increase
                    if laps % SPACING_DECREASE_RATE == 0:
                        current_spacing = max(MIN_SPACING, current_spacing - 1)
                        
                # Extinguish obstacles passed by 2 columns
                active_obstacles = [obs for obs in active_obstacles if obs["abs_x"] >= abs_x - 1]
                
                # Spawn new obstacles dynamically on the horizon
                while next_spawn_abs_x <= abs_x + 12:
                    spawn_obstacle()

            # 3. COLLISION DETECTION
            # Check if player occupies a blocked pixel in the current absolute column
            for obs in active_obstacles:
                if obs["abs_x"] == abs_x and player_y in obs["blocked"]:
                    alive = False
                    break

            if not alive:
                break

            # 4. HARDWARE RENDER
            if lights:
                frame = {}
                
                # Draw Obstacles
                for obs in active_obstacles:
                    render_x = obs["abs_x"] % GRID_WIDTH
                    for y in obs["blocked"]:
                        frame[get_chain_index(render_x, y)] = COLOR_OBSTACLE
                        
                # Draw Player
                frame[get_chain_index(abs_x % GRID_WIDTH, player_y)] = COLOR_PLAYER
                
                frame[254] = (0, 0, 0) # Hardware render latch
                await lights.send_frame(channel=SNAKE_CHANNEL, pixel_dict=frame)
                
            # Rest the CPU to allow input polling from dalek.py
            await asyncio.sleep(0.01)
            
    finally:
        await run_endgame_sequence(score, "runner", lights, sounds)
    return score