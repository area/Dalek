import pygame
import time

# 1. Initialize the audio engine
print("Initializing audio mixer...")
pygame.mixer.init()

try:
    # 2. Try to load the file (Change the filename to match your actual file)
    print("Attempting to load 'superior.wav'...")
    superior_sound = pygame.mixer.Sound("media/inferior.wav")
    print("File loaded successfully!")
    
    # 3. Play it
    print("Playing sound...")
    superior_sound.play()
    
    # Keep the script alive for a few seconds so the audio can finish
    time.sleep(4)
    print("Test complete.")

except FileNotFoundError:
    print("CRITICAL ERROR: The file path is wrong. Python can't find the audio file.")
except pygame.error as e:
    print(f"CRITICAL ERROR: Pygame found the file, but can't read it. (Wrong format?): {e}")