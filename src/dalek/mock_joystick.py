# src/dalek/mock_joystick.py
import sys
import select
import termios
import tty

class MockJoystick:
    def __init__(self, button_mappings=None):
        self.connected = True
        self._held_keys = set()
        
        # Save original terminal settings so we don't break your bash prompt on exit
        self.old_settings = termios.tcgetattr(sys.stdin)
        # Set terminal to cbreak mode (reads keys instantly without needing 'Enter')
        tty.setcbreak(sys.stdin.fileno())
        
        buttons = list(button_mappings.keys()) if button_mappings else []
        self.presses = {b: False for b in buttons}
        # Guarantee all auxiliary/unmapped buttons required by core loop or snake exist
        self.presses.update({
            "select": False, "home": False, "rs": False, "l2": False, "r2": False,
            "dup": False, "ddown": False, "dleft": False, "dright": False, "r1": False
        })
        self.releases = self.presses.copy()
        
        # Track stick axes
        self.axes = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}

    def __getitem__(self, key):
        return self.axes.get(key, 0.0)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Crucial: Restore original terminal behavior when stopping the script
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def check_presses(self):
        """Read pending keystrokes out of the terminal input stream buffer."""
        while select.select([sys.stdin], [], [], 0)[0]:
            char = sys.stdin.read(1)
            
            # --- Left Stick: Driving (Sticky Toggles) ---
            if char == 'w': self.axes["ly"] = 1.0
            elif char == 's': self.axes["ly"] = -1.0
            elif char == 'a': self.axes["lx"] = -1.0
            elif char == 'd': self.axes["lx"] = 1.0
            
            # --- Right Stick & D-Pad: Head/Eye & Snake Controls ---
            elif char == 'i': 
                self.axes["ry"] = 1.0
                self.presses["dup"] = True
            elif char == 'k': 
                self.axes["ry"] = -1.0
                self.presses["ddown"] = True
            elif char == 'j': 
                self.axes["rx"] = -1.0
                self.presses["dleft"] = True
            elif char == 'l': 
                self.axes["rx"] = 1.0
                self.presses["dright"] = True
                
            # --- Spacebar: Emergency Stop / Reset Sticks ---
            elif char == ' ':
                self.axes["lx"] = 0.0
                self.axes["ly"] = 0.0
                self.axes["rx"] = 0.0
                self.axes["ry"] = 0.0

            # --- Action Buttons (One-shot pulse per tap) ---
            elif char == 'r': 
                self.presses["r1"] = True
            elif char == 'g': 
                self.presses["l2"] = True; self.presses["r2"] = True 
            elif char == 'm': 
                self.presses["home"] = True
            elif char == 'p': 
                self.presses["select"] = True

    def clear_buffered_states(self):
        """Clears edge-triggered button pulse flags at the end of the loop frame."""
        for k in self.presses:
            self.presses[k] = False
            self.releases[k] = False

    def rumble(self, duration):
        # Printed with carriage return \r to look clean in active streaming terminal logs
        print(f"\r[MOCK RUMBLE] For {duration}s", end="", flush=True)