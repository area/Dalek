import asyncio
import struct
import serial

class ArduinoLedController:
    def __init__(self, vendor_id="1a86", product_id="7523", baudrate=115200):

        target_port = self._find_arduino_port(vendor_id, product_id)
        
        if not target_port:
            raise RuntimeError(
                f"Lighting Hardware Error: No Arduino found with VID:{vendor_id} PID:{product_id}. "
                f"Check the USB cable connection!"
            )
            
        print(f"Successfully matched lighting hardware on device path: {target_port}")
        
        # 2. Connect to the auto-discovered port string
        self.ser = serial.Serial(target_port, baudrate, timeout=1.0)
        self.lock = asyncio.Lock()
        
        self.packet_format = ">7B"  
        self.START_MARKER = 0x12
        self.END_MARKER = 0x13

    def _find_arduino_port(self, vid: str, pid: str) -> str:
        """Scans the USB tree and returns the port name matching the target IDs."""
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            # Check if the port has valid VID/PID tracking structures
            if port.vid is not None and port.pid is not None:
                # Convert numbers to lowercase hex strings matching standard lsusb outputs
                current_vid = f"{port.vid:04x}".lower()
                current_pid = f"{port.pid:04x}".lower()
                
                if current_vid == vid.lower() and current_pid == pid.lower():
                    return port.device  # Returns '/dev/ttyUSB0', '/dev/ttyUSB1', etc.
                    
        return None

    async def _send_packet(self, channel: int, index: int, r: int, g: int, b: int):
        """Low-level method that builds the binary packet and enforces the handshake."""
        async with self.lock:
            packet = struct.pack(
                self.packet_format, 
                self.START_MARKER, 
                channel, 
                index, 
                r, g, b, 
                self.END_MARKER
            )
            
            loop = asyncio.get_running_loop()
            
            # Non-blocking write to hardware buffer via executor threads
            await loop.run_in_executor(None, self.ser.write, packet)
            
            # Loop until the Arduino releases the lock by returning the exact acknowledgement byte
            while True:
                response = await loop.run_in_executor(None, self.ser.read, 1)
                if response == b'A':
                    break
                await asyncio.sleep(0.001)

    async def set_pixel(self, channel: int, index: int, r: int, g: int, b: int):
        """Address a specific LED on a specific channel strip."""
        await self._send_packet(channel, index, r, g, b)

    async def set_strip_color(self, channel: int, r: int, g: int, b: int):
        """Paints an entire strip channel instantly using magic index 255."""
        await self._send_packet(channel, 255, r, g, b)

    async def clear_strip(self, channel: int):
        """Turns off a targeted strip channel."""
        await self._send_packet(channel, 255, 0, 0, 0)

    async def global_broadcast_color(self, r: int, g: int, b: int):
        """Flashes all light strings on the robot to a single uniform color color layout."""
        await self._send_packet(255, 255, r, g, b)

    async def global_blackout(self):
        """Instantly terminates all lights across all connected channels simultaneously."""
        await self._send_packet(255, 255, 0, 0, 0)