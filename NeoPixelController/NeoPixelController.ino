#include <Adafruit_NeoPixel.h>

// Multi-string hardware configuration
#define STRIP_COUNT 2
const int STRIP_PINS[STRIP_COUNT]   = {6, 7};   // Channel 0 = Pin 6, Channel 1 = Pin 7
const int STRIP_LENGTHS[STRIP_COUNT] = {16, 8};  // Channel 0 has 16 LEDs, Channel 1 has 8 LEDs

Adafruit_NeoPixel strips[STRIP_COUNT];

const byte START_MARKER = 0x12;
const byte END_MARKER   = 0x13;
byte packetBuffer[5]; // Storage for payload: Channel, Index, R, G, B

void setup() {
  Serial.begin(115200);
  
  // Dynamically setup all light arrays
  for (int i = 0; i < STRIP_COUNT; i++) {
    strips[i] = Adafruit_NeoPixel(STRIP_LENGTHS[i], STRIP_PINS[i], NEO_GRB + NEO_KHZ800);
    strips[i].begin();
    strips[i].show(); // Default to off
  }
  
  // Fire initial handshake to clear the Raspberry Pi's boot barrier
  Serial.write(0x41); // ASCII 'A'
}

// Low-level renderer that safely targets individual strings
void updateStripColor(int channel, byte index, byte r, byte g, byte b) {
  if (channel >= STRIP_COUNT) return; // Drop commands targeting non-existent hardware

  if (index == 255) {
    // Magic Index 255: Paint the entire strip immediately
    for (int i = 0; i < STRIP_LENGTHS[channel]; i++) {
      strips[channel].setPixelColor(i, strips[channel].Color(r, g, b));
    }
  } else if (index < STRIP_LENGTHS[channel]) {
    // Standard Index: Target an isolated single pixel
    strips[channel].setPixelColor(index, strips[channel].Color(r, g, b));
  }
  strips[channel].show();
}

void loop() {
  // Wait until a perfect 7-byte packet block fills the serial buffer
  if (Serial.available() >= 7) {
    if (Serial.read() == START_MARKER) {
      
      // Extract payload parameters
      Serial.readBytes(packetBuffer, 5);
      
      if (Serial.read() == END_MARKER) {
        byte channel = packetBuffer[0];
        byte index   = packetBuffer[1];
        byte r       = packetBuffer[2];
        byte g       = packetBuffer[3];
        byte b       = packetBuffer[4];
        
        if (channel == 255) {
          // GLOBAL BROADCAST: Apply to every configured string
          for (int c = 0; c < STRIP_COUNT; c++) {
            updateStripColor(c, index, r, g, b);
          }
        } else {
          // DIRECT ROUTING: Target the specific string channel
          updateStripColor(channel, index, r, g, b);
        }
      }
      
      // Handshake complete: Authorize the Pi to send the next block
      Serial.write(0x41); 
    }
  }
}