#include <Adafruit_NeoPixel.h>

// Multi-string hardware configuration
#define STRIP_COUNT 2
const int STRIP_PINS[STRIP_COUNT]    = {6, 7};   // Channel 0 = Pin 6, Channel 1 = Pin 7
const int STRIP_LENGTHS[STRIP_COUNT] = {56, 16};  // Channel 0 has 56 LEDs, Channel 1 has 16 LEDs

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
  
  // Fire initial handshake
  Serial.write(0x41); 
}

// Low-level buffer writer (Updates memory array only - does not call .show())
void updateStripColor(int channel, byte index, byte r, byte g, byte b) {
  if (channel >= STRIP_COUNT) return; 

  if (index == 255) {
    // Magic Index 255: Clear/Tint entire memory buffer
    for (int i = 0; i < STRIP_LENGTHS[channel]; i++) {
      strips[channel].setPixelColor(i, strips[channel].Color(r, g, b));
    }
  } else if (index < STRIP_LENGTHS[channel]) {
    // Standard Index: Update single pixel in memory
    strips[channel].setPixelColor(index, strips[channel].Color(r, g, b));
  }
}

void loop() {
  if (Serial.available() >= 7) {
    if (Serial.read() == START_MARKER) {
      
      Serial.readBytes(packetBuffer, 5);
      
      if (Serial.read() == END_MARKER) {
        byte channel = packetBuffer[0];
        byte index   = packetBuffer[1];
        byte r       = packetBuffer[2];
        byte g       = packetBuffer[3];
        byte b       = packetBuffer[4];
        
        // 1. Process Data
        if (channel == 255) {
          // GLOBAL BROADCAST
          for (int c = 0; c < STRIP_COUNT; c++) {
            if (index == 254) {
              strips[c].show(); // Global Latch
            } else {
              updateStripColor(c, index, r, g, b);
            }
          }
        } else {
          // DIRECT ROUTING
          if (index == 254) {
            // MAGIC INDEX 254: Force Hardware Render
            strips[channel].show();
          } else {
            // Normal update
            updateStripColor(channel, index, r, g, b);
          }
        }
      }
      
      // Acknowledge after every packet processed
      Serial.write(0x41); 
    }
  }
}