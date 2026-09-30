

/*
  ================================
  Multi-Button + NeoPixel Task Code
  ================================

  FUNCTION:
  - Controls 4 buttons and 4 individual NeoPixels.
  - Receives commands from Python via Serial.
  - Lights LEDs according to the current trial pattern (Go-Blue, Stop-Red, Only-Blue, Only-Red).
  - Detects button presses and reports them back to Python.
  - When a button is pressed during an active trial, all LEDs turn off immediately.
  - Debounce added to prevent electrical noise from triggering false presses.

  CONNECTIONS:
  Buttons: Digital pins 2, 3, 4, 5 (wired to GND, using INPUT_PULLUP)
  NeoPixels: Digital pins 6, 7, 8, 9 (1 LED each)
*/

#include <Adafruit_NeoPixel.h>

// ----- CONFIGURATION -----
#define NUM_BUTTONS 4
#define DEBOUNCE_MS 50  // Milliseconds a button must be held LOW to count as a real press

const uint8_t buttonPins[NUM_BUTTONS] = {2, 3, 4, 5};
const uint8_t ledPins[NUM_BUTTONS] = {6, 7, 8, 9};

Adafruit_NeoPixel leds[NUM_BUTTONS] = {
  Adafruit_NeoPixel(1, ledPins[0], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(1, ledPins[1], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(1, ledPins[2], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(1, ledPins[3], NEO_GRB + NEO_KHZ800)
};

// Track LED color state: -1 = off, 0 = red, 1 = blue
int activeColor[NUM_BUTTONS] = {-1, -1, -1, -1};

// Debounce tracking per button
bool buttonPressed[NUM_BUTTONS] = {false, false, false, false};
unsigned long pressStartTime[NUM_BUTTONS] = {0, 0, 0, 0};
bool buttonArmed[NUM_BUTTONS] = {false, false, false, false};  // True when LOW first detected

// ----- SETUP -----
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    leds[i].begin();
    leds[i].show();
  }
}

uint32_t colorRGB(uint8_t r, uint8_t g, uint8_t b) {
  return leds[0].Color(r, g, b);
}

void clearAll() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    leds[i].setPixelColor(0, 0);
    leds[i].show();
    activeColor[i] = -1;
  }
}

void setLED(int index, uint32_t color) {
  leds[index].setPixelColor(0, color);
  leds[index].show();
  activeColor[index] = (color == colorRGB(255, 0, 0)) ? 0 : 1;
}

// ----- MAIN LOOP -----
void loop() {

  // ===============================
  // Handle incoming serial commands
  // ===============================
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("GO_BLUE")) {
      int idx = line.substring(8).toInt();
      clearAll();
      setLED(idx, colorRGB(0, 0, 255));
      setLED(idx, colorRGB(0, 0, 255));
      Serial.print("LIT ");
      Serial.println(idx);
    }

    else if (line.startsWith("STOP_RED")) {
      int idx = line.substring(9).toInt();
      clearAll();
      setLED(idx, colorRGB(255, 0, 0));
      setLED(idx, colorRGB(255, 0, 0));
      Serial.print("LIT ");
      Serial.println(idx);
    }

    else if (line.startsWith("ONLY_BLUE")) {
      clearAll();
      int nums[NUM_BUTTONS];
      int start = 10;
      for (int i = 0; i < NUM_BUTTONS; i++) {
        int end = line.indexOf(',', start);
        if (end == -1) end = line.length();
        nums[i] = line.substring(start, end).toInt();
        start = end + 1;
      }
      for (int i = 0; i < NUM_BUTTONS; i++) {
        if (i == NUM_BUTTONS - 1)
          setLED(nums[i], colorRGB(0, 0, 255));
        else
          setLED(nums[i], colorRGB(255, 0, 0));
        Serial.print("LIT ");
        Serial.println(nums[i]);
      }
    }

    else if (line.startsWith("ONLY_RED")) {
      clearAll();
      int nums[NUM_BUTTONS];
      int start = 9;
      for (int i = 0; i < NUM_BUTTONS; i++) {
        int end = line.indexOf(',', start);
        if (end == -1) end = line.length();
        nums[i] = line.substring(start, end).toInt();
        start = end + 1;
      }
      for (int i = 0; i < NUM_BUTTONS; i++) {
        if (i == NUM_BUTTONS - 1)
          setLED(nums[i], colorRGB(255, 0, 0));
        else
          setLED(nums[i], colorRGB(0, 0, 255));
        Serial.print("LIT ");
        Serial.println(nums[i]);
      }
    }

    else if (line == "ALL_OFF" || line == "CLEAR") {
      clearAll();
    }
  }

  // ===============================
  // Monitor button presses (with debounce)
  // ===============================
  unsigned long now = millis();

  for (int i = 0; i < NUM_BUTTONS; i++) {
    int state = digitalRead(buttonPins[i]);

    if (state == LOW) {
      if (!buttonArmed[i]) {
        // First time we see this button go LOW — start the debounce timer
        buttonArmed[i] = true;
        pressStartTime[i] = now;
      } else if (!buttonPressed[i] && (now - pressStartTime[i]) >= DEBOUNCE_MS) {
        // Button has been held LOW for the full debounce period — it's a real press
        buttonPressed[i] = true;

        Serial.print("PRESSED ");
        Serial.println(i);

        // Turn off all LEDs immediately on confirmed press
        bool anyActive = false;
        for (int j = 0; j < NUM_BUTTONS; j++) {
          if (activeColor[j] != -1) { anyActive = true; break; }
        }
        if (anyActive) clearAll();
      }
    } else {
      // Button released — reset everything for this button
      buttonArmed[i] = false;
      buttonPressed[i] = false;
    }
  }
}
