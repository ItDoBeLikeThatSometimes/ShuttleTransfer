#include <Wire.h>
#include <Servo.h>
#include <FastLED.h>

// For Stepper Motor
#define STEPPER_PIN_1 9
#define STEPPER_PIN_2 10
#define STEPPER_PIN_3 11
#define STEPPER_PIN_4 12

// For Servo Motor
Servo Servo1;
const int servoPin = 6;
int servoCurrentAngle = 90;      // Current angle of the servo
int servoTargetAngle = 45;       // Target angle of the servo
unsigned long servoLastUpdate = 0;
const unsigned long servoUpdateInterval = 20; // Update servo every 20 ms
const int servoStep = 1;               // Step size for servo movement

// For stepper Game control
static int incrementCounter = 0; // Counts every time the basket is hit.
static int moveBackCounter = 0;   
static int moveBackCounterLimit = 5; // How often the stepper moves back before level 2
static int step_size = 600;          // How far the stepper moves back
int lastStepperMoveCounter = 0;

// For Stepper Motor Control
int step_number = 0;
long stepper_position = 0;       // Current position in steps
const long stepper_max_steps = 1024;   // Number of steps for 180 degrees (adjust based on your stepper motor)

// For LED Strip Control
#define DATA_PIN 5                // Adjusted to avoid conflict with servoPin
#define NUM_LEDS 30               // The actual number of LEDs to control
#define BRIGHTNESS 64
CRGB leds[NUM_LEDS];             // Declare an array for the LEDs

// For Counter Checking
unsigned long lastCounterCheckTime = 0;
const unsigned long counterCheckInterval = 100; // Check every 100 ms
int lastCounterValue = -1;                // Initialize to -1 so that the first read triggers the LED effect

// Game Duration
unsigned long gameStartTime = 0;
unsigned long gameDuration = 60000; // 60 seconds in milliseconds

// State Machine for Game Flow
enum GameState {
  WAIT_FOR_BUTTON,
  TRANSITION,
  RUNNING,
  RUNNING_LEVEL2,
  COMPLETE
};

GameState gameState = WAIT_FOR_BUTTON;

// Button Configuration
const int buttonPin = A0;
bool buttonPressed = false;

// LED Transition Configuration
unsigned long ledTransitionStartTime = 0;
const unsigned long ledTransitionDuration = 3000; // 3 seconds

void setup() {
  Wire.begin();               // Join I2C bus as master
  Serial.begin(9600);         // Start serial communication at 9600 baud rate

  // Instructions for the user
  Serial.println("Enter commands to control the slave device:");
  Serial.println("'R' - Reset counter");
  Serial.println("'S' - Set counter value");
  Serial.println("'T' - Set LDR threshold");
  Serial.println("'G' - Get counter value");
  Serial.println("'P' - Pause counter");
  Serial.println("'C' - Continue counter");

  // Initialize Servo
  Servo1.attach(servoPin);
  Servo1.write(servoCurrentAngle); // Set initial position

  // Initialize Stepper Motor Pins
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);

  // Initialize LED Strip
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // Set LEDs to red initially
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();

  // Seed the random number generator
  randomSeed(analogRead(A1)); // Use an unconnected analog pin for randomness

  // Set initial stepper position to 90 degrees
  stepper_position = map(90, 0, 180, 0, stepper_max_steps); // Map 90 degrees to steps

  // Initialize button pin with internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);

  // Reset game-related variables
  incrementCounter = 0;
  moveBackCounter = 0;
  lastCounterValue = -1;
  gameStartTime = 0;

  // Set the game state to WAIT_FOR_BUTTON
  gameState = WAIT_FOR_BUTTON;


  Serial.print("Current gameState: ");
  Serial.println(gameState);

    // Debug initial button state
  bool initialButtonState = digitalRead(buttonPin);
  if (initialButtonState == LOW) {
    Serial.println("Warning: Button is pressed during setup. Check wiring.");
  } else {
    Serial.println("Button unpressed. Ready to start.");
  }

    Serial.println("Game initialized. Waiting for button press.");

}

void loop() {
  // Handle Serial Commands
  if (Serial.available()) {
    char cmd = Serial.read();
    cmd = toupper(cmd); // Ensure command is uppercase
    switch (cmd) {
      case 'R':
        resetCounter();
        Serial.println("Counter reset.");
        break;
      case 'S': {
        Serial.println("Enter new counter value (0-9999): ");
        while (Serial.available() == 0); // Wait for input
        int newCounter = Serial.parseInt();
        setCounter(newCounter);
        Serial.print("Counter set to ");
        Serial.println(newCounter);
        break;
      }
      case 'T': {
        Serial.println("Enter new LDR threshold (0-1023): ");
        while (Serial.available() == 0); // Wait for input
        int newThreshold = Serial.parseInt();
        setLDRThreshold(newThreshold);
        Serial.print("LDR threshold set to ");
        Serial.println(newThreshold);
        break;
      }
      case 'G': {
        int counterValue = getCounterValue();
        Serial.print("Counter value is ");
        Serial.println(counterValue);
        break;
      }
      case 'P':
        pauseCounter();
        Serial.println("Counter paused.");
        break;
      case 'C':
        continueCounter();
        Serial.println("Counter continued.");
        break;
      default:
        Serial.println("Unknown command. Please enter 'R', 'S', 'T', 'G', 'P', or 'C'.");
        break;
    }
  }

  updateServo();

  // State machine to control game flow
  switch (gameState) {
  case WAIT_FOR_BUTTON: {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50; // Debounce time
  static bool lastButtonState = HIGH;

  bool currentButtonState = digitalRead(buttonPin);

  // Debounce logic
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentButtonState == LOW && !buttonPressed) {
      buttonPressed = true;
      ledTransitionStartTime = millis();
      gameStartLEDEffect();
      gameState = TRANSITION;
      Serial.println("Button pressed. Starting game.");
    }
  }

  if (currentButtonState == HIGH && buttonPressed) {
    buttonPressed = false; // Reset button state after release
  }

  lastButtonState = currentButtonState;
  break;
}

    case TRANSITION: {
      //gameStartLEDEffect();
      unsigned long elapsedTime = millis() - ledTransitionStartTime;
      if (elapsedTime >= ledTransitionDuration) {
        // Transition complete
        //gameStartLEDEffect();

        moveBackCounter = 0;
        incrementCounter = 0;
        // Send command to slave to start the counter
        resetCounter(); // Reset counter to 0
        continueCounter(); // Start the counter
        Serial.println("LED is green. Counter started.");
        lastCounterValue = -1; // Reset counter tracking
        gameStartTime = millis(); // Start the game timer
        gameState = RUNNING;
      }
      break;
    }

    case RUNNING:
      // Check if game duration has been exceeded
      if (millis() - gameStartTime >= gameDuration) {
        gameOver(); // Reset light and counter
      } else {
        // Check the counter value from the slave at regular intervals
        if (millis() - lastCounterCheckTime >= counterCheckInterval) {
          lastCounterCheckTime = millis();
          int currentCounterValue = getCounterValue();
          Serial.print("Counter value: ");
          Serial.println(currentCounterValue);
          if (currentCounterValue > lastCounterValue) {
            // Counter has increased, run LED effect and adjust servo and stepper randomly
            incrementCounter++;
            runLEDEffect();

            // Randomly adjust the servo motor within 0-180 degrees
            servoTargetAngle = random(40, 140); // 0 to 180 inclusive
            
            Serial.print("Servo target angle set to ");
            Serial.print(servoTargetAngle);
            Serial.println(" degrees.");

            // move the stepper back every third time the counter increases
            // start level 2 if the stepper has been moved back 5 times
            if (moveBackCounter >= moveBackCounterLimit) {
              returnStepperToStart();
              gameState = RUNNING_LEVEL2;
              // Increase game time with 30 seconds
              gameDuration += 30000;
              break;
            }
            
            if (incrementCounter % 3 == 0) {  
              // Move stepper motor step_size steps forward
              long target_stepper_position = stepper_position + step_size;

              // Calculate steps to move
              long steps_to_move = target_stepper_position - stepper_position;

              // Move the stepper motor forward
              MoveStepper(steps_to_move, true);

              // Update stepper_position
              stepper_position = target_stepper_position;

              lastStepperMoveCounter = incrementCounter;

              // Update moveBackCounter
              moveBackCounter++;

              Serial.print("Stepper motor moved by ");
              Serial.print(step_size);
              Serial.println(" steps.");
            }
            lastCounterValue = currentCounterValue;
          }
        }
      }
      break;

    case RUNNING_LEVEL2:
      // Check if game duration has been exceeded
      if (millis() - gameStartTime >= gameDuration) {
        gameOver();
      } else {
        // Move the servo back and forth continuously
        moveServoContinuously();
        if (millis() - lastCounterCheckTime >= counterCheckInterval) {
          lastCounterCheckTime = millis();
          int currentCounterValue = getCounterValue();
          Serial.print("Counter value: ");
          Serial.println(currentCounterValue);
          if (currentCounterValue > lastCounterValue) {
            // Counter has increased, run LED effect
            incrementCounter++;
            runLEDEffect();

          // Move the stepper back every third time the counter increases
          if (incrementCounter % 3 == 0 && incrementCounter != lastStepperMoveCounter) {  
            // Move stepper motor 300 steps forward
            const long step_size = 600;
            long target_stepper_position = stepper_position + step_size;

            // Calculate steps to move
            long steps_to_move = target_stepper_position - stepper_position;

            // Move the stepper motor forward
            MoveStepper(steps_to_move, true);

            // Update stepper_position
            stepper_position = target_stepper_position;

            // Update moveBackCounter
            moveBackCounter++;

            lastStepperMoveCounter = incrementCounter;

            Serial.print("Stepper motor moved by ");
            Serial.print(step_size);
            Serial.println(" steps.");
            Serial.println("MOVE BACK COUNTER: " );
            Serial.println(moveBackCounter);

            if(moveBackCounter >= 5) {
              Serial.println("Move back counter reached 5. Ending game.");
              gameOver(); // End the game and move the stepper back
            }

          }
          lastCounterValue = currentCounterValue;
        }
      }
      }
      break;

    case COMPLETE:
      // Wait for next button press to restart the game
      if (digitalRead(buttonPin) == LOW && !buttonPressed) {
        buttonPressed = true;
        ledTransitionStartTime = millis();
        gameState = TRANSITION;
        Serial.println("Button pressed. Starting LED transition.");
      }
      if (digitalRead(buttonPin) == HIGH && buttonPressed) {
        // Button released
        buttonPressed = false;
      }
      // Return stepper motor to starting position
      if (stepper_position != 0) {
        returnStepperToStart();
      }
      // Return servo to Center
      servoTargetAngle = 90;
      updateServo();
      break;
  }
}

// Continue fixing further as required by ensuring you repeat consistent closing braces. This debugged template layout should now work.


void gameOver() {
  Serial.println("Game time over.");
  // Change LED back to red
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  pauseCounter(); // Pause the counter

  if(stepper_position != 0){
    returnStepperToStart();
  }

  moveBackCounter = 0;

  gameState = COMPLETE;
}

void returnStepperToStart(){
  long steps_to_return = stepper_position; // Current position is the number of steps from the start
  MoveStepper(steps_to_return, false);    // Move backward to starting position
  stepper_position = 0;                   // Reset stepper position to 0
  moveBackCounter = 0;
  Serial.println("Stepper motor returned to starting position.");
}

void moveServoContinuously() {
  // Only alternate if the servo has reached the target angle
  if (servoCurrentAngle == servoTargetAngle) {
    if (gameState == RUNNING_LEVEL2) {
      // Alternate between 0° and 90°
      if (servoTargetAngle == 135) {
        servoTargetAngle = 45; // Switch to 0 degrees
      } else {
        servoTargetAngle = 135; // Switch to 180 degrees
      }
    }
  }
}
// Function to gradually update the servo position
void updateServo() {
  unsigned long currentMillis = millis();
  if (currentMillis - servoLastUpdate >= servoUpdateInterval) {
    servoLastUpdate = currentMillis;
    if (servoCurrentAngle < servoTargetAngle) {
      servoCurrentAngle += servoStep;
      if (servoCurrentAngle > servoTargetAngle) {
        servoCurrentAngle = servoTargetAngle;
      }
      Servo1.write(servoCurrentAngle);
    } else if (servoCurrentAngle > servoTargetAngle) {
      servoCurrentAngle -= servoStep;
      if (servoCurrentAngle < servoTargetAngle) {
        servoCurrentAngle = servoTargetAngle;
      }
      Servo1.write(servoCurrentAngle);
    }
  }
}


// I2C Communication Functions
void resetCounter() {
  Wire.beginTransmission(4); // Address of slave
  Wire.write('R');
  Wire.endTransmission();
}

void setCounter(int value) {
  Wire.beginTransmission(4);
  Wire.write('S');
  Wire.write((value >> 8) & 0xFF); // High byte
  Wire.write(value & 0xFF);        // Low byte
  Wire.endTransmission();
}

void setLDRThreshold(int threshold) {
  Wire.beginTransmission(4);
  Wire.write('T');
  Wire.write((threshold >> 8) & 0xFF); // High byte
  Wire.write(threshold & 0xFF);        // Low byte
  Wire.endTransmission();
}

int getCounterValue() {
  int value = -1;
  Wire.beginTransmission(4);
  Wire.write('G');
  Wire.endTransmission();

  Wire.requestFrom(4, 2); // Request 2 bytes from slave
  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    value = (highByte << 8) | lowByte;
  }
  return value;
}

void pauseCounter() {
  Wire.beginTransmission(4);
  Wire.write('P');
  Wire.endTransmission();
}

void continueCounter() {
  Wire.beginTransmission(4);
  Wire.write('C');
  Wire.endTransmission();
}

// Function to Control One Step of the Stepper Motor
void OneStep(bool dir) {
  if (!dir) {
    switch (step_number) {
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    }
  } else {
    switch (step_number) {
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
    }
  }

  step_number++;
  if (step_number > 3) {
    step_number = 0;
  }
}

// Function to move the stepper motor a certain number of steps in a given direction
void MoveStepper(long steps, bool direction) {
  for (long i = 0; i < steps; i++) {
    OneStep(direction);
    delay(2); // Adjust delay for speed
  }
}

// ----------- LED EFFECTS -----------------

void runLEDEffect() {
  // Only run the effect if gameState is RUNNING
  if (gameState == RUNNING || gameState == RUNNING_LEVEL2) {
    // Light different colors for 1000 ms
    unsigned long effectStartTime = millis();
    unsigned long effectDuration = 1000; // 1000 ms
    while (millis() - effectStartTime < effectDuration) {
      fill_solid(leds, NUM_LEDS, CHSV(random(0, 255), 255, 255)); // Random color
      FastLED.show();
      delay(100); // Update color every 100 ms
    }
    // After effect, set LEDs back to green
    fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.show();
  }
}

// Function to run LED effect when counter increases
void blueLEDEffect() {
  // Only run the effect if gameState is RUNNING
  if (gameState == RUNNING) {
    // Light different colors for 1000 ms
    unsigned long effectStartTime = millis();
    unsigned long effectDuration = 1000; // 1000 ms
    while (millis() - effectStartTime < effectDuration) {
      fill_solid(leds, NUM_LEDS, CHSV(random(160, 200), 255, 255)); // Random blue shades
      FastLED.show();
      delay(100); // Update color every 100 ms
    }
    // After effect, set LEDs back to green
    fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.show();
  }
}

void blueLEDScene() {
  static uint8_t hue = 160; // Start hue (blue region)
  static int8_t hueDirection = 1; // Direction to change hue
  static uint8_t brightness = 255; // Full brightness

  // Loop to fade across different blue hues
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(hue, 255, brightness); // Hue varies, max saturation, max brightness
  }
  FastLED.show();

  delay(50); // Adjust for fade speed

  // Change hue to cycle through shades of blue
  hue += hueDirection;
  if (hue >= 200 || hue <= 160) { // Stay in blue range (160-200)
    hueDirection = -hueDirection; // Reverse direction
  }
}


void gameStartLEDEffect() {
  const unsigned long redDuration = 500;    // 1 second for red
  const unsigned long yellowDuration = 1000; // 1 second for yellow
  const unsigned long greenDuration = 1000; // 1 second for green
  const unsigned long offDuration = 100;    // 100 ms for LEDs off between lights

  // Show Red
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(redDuration);

  // Turn Off LEDs for 100 ms
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Set all LEDs to off
  FastLED.show();                         // Update LED state
  delay(offDuration);

  // Show Yellow
  fill_solid(leds, NUM_LEDS, CRGB::Yellow);
  FastLED.show();
  delay(yellowDuration);

  // Turn Off LEDs for 100 ms
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Set all LEDs to off
  FastLED.show();                         // Update LED state
  delay(offDuration);

  // Show Green
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
  delay(greenDuration);

  // Turn Off LEDs for 100 ms before ending
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Set all LEDs to off
  FastLED.show();                         // Update LED state
  delay(offDuration);

  // Ensure LEDs end in solid green
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
}

