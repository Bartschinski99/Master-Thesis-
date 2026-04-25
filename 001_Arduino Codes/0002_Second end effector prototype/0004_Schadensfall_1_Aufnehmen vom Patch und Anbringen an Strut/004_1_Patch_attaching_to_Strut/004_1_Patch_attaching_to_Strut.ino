// =============================================
//  Project:  Masterthesis
//  Author:   Lukas Bartschinski
//  Date:     26.03.2026
//  Description:
//  This sketch controls two VL53L0X distance sensors, one stepper motor, and one servo motor
// =============================================

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

// =============================================
//               Pin Definitions
// =============================================

// XSHUT pins for VL53L0X sensors
const int sensor1XSHUT = 8;
const int sensor2XSHUT = 9;

// Limit switch pins
const int taster1Pin = 29;  // Limit switch Nr. 1 Joint
const int taster2Pin = 31;  // Limit switch Nr. 2 Joint
const int taster3Pin = 22;  // Limit switch Nr. 3 Patch
const int taster4Pin = 35;  // Limit switch Nr. 4 (Backup)

// Stepper motor control pins
#define STEP_PIN 2    // Step signal (PWR on TB6600)
#define DIR_PIN 3     // Direction signal (PUL on TB6600)
#define ENA_PIN 4     // Enable signal (ENBL on TB6600)

// NEMA 17 Stepper Motor settings: 1.46A (ON;ON;OFF) / 12V

// Servo motor pin
#define SERVO_PIN 5   // Servo motor pin

// =============================================
//               Constants
// =============================================

// VOF sensor offsets
const int SENSOR_OFFSET1_STRUT = 77;  // Offset for Strut alignment (mm)
const int SENSOR_OFFSET2_STRUT = 67;  // Offset for Strut alignment (mm)
const int SENSOR_OFFSET1_PATCH = 77;  // Offset for Patch alignment (mm)
const int SENSOR_OFFSET2_PATCH = 67;  // Offset for Patch alignment (mm)

// Stepper motor --> step delay in microseconds
const int STEP_DELAY_US_INITIAL = 500; // 500 µs delay for initial steps

// Servo motor angles
const int SERVO_PICKUP_ANGLE = 100;   // Angle for patch pickup
const int SERVO_RELEASE_ANGLE = 0;   // Angle for patch release

// Other
const unsigned long PRINT_DELAY = 1000; // Delay for serial output in ms

// =============================================
//               Global Variables
// =============================================

// VOF sensor objects
VL53L0X sensor1;
VL53L0X sensor2;

// Servo motor object
Servo patchServo;

// Process state tracking
enum ProcessState {
  WAIT_FOR_PATCH_ALIGNMENT,
  PATCH_PICKUP,
  WAIT_FOR_STRUT_ALIGNMENT,
  MOVE_MOTOR,
  WAIT_DELAY,
  PATCH_ATTACH,
  MOVE_MOTOR_BACKWARD
};
ProcessState currentState = WAIT_FOR_PATCH_ALIGNMENT;

// Variables for delay and step counting
unsigned long delayStartTime = 0;
unsigned long stepCounter = 0;
const unsigned long MOTOR_STEPS = 12000; // Steps after alignment

// Global state variables
bool patchPickedUp = false;
bool patchAttached = false;

// =============================================
//  SETUP: Initialize hardware and sensors
// =============================================

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I²C bus

  // Set XSHUT pins as outputs
  pinMode(sensor1XSHUT, OUTPUT);
  pinMode(sensor2XSHUT, OUTPUT);

  // Reset both VL53L0X sensors
  digitalWrite(sensor1XSHUT, LOW);
  digitalWrite(sensor2XSHUT, LOW);
  delay(10);

  // Start VL53L0X sensor 1 and set address
  digitalWrite(sensor1XSHUT, HIGH);
  delay(10);
  sensor1.init(true);
  sensor1.setAddress(0x30);  // New address for sensor 1
  delay(10);

  // Start VL53L0X sensor 2 (default address 0x29)
  digitalWrite(sensor2XSHUT, HIGH);
  delay(10);
  sensor2.init(true);
  sensor2.setAddress(0x29);  // Default address for sensor 2
  delay(10);

  // Start continuous measurement
  sensor1.startContinuous(50);
  sensor2.startContinuous(50);

  // Initialize limit switches
  pinMode(taster1Pin, INPUT_PULLUP);
  pinMode(taster2Pin, INPUT_PULLUP);
  pinMode(taster3Pin, INPUT_PULLUP);
  pinMode(taster4Pin, INPUT_PULLUP);

  // Initialize stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  // Initialize servo motor
  patchServo.attach(SERVO_PIN);
  patchServo.write(SERVO_RELEASE_ANGLE);  // Servo in initial position

  Serial.println("EE system ready. Waiting for patch alignment");
}

// =============================================
//  LOOP: Main program loop
// =============================================

void loop() {
  switch (currentState) {
    case WAIT_FOR_PATCH_ALIGNMENT:
      waitForPatchAlignment();
      break;
    case PATCH_PICKUP:
      patchPickup();
      break;
    case WAIT_FOR_STRUT_ALIGNMENT:
      waitForStrutAlignment();
      break;
    case MOVE_MOTOR:
      moveMotor();
      break;
    case WAIT_DELAY:
      waitDelay();
      break;
    case PATCH_ATTACH:
      patchAttach();
      break;
    case MOVE_MOTOR_BACKWARD:
      moveMotorBackward();
      break;
  }
}

// ========================================================================================
//  Function --> waitForPatchAlignment: Wait for correct patch alignment using VL53L0X sensors
// ========================================================================================

void waitForPatchAlignment() {
  static unsigned long lastDistancePrint = 0;

  if (millis() - lastDistancePrint >= PRINT_DELAY) {
    int distance1 = sensor1.readRangeContinuousMillimeters();
    int distance2 = sensor2.readRangeContinuousMillimeters();
    int adjustedDistance1 = distance1 - SENSOR_OFFSET1_PATCH;
    int adjustedDistance2 = distance2 - SENSOR_OFFSET2_PATCH;

    if (sensor1.timeoutOccurred() || sensor2.timeoutOccurred()) {
      Serial.println("Error: VL53L0X-Sensor - Timeout");
    } else {
      Serial.print("Distance to Patch (S1): ");
      Serial.print(adjustedDistance1);
      Serial.print(" mm | Distance to Patch (S2): ");
      Serial.print(adjustedDistance2);
      Serial.println(" mm");

      // Check alignment
      bool aligned = abs(adjustedDistance1 - adjustedDistance2) <= 3;
      Serial.print("Patch Alignment: ");
      Serial.println(aligned ? "Aligned" : "Not Aligned");

      // Check distance alignment
      bool correctDistance = (adjustedDistance1 < 10 && adjustedDistance1 > 0) &&
                             (adjustedDistance2 < 10 && adjustedDistance2 > 0);
      Serial.print("Patch Distance: ");
      Serial.println(correctDistance ? "Correct Distance" : "Error Distance");

      if (aligned && correctDistance) {
        Serial.println("Patch is correctly aligned AND within the correct distance. Ready for pickup...");
      }
    }
    lastDistancePrint = millis();
  }

  // Proceed to pickup if patch is aligned and endstop 3 is pressed
  if (digitalRead(taster3Pin) == LOW) {
    currentState = PATCH_PICKUP;
  }
}

// ========================================================================================
//  Function --> patchPickup: Rotate servo to pickup the patch
// ========================================================================================

void patchPickup() {
  if (!patchPickedUp) {
    Serial.println("Patch in the correct position. Locking started");
    delay(5000); // Delay after endstop activation: wait 5 seconds
    patchServo.write(SERVO_PICKUP_ANGLE);  // Servo rotates to pickup angle
    Serial.println("Patch successfully connected.");
    patchPickedUp = true;
    delay(3000); // Delay after pickup: wait 3 seconds
  }

  if (patchPickedUp) {
    currentState = WAIT_FOR_STRUT_ALIGNMENT;
    stepCounter = 0;
  }
}

// ===========================================================================
//  waitForStrutAlignment: Wait for correct strut alignment and distance
// ===========================================================================

void waitForStrutAlignment() {
  static unsigned long lastDistancePrint = 0;

  if (millis() - lastDistancePrint >= PRINT_DELAY) {
    int distance1 = sensor1.readRangeContinuousMillimeters();
    int distance2 = sensor2.readRangeContinuousMillimeters();
    int adjustedDistance1 = distance1 - SENSOR_OFFSET1_STRUT;
    int adjustedDistance2 = distance2 - SENSOR_OFFSET2_STRUT;

    if (sensor1.timeoutOccurred() || sensor2.timeoutOccurred()) {
      Serial.println("Error: VL53L0X-Sensor - Timeout");
    } else {
      Serial.print("Distance to Strut (S1): ");
      Serial.print(adjustedDistance1);
      Serial.print(" mm | Distance to Strut (S2): ");
      Serial.print(adjustedDistance2);
      Serial.println(" mm");

      bool aligned = abs(adjustedDistance1 - adjustedDistance2) <= 8;
      Serial.print("Strut Alignment: ");
      Serial.println(aligned ? "Aligned" : "Not Aligned");

      bool correctDistance = (adjustedDistance1 < 20 && adjustedDistance1 > 0) &&
                             (adjustedDistance2 < 20 && adjustedDistance2 > 0);
      Serial.print("Strut Distance: ");
      Serial.println(correctDistance ? "Correct Distance" : "Error Distance");

      if (aligned && correctDistance) {
        Serial.println("Strut is correctly aligned AND within the correct distance. Motor ready for starting...");
        currentState = MOVE_MOTOR;
        stepCounter = 0;
        digitalWrite(ENA_PIN, HIGH);  // Enable motor
        digitalWrite(DIR_PIN, HIGH);  // Direction: clockwise
      }
    }
    lastDistancePrint = millis();
  }

  // Backup for the alignment and distance test with endstop 1 and 2
  int taster1Zustand = digitalRead(taster1Pin);
  int taster2Zustand = digitalRead(taster2Pin);

  if (taster1Zustand == LOW && taster2Zustand == LOW) {
    Serial.println("Strut is correctly aligned AND within the correct distance (with endstop 1 & 2). Motor ready for starting...");
    currentState = MOVE_MOTOR;
    stepCounter = 0;
    digitalWrite(ENA_PIN, HIGH);  // Enable motor
    digitalWrite(DIR_PIN, HIGH);  // Direction: clockwise
  }
}

// ============================================================================
//  moveMotor: Move stepper motor 12000 steps
// ============================================================================

void moveMotor() {
  static unsigned long lastStepPrint = 0;

  if (stepCounter < MOTOR_STEPS) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    stepCounter++;

    if (millis() - lastStepPrint >= PRINT_DELAY) {
      Serial.print("Step ");
      Serial.print(stepCounter);
      Serial.print(" of ");
      Serial.println(MOTOR_STEPS);
      lastStepPrint = millis();
    }
  } else {
    Serial.println("Set steps reached.");
    delayStartTime = millis();    // Save delay start time
    currentState = PATCH_ATTACH;
  }
}

// ==============================================================================================================
//  waitDelay: Wait for 60 seconds
// ==============================================================================================================

void waitDelay() {
  static unsigned long lastDelayPrint = 0;

  if (millis() - lastDelayPrint >= PRINT_DELAY) {
    unsigned long remainingTime = (60000 - (millis() - delayStartTime)) / 1000;
    Serial.print("Waiting: ");
    Serial.print(remainingTime);
    Serial.println(" seconds remaining");
    lastDelayPrint = millis();
  }

  if (millis() - delayStartTime >= 60000) {
    currentState = MOVE_MOTOR_BACKWARD;
  }
}

// =====================================================================
//  patchAttach: Attach the patch to the strut by releasing the servo
// =====================================================================

void patchAttach() {
  if (!patchAttached) {
    Serial.println("Patch attached to strut.");
    delay(5000); // Delay of 5000 milliseconds
    patchServo.write(SERVO_RELEASE_ANGLE);  // Servo back to initial position
    patchAttached = true;
    delayStartTime = millis();  // Save delay start time
  }

  if (millis() - delayStartTime >= 5000) {
    currentState = MOVE_MOTOR_BACKWARD;
  }
}

// ============================================================================
//  moveMotorBackward: Move motor 12000 steps backward to release strut + patch
// ============================================================================

void moveMotorBackward() {
  static unsigned long lastBackwardPrint = 0;
  static int backwardSteps = 0;

  if (backwardSteps == 0) {
    Serial.println("Release of Strut and Patch");
    digitalWrite(ENA_PIN, HIGH);  // Enable motor
    digitalWrite(DIR_PIN, LOW);   // Direction: counterclockwise
  }

  if (backwardSteps < 12000) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    backwardSteps++;

    if (millis() - lastBackwardPrint >= PRINT_DELAY) {
      Serial.print("Backward step ");
      Serial.print(backwardSteps);
      Serial.println(" of 12000");
      lastBackwardPrint = millis();
    }
  } else {
    digitalWrite(ENA_PIN, LOW);  // Disable motor
    currentState = WAIT_FOR_PATCH_ALIGNMENT;
    backwardSteps = 0;
    stepCounter = 0;
    patchPickedUp = false;
    patchAttached = false;
    Serial.println("Strut with Patch successfully released. EE system ready for new task");
  }
}
