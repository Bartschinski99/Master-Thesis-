// ==========================================================================================
//  Project:  Masterthesis
//  Author:   Lukas Bartschinski
//  Date:     26.03.2026
//  Description:
//  This sketch controls two VL53L0X distance sensors, two Endstops and a stepper motor
// ==========================================================================================

#include <Wire.h>
#include <VL53L0X.h>

// =============================================
//               Pin Definitions
// =============================================

// XSHUT pins for VL53L0X sensors
const int sensor1XSHUT = 8;
const int sensor2XSHUT = 9;

// Limit switch pins
const int taster1Pin = 29;  // Limit switch Nr. 1 Joint
const int taster2Pin = 31;  // Limit switch Nr. 2 Joint
const int taster4Pin = 22;  // Limit switch Nr. 3 Patch

// Stepper motor control pins
#define STEP_PIN 2    // Step signal (PWR on TB6600)
#define DIR_PIN 3     // Direction signal (PUL on TB6600)
#define ENA_PIN 4     // Enable signal (ENBL on TB6600)

// NEMA 17 Stepper Motor settings: 1.46 (ON;ON;OFF) / 12V  
// =============================================
//               Constants
// =============================================

// VOF sensor
const int SENSOR_OFFSET1 = 77; //measured Offset in mm
const int SENSOR_OFFSET2 = 67; //measured Offset in mm
const int VOF_THRESHOLD = 10;  // Threshold for VOF distance in mm

// Stepper motor --> step delay in microseconds
const int STEP_DELAY_US_INITIAL = 500; // 500 µs delay for initial steps

// Other
const unsigned long PRINT_DELAY = 1000; // Delay for serial output in ms

// =============================================
//               Global Variables
// =============================================

// VOF sensor objects
VL53L0X sensor1;
VL53L0X sensor2;

// Process state tracking
enum ProcessState { WAIT_FOR_ALIGNMENT, MOVE_MOTOR, CONFIRM_CONNECTION, WAIT_DELAY, MOVE_MOTOR_BACKWARD };
ProcessState currentState = WAIT_FOR_ALIGNMENT;

// Variables for delay and step counting
unsigned long delayStartTime = 0;
unsigned long stepCounter = 0;
const unsigned long MOTOR_STEPS = 9800; // Stepper Motor steps after alignment

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

  // Normally both sensors have address 0x29 so one address must be changed to 0x30
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
  pinMode(taster4Pin, INPUT_PULLUP);

  // Initialize stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  Serial.println("EE system ready. Waiting for correct alignment with Joint");
}

// =============================================
//  LOOP: Main program loop
// =============================================

void loop() {
  switch (currentState) {
    case WAIT_FOR_ALIGNMENT:
      waitForAlignment();
      break;
    case MOVE_MOTOR:
      moveMotor();
      break;
    case CONFIRM_CONNECTION:
      confirmConnection();
      break;
    case WAIT_DELAY:
      waitDelay();
      break;
    case MOVE_MOTOR_BACKWARD:
      moveMotorBackward();
      break;
  }
}

// ============================================================================================================================================================
//  Function --> waitForAlignment: Wait for correct alignment with the joint or the switch press the connection of endstops 1 and 2 to start the stepper motor
// ============================================================================================================================================================

void waitForAlignment() {
  static unsigned long lastDistancePrint = 0;

  if (millis() - lastDistancePrint >= PRINT_DELAY) {
    // Read distance from VL53L0X sensors
    int distance1 = sensor1.readRangeContinuousMillimeters();
    int distance2 = sensor2.readRangeContinuousMillimeters();

    // Subtract VOF offset
    int adjustedDistance1 = distance1 - SENSOR_OFFSET1;
    int adjustedDistance2 = distance2 - SENSOR_OFFSET2;

    if (sensor1.timeoutOccurred() || sensor2.timeoutOccurred()) {
      Serial.println("Error: VL53L0X-Sensor - Timeout");
    } else {
      Serial.print("Distance to Joint (S1): ");
      Serial.print(adjustedDistance1);
      Serial.print(" mm | Distance to Joint (S2): ");
      Serial.print(adjustedDistance2);
      Serial.println(" mm");

      // Check alignment
      bool aligned = abs(adjustedDistance1 - adjustedDistance2) <= 8;
      Serial.print("Alignment: ");
      Serial.println(aligned ? "Aligned" : "Not Aligned");

      // Check distance to joint
      bool correctDistance = (adjustedDistance1 < VOF_THRESHOLD && adjustedDistance1 > 0) &&  // Distance Joint to Sensor S1 below 10mm
                             (adjustedDistance2 < VOF_THRESHOLD && adjustedDistance2 > 0);    // Distance Joint to Sensor S2 below 10mm
      Serial.print("Distance: ");
      Serial.println(correctDistance ? "Correct Distance" : "Error Distance");

      if (aligned && correctDistance) {
        Serial.println("Joint is correctly aligned AND within the correct distance. Starting motor...");
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

  // If switches 1 and 2 are pressed simultaneously, start motor
  if (taster1Zustand == LOW && taster2Zustand == LOW) {
    Serial.println("Joint is correctly aligned AND within the correct distance (with endstop 1 & 2). Starting motor...");
    currentState = MOVE_MOTOR;
    stepCounter = 0;
    digitalWrite(ENA_PIN, HIGH);  // Enable motor
    digitalWrite(DIR_PIN, HIGH);  // Direction: clockwise
  }
}

// ============================================================================
//  moveMotor: Move stepper motor 9800 steps
// ============================================================================

void moveMotor() {
  static unsigned long lastStepPrint = 0;

  // 9800 steps
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
    Serial.println("Set steps reached. Confirming connection...");
    currentState = CONFIRM_CONNECTION;
  }
}

// ========================================================================================================================================================
//  Function --> confirmConnection: Check endstops to confirm successful connection
// ========================================================================================================================================================

void confirmConnection() {
  int taster1Zustand = digitalRead(taster1Pin);
  int taster2Zustand = digitalRead(taster2Pin);

  if (taster1Zustand == LOW && taster2Zustand == LOW) {
    Serial.println("Connection between EE and Joint successful!");
    delayStartTime = millis();    // Save delay start time
    currentState = WAIT_DELAY;
  } else {
    Serial.println("Error: Connection not confirmed by endstops.");
    // Handle error or retry logic here if needed
  }
}

// ==============================================================================================================
//  waitDelay: Wait for 60 seconds until stepper motor turns back in the other direction to release joint again
// ==============================================================================================================

void waitDelay() {
  static unsigned long lastDelayPrint = 0;

  if (millis() - lastDelayPrint >= PRINT_DELAY) {
    unsigned long remainingTime = (60000 - (millis() - delayStartTime)) / 1000;      // currently 60s delay set
    Serial.print("Waiting: ");
    Serial.print(remainingTime);
    Serial.println(" seconds remaining");
    lastDelayPrint = millis();
  }

  // 60 second delay
  if (millis() - delayStartTime >= 60000) {                // currently 60s delay set
    currentState = MOVE_MOTOR_BACKWARD;
  }
}

// ======================================================================
//  moveMotorBackward: Move motor 9800 steps backward to release joint
// ======================================================================

void moveMotorBackward() {
  static unsigned long lastBackwardPrint = 0;
  static int backwardSteps = 0;

  if (backwardSteps == 0) {
    Serial.println("Release of Joint");
    digitalWrite(ENA_PIN, HIGH);  // Enable motor
    digitalWrite(DIR_PIN, LOW);   // Direction: counterclockwise
  }

  // 9800 steps backwards
  if (backwardSteps < 9800) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    backwardSteps++;

    if (millis() - lastBackwardPrint >= PRINT_DELAY) {
      Serial.print("Backward step ");
      Serial.print(backwardSteps);
      Serial.println(" of 9800");
      lastBackwardPrint = millis();
    }
  } else {
    digitalWrite(ENA_PIN, LOW);  // Disable motor
    currentState = WAIT_FOR_ALIGNMENT;
    backwardSteps = 0;
    Serial.println("Joint successfully released. EE system ready for new task");
  }
}
