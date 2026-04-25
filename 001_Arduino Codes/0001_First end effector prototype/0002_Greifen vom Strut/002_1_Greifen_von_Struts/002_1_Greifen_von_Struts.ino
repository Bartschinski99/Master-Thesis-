// =============================================
//  Project:  Masterthesis
//  Author:   Lukas Bartschinski
//  Date:     16.02.2026
//  Description:
//  This sketch controls two VL53L0X distance sensors and a stepper motor
// =============================================

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

// NEMA 17 Stepper Motor settings: 1.46A (ON;ON;OFF) / 12V / 400 (OFF;ON;ON) 
// =============================================
//               Constants
// =============================================

// VOF sensor
const int SENSOR_OFFSET1 = 47; //measured Offset in mm 
const int SENSOR_OFFSET2 = 67; //measured Offset in mm

// Stepper motor --> step delay in microseconds
const int STEP_DELAY_US_INITIAL = 500; // 500 µs delay for initial steps

// Other
const unsigned long PRINT_DELAY = 1000; // Verzögerung für serielle Ausgaben in ms

// =============================================
//               Global Variables
// =============================================

// VOF sensor objects
VL53L0X sensor1;
VL53L0X sensor2;

// Process state tracking
enum ProcessState { WAIT_FOR_ALIGNMENT, MOVE_MOTOR, WAIT_DELAY, MOVE_MOTOR_BACKWARD };
ProcessState currentState = WAIT_FOR_ALIGNMENT;

// Variables for delay and step counting
unsigned long delayStartTime = 0;
unsigned long stepCounter = 0;
const unsigned long MOTOR_STEPS = 6900; // 6900 steps after alignment

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

  Serial.println("EE system ready. Waiting for correct alignment with Strut");
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
    case WAIT_DELAY:
      waitDelay();
      break;
    case MOVE_MOTOR_BACKWARD:
      moveMotorBackward();
      break;
  }
}

// ============================================================================================================================================================
//  Function --> waitForAlignment: Wait for correct alignment with the struts or the switch press the connection of endstops 1 and 2 to start the stepper motor
// ============================================================================================================================================================

void waitForAlignment() {
  static unsigned long lastDistancePrint = 0;

  if (millis() - lastDistancePrint >= PRINT_DELAY) {
    // Read distance from VL53L0X sensors
    int distance1 = sensor1.readRangeContinuousMillimeters();
    int distance2 = sensor2.readRangeContinuousMillimeters();

    // Subtract 20mm offset
    int adjustedDistance1 = distance1 - SENSOR_OFFSET1;
    int adjustedDistance2 = distance2 - SENSOR_OFFSET2;

    if (sensor1.timeoutOccurred() || sensor2.timeoutOccurred()) {
      Serial.println("Fehler: VL53L0X-Sensor - Timeout");
    } else {
      Serial.print("Distance to Strut (S1): ");
      Serial.print(adjustedDistance1);
      Serial.print(" mm | Distance to Strut (S2): ");
      Serial.print(adjustedDistance2);
      Serial.println(" mm");

      // Check alignment
      bool aligned = abs(adjustedDistance1 - adjustedDistance2) <= 8;
      Serial.print("Alignment: ");
      Serial.println(aligned ? "Aligned" : "Not Aligned");

      // Check distance alignment
      bool correctDistance = (adjustedDistance1 < 15 && adjustedDistance1 > 0) &&        //  distance Strut to Sensor S1 below 15mm
                             (adjustedDistance2 < 15 && adjustedDistance2 > 0);          //  distance Strut to Sensor S2 below 15mm
      Serial.print("Distance: ");
      Serial.println(correctDistance ? "Correct Distance" : "Error Distance");

      if (aligned && correctDistance) {
        Serial.println("Strut is correctly aligned AND within the correct distance. Motor ready for starting...");
        currentState = MOVE_MOTOR;
        stepCounter = 0;
        digitalWrite(ENA_PIN, HIGH);  // Enable motor
        digitalWrite(DIR_PIN, LOW);  // Direction: clockwise
      }
    }
    lastDistancePrint = millis();
  }

  // Backup for the alignment and distance test with endstop 1 and 2
  int taster1Zustand = digitalRead(taster1Pin);
  int taster2Zustand = digitalRead(taster2Pin);

  // If switches 1 and 2 are pressed simultaneously, start motor
  if (taster1Zustand == LOW && taster2Zustand == LOW) {
    Serial.println("Strut is correctly aligned AND within the correct distance (with endstop 1 & 2). Motor ready for starting...");
    currentState = MOVE_MOTOR;
    stepCounter = 0;
    digitalWrite(ENA_PIN, HIGH);  // Enable motor
    digitalWrite(DIR_PIN, LOW);  // Direction: clockwise
  }
}

// ============================================================================
//  moveMotor: Move stepper motor 6900 steps
// ============================================================================

void moveMotor() {
  static unsigned long lastStepPrint = 0;

  // 6900 steps
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
    currentState = WAIT_DELAY;
  }
}

// ==============================================================================================================
//  waitDelay: Wait for 90 seconds until stepper motor turns back in the other direction to release strut again
// ==============================================================================================================

void waitDelay() {
  static unsigned long lastDelayPrint = 0;

  if (millis() - lastDelayPrint >= PRINT_DELAY) {
    unsigned long remainingTime = (90000 - (millis() - delayStartTime)) / 1000;    // currently 90s delay set
    Serial.print("Waiting: ");
    Serial.print(remainingTime);
    Serial.println(" seconds remaining");
    lastDelayPrint = millis();
  }

  // 30 second delay
  if (millis() - delayStartTime >= 90000) {                                        // currently 90s delay set
    currentState = MOVE_MOTOR_BACKWARD;
  }
}

// ======================================================================
//  moveMotorBackward: Move motor 6900 steps backward to release strut
// ======================================================================

void moveMotorBackward() {
  static unsigned long lastBackwardPrint = 0;
  static int backwardSteps = 0;

  if (backwardSteps == 0) {
    Serial.println("Release of Strut");
    digitalWrite(ENA_PIN, HIGH);  // Enable motor
    digitalWrite(DIR_PIN, HIGH);   // Direction: counterclockwise
  }

  // 6900 steps backwards
  if (backwardSteps < 6900) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    backwardSteps++;

    if (millis() - lastBackwardPrint >= PRINT_DELAY) {
      Serial.print("Backward step ");
      Serial.print(backwardSteps);
      Serial.println(" of 6900");
      lastBackwardPrint = millis();
    }
  } else {
    digitalWrite(ENA_PIN, LOW);  // Disable motor
    currentState = WAIT_FOR_ALIGNMENT;
    backwardSteps = 0;
    Serial.println("Strut successfully released. EE system ready for new task");
  }
}
