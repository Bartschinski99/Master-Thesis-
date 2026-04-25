// =============================================
//  Project: Masterthesis
//  Author:   Lukas Bartschinski
//  Date:     26.03.2026
//  Description:
//  This sketch controls two VL53L0X distance sensors, a stepper motor
//  and 4 Force Resistance Sensors (FSR)
// =============================================

#include <Wire.h>
#include <VL53L0X.h>

// =============================================
//                Pin Definitions
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

// NEMA 17 Stepper Motor settings: 1.46A (ON;ON;OFF) / 12V

// Force Resistance Sensors (FSR) Pins
const int fsrPins[] = {A0, A1, A2, A3}; 

// =============================================
//                Constants
// =============================================

// VOF sensor
const int SENSOR_OFFSET1 = 77; //measured Offset in mm 
const int SENSOR_OFFSET2 = 67; //measured Offset in mm

// Stepper motor --> step delay in microseconds
const int STEP_DELAY_US_INITIAL = 500; // 500 µs delay for initial steps
const unsigned long MOTOR_STEPS = 12500; // Total steps for movement --> backup steps

// FSR Constants from Datasheet
float VCC_FSR = 5.0;       // Define VCC as 5V (Supply Voltage from the Arduino Board
float THRESHOLD_RC = 10.0; // 10kOhm Threshold for stopping motor

// Other
const unsigned long PRINT_DELAY = 1000; // Verzögerung für serielle Ausgaben in ms

// =============================================
//                Global Variables
// =============================================

VL53L0X sensor1;
VL53L0X sensor2;

enum ProcessState { WAIT_FOR_ALIGNMENT, MOVE_MOTOR, WAIT_DELAY, MOVE_MOTOR_BACKWARD };
ProcessState currentState = WAIT_FOR_ALIGNMENT;

unsigned long delayStartTime = 0;
unsigned long stepCounter = 0;

// =============================================
//  SETUP: Initialize hardware and sensors
// =============================================

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I²C bus

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

  sensor1.startContinuous(50);
  sensor2.startContinuous(50);

  pinMode(taster1Pin, INPUT_PULLUP);
  pinMode(taster2Pin, INPUT_PULLUP);
  pinMode(taster4Pin, INPUT_PULLUP);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  // FSR Pins initialisieren
  for(int i=0; i<4; i++) pinMode(fsrPins[i], INPUT);

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

// =============================================
// FRS Functions from Datasheet
// =============================================

bool isForceLimitReached() {
  bool limitReached = false;
  
  for (int i = 0; i < 4; i++) {
    int val = analogRead(fsrPins[i]);
    float VOUT = (val / 1023.0) * VCC_FSR;
    float RC;

    if (VOUT > 0) {
      RC = (510.0 * VCC_FSR / VOUT) - 510.0;
    } else {
      RC = 9999.0; 
    }

    // Check if resistance falls below 10kOhm (Pressure increases)
    if (RC < THRESHOLD_RC && VOUT > 0) {
      Serial.print("\n>>> TARGET FORCE REACHED ON SENSOR "); Serial.print(i);
      Serial.print(" | RC: "); Serial.print(RC, 2); Serial.println(" kOhm");
      limitReached = true;
    }
  }
  return limitReached;
}

// ============================================================================================================================================================
//  Function --> waitForAlignment: Wait for correct alignment with the struts or the switch press the connection of endstops 1 and 2 to start the stepper motor
// ============================================================================================================================================================

void waitForAlignment() {
  static unsigned long lastDistancePrint = 0;

  if (millis() - lastDistancePrint >= PRINT_DELAY) {
    int distance1 = sensor1.readRangeContinuousMillimeters();
    int distance2 = sensor2.readRangeContinuousMillimeters();

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

      bool aligned = abs(adjustedDistance1 - adjustedDistance2) <= 8;
      Serial.print("Alignment: ");
      Serial.println(aligned ? "Aligned" : "Not Aligned");

      bool correctDistance = (adjustedDistance1 < 15 && adjustedDistance1 > 0) && 
                             (adjustedDistance2 < 15 && adjustedDistance2 > 0);
      Serial.print("Distance: ");
      Serial.println(correctDistance ? "Correct Distance" : "Error Distance");

      if (aligned && correctDistance) {
        Serial.println("Strut is correctly aligned AND within the correct distance. Motor ready for starting...");
        startMoving();
      }
    }
    lastDistancePrint = millis();
  }

  if (digitalRead(taster1Pin) == LOW && digitalRead(taster2Pin) == LOW) {
    Serial.println("Strut is correctly aligned AND within the correct distance (with endstop 1 & 2). Motor ready for starting...");
    startMoving();
  }
}

void startMoving() {
  currentState = MOVE_MOTOR;
  stepCounter = 0;
  digitalWrite(ENA_PIN, HIGH);  // Enable motor
  digitalWrite(DIR_PIN, HIGH);  // Direction: clockwise
}

// ==============================================================================
//  moveMotor: Move stepper until backup steps reached or Force Limit is reached
// ==============================================================================

void moveMotor() {
  static unsigned long lastStepPrint = 0;

  // Prüfung der Kraftsensoren
  if (isForceLimitReached()) {
    Serial.println("Motor stopped by Force Sensor.");
    delayStartTime = millis();
    currentState = WAIT_DELAY;
    return;
  }

  if (stepCounter < MOTOR_STEPS) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    stepCounter++;

    if (millis() - lastStepPrint >= PRINT_DELAY) {
      Serial.print("Schritt ");
      Serial.print(stepCounter);
      Serial.print(" von ");
      Serial.println(MOTOR_STEPS);
      lastStepPrint = millis();
    }
  } else {
    Serial.println("Set steps reached.");
    delayStartTime = millis();
    currentState = WAIT_DELAY;
  }
}

// ==============================================================================================================
//  waitDelay: Wait for 90 seconds until stepper motor turns back in the other direction to release strut again
// ==============================================================================================================

void waitDelay() {
  static unsigned long lastDelayPrint = 0;
  unsigned long waitTimeMs = 90000;

  if (millis() - lastDelayPrint >= PRINT_DELAY) {
    unsigned long remainingTime = (waitTimeMs - (millis() - delayStartTime)) / 1000;
    Serial.print("Waiting: ");
    Serial.print(remainingTime);
    Serial.println(" seconds remaining");
    lastDelayPrint = millis();
  }

  if (millis() - delayStartTime >= waitTimeMs) {
    currentState = MOVE_MOTOR_BACKWARD;
  }
}

// ======================================================================
//  moveMotorBackward: Move motor 11400 steps backward to release strut
// ======================================================================


void moveMotorBackward() {
  static unsigned long lastBackwardPrint = 0;
  static int backwardSteps = 0;

  if (backwardSteps == 0) {
    Serial.println("Release of Strut");
    digitalWrite(ENA_PIN, HIGH); // Enable motor
    digitalWrite(DIR_PIN, LOW); // Direction: counterclockwise
  }
  
 // 11400 steps backwards
  if (backwardSteps < 11400) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US_INITIAL);
    backwardSteps++;

    if (millis() - lastBackwardPrint >= PRINT_DELAY) {
      Serial.print("Backward step ");
      Serial.print(backwardSteps);
      Serial.println(" of ");
      lastBackwardPrint = millis();
    }
  } else {
    digitalWrite(ENA_PIN, LOW);
    currentState = WAIT_FOR_ALIGNMENT;
    backwardSteps = 0;
    Serial.println("Strut successfully released. EE system ready for new task");
  }
}
