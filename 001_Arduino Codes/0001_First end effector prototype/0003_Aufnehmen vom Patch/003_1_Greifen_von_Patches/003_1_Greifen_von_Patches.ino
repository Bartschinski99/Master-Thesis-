// =========================================================
//  Project:  Masterthesis
//  Author:   Lukas Bartschinski
//  Date:     16.02.2026
//  Description:
//  This sketch controls one servo motor, one endstop, and two VL53L0X distance sensors for patch alignment
// =========================================================

#include <Servo.h>      // Include Servo library
#include <Wire.h>       // Include Wire library for I2C
#include <VL53L0X.h>    // Include VL53L0X library

// Servo motor
Servo meinServo;         // Create servo object
const int tasterPin = 22; // Limit switch Nr. 3 Patch

// VL53L0X sensor pins
const int sensor1XSHUT = 8;
const int sensor2XSHUT = 9;

// VL53L0X sensor objects
VL53L0X sensor1;
VL53L0X sensor2;

// Constants for VL53L0X sensors
const int SENSOR_OFFSET1 = 47;  // Offset for sensor 1
const int SENSOR_OFFSET2 = 67;  // Offset for sensor 2
const unsigned long PRINT_DELAY = 1000; // Delay for serial output in ms

// Process state tracking
enum ProcessState { WAIT_FOR_PATCH_ALIGNMENT, PICKUP_PATCH };
ProcessState currentState = WAIT_FOR_PATCH_ALIGNMENT;

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I²C bus

  // Initialize servo motor
  meinServo.attach(5);  // Attach servo to pin 5
  meinServo.write(0);   // Set servo to starting position (0°)

  // Initialize limit switch
  pinMode(tasterPin, INPUT_PULLUP);  // Activate internal pull-up resistor

  // Initialize VL53L0X sensors
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

  Serial.println("EE system ready. Waiting for patch alignment and pickup");
}

// ========================================================================================
//  Function --> waitForPatchAlignment: Wait for correct patch alignment using VL53L0X sensors
// ========================================================================================

void waitForPatchAlignment() {
  static unsigned long lastDistancePrint = 0;

  if (millis() - lastDistancePrint >= PRINT_DELAY) {
    // Read distance from VL53L0X sensors
    int distance1 = sensor1.readRangeContinuousMillimeters();
    int distance2 = sensor2.readRangeContinuousMillimeters();

    // Subtract offset
    int adjustedDistance1 = distance1 - SENSOR_OFFSET1;
    int adjustedDistance2 = distance2 - SENSOR_OFFSET2;

    if (sensor1.timeoutOccurred() || sensor2.timeoutOccurred()) {
      Serial.println("Error: VL53L0X-Sensor - Timeout");
    } else {
      Serial.print("Distance to Patch MGSE(S1): ");
      Serial.print(adjustedDistance1);
      Serial.print(" mm | Distance to Patch MGSE (S2): ");
      Serial.print(adjustedDistance2);
      Serial.println(" mm");

      // Check alignment
      bool aligned = abs(adjustedDistance1 - adjustedDistance2) <= 10;
      Serial.print("Alignment: ");
      Serial.println(aligned ? "Aligned" : "Not Aligned");

      // Check distance alignment
      bool correctDistance = (adjustedDistance1 < 200 && adjustedDistance1 > 0) &&
                             (adjustedDistance2 < 200 && adjustedDistance2 > 0);
      Serial.print("Distance: ");
      Serial.println(correctDistance ? "Correct Distance" : "Error Distance");

      if (aligned && correctDistance) {
        Serial.println("Patch is correctly aligned AND within the correct distance. Ready for pickup...");
        currentState = PICKUP_PATCH;
      }
    }
    lastDistancePrint = millis();
  }
}

// ===============================================================================
//  Function --> pickupPatch: Pickup the patch when aligned and endstop is pressed
// ===============================================================================

void pickupPatch() {
  if (digitalRead(tasterPin) == LOW) {
    Serial.println("Patch in the correct position. Locking started");

    delay(5000); // Delay after endstop activation: wait 5 seconds

    // Rotate servo to 100°
    meinServo.write(100);
    Serial.println("Patch successfully connected.");
    delay(10000);  // Delay after pickup: wait 10 seconds

    // Rotate servo back to starting position (0°)
    meinServo.write(0);
    delay(5000);  // Wait 5 seconds

    // Reset state for next cycle
    currentState = WAIT_FOR_PATCH_ALIGNMENT;
    Serial.println("EE system ready for new patch pickup");
  }
}

// ===================
//  Main program loop
// ===================

void loop() {
  switch (currentState) {
    case WAIT_FOR_PATCH_ALIGNMENT:
      waitForPatchAlignment();
      break;
    case PICKUP_PATCH:
      pickupPatch();
      break;
  }
}
