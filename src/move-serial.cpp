// For moving 2 joints point to point in joint space via commands over Serial, for testing purpose. 

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#include "motor_init.h"
#include <math.h>
#include <ArduinoEigen.h>

const float L1 = 110.4;
const float L2 = 142.0;
const int TRAJECTORY_STEPS = 200; // 30 points for 30mm (1mm resolution)

struct Waypoint {
  float j1_angle;
  float j2_angle;
};
Waypoint path[TRAJECTORY_STEPS];

// ================= UART PINS =================
#define TMC2_RX 16
#define TMC2_TX 17

void setup()
{   
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Initialise
    delay(500);
    Serial.println("\n--- Initializing System ---");
    delay(100);

    initJoints(1, 1, 1); 

    engine.init();
    stepper1 = engine.stepperConnectToPin(J1_STEP_PIN);
    stepper1->setDirectionPin(J1_DIR_PIN, true);
    stepper1->setAutoEnable(true);
    stepper1->setSpeedInHz(6000);      
    stepper1->setAcceleration(3000);     

    stepper2 = engine.stepperConnectToPin(J2_STEP_PIN);
    stepper2->setDirectionPin(J2_DIR_PIN, true);
    stepper2->setAutoEnable(true);
    stepper2->setSpeedInHz(6000);      
    stepper2->setAcceleration(3000); 
}

String inputString = "";

float j1_angle = 0;
float j2_angle = 0;

// Joint limits
const float J1_MIN = 0;
const float J1_MAX = 360;

const float J2_MIN = 0;
const float J2_MAX = 360;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      int commaIndex = inputString.indexOf(',');

      if (commaIndex > 0) {
        String j1_str = inputString.substring(0, commaIndex);
        String j2_str = inputString.substring(commaIndex + 1);

        j1_angle = j1_str.toFloat();
        j2_angle = j2_str.toFloat();

        // Apply limits
        j1_angle = constrain(j1_angle, J1_MIN, J1_MAX);
        j2_angle = constrain(j2_angle, J2_MIN, J2_MAX);

        Serial.print("J1: ");
        Serial.print(j1_angle);

        Serial.print("  J2: ");
        Serial.println(j2_angle);

        stepper1->moveTo(j1_angle * J1_STEPS_PER_DEG);
        stepper2->moveTo(j2_angle * J2_STEPS_PER_DEG);
      }

      inputString = "";
    }
    else {
      inputString += c;
    }
  }
}