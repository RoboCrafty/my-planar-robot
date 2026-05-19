#pragma once

#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"



// ================= JOINT DEFINITIONS =================
// Joint 1 
#define J1_STEP_PIN 13
#define J1_DIR_PIN  12

// Joint 2 
#define J2_STEP_PIN 14
#define J2_DIR_PIN  27


// ================= UART Driver Addresses =================

#define J1_ADDRESS 0b01
#define J2_ADDRESS 0b00

#define R_SENSE 0.10f // Match to your driver

#define SERIAL_PORT1 Serial2

// ================= EXTERN VARIABLES =================

extern FastAccelStepperEngine engine;
extern FastAccelStepper *stepper1;
extern FastAccelStepper *stepper2;

extern TMC2209Stepper J1_driver;
extern TMC2209Stepper J2_driver;

// =============== Constants and Macros ===============
#define J1_CURRENT 1200
#define J2_CURRENT 1200


#define J1_HOLD_MULTIPLIER 0.6
#define J2_HOLD_MULTIPLIER 0.6


#define J1_TPWMTHRS 100
#define J2_TPWMTHRS 100


#define J1_MICOSTEPS 8
#define J2_MICOSTEPS 8


#define J1_GEAR_RATIO 1.0
#define J2_GEAR_RATIO 1.0


#define J1_STEP_PER_REV (400.0 * J1_MICOSTEPS * J1_GEAR_RATIO)
#define J2_STEP_PER_REV (400.0 * J2_MICOSTEPS * J2_GEAR_RATIO)

// #define J1_DEG_PER_STEP (360.0 / J1_STEP_PER_REV)
// #define J2_DEG_PER_STEP (360.0 / J2_DEG_PER_STEP)

#define J1_STEPS_PER_DEG (J1_STEP_PER_REV / 360.0)
#define J2_STEPS_PER_DEG (J2_STEP_PER_REV / 360.0)



// ================= Start =================

void initJoints(bool debug, bool start_J1, bool start_J2);
