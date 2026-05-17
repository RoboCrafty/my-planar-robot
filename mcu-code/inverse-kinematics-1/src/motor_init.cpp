#include "motor_init.h"


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;  

TMC2209Stepper J1_driver(&SERIAL_PORT1, R_SENSE, J1_ADDRESS);
TMC2209Stepper J2_driver(&SERIAL_PORT1, R_SENSE, J2_ADDRESS);

// Layout
// ESP32 --- J5
//       --- J2
//       --- J4
//       --- J6 
//       --- J1
//       --- J3

void initJoints(bool debug, bool start_J1, bool start_J2)
{
    Serial.println("Initializing and starting requested motors...");
    uint8_t result;


    // ============== J1 =================
    J1_driver.begin();
    J1_driver.blank_time(20);
    J1_driver.en_spreadCycle(false);
    J1_driver.TCOOLTHRS(0);
    J1_driver.microsteps(J1_MICOSTEPS);
    J1_driver.intpol(true);
    J1_driver.I_scale_analog(0);
    J1_driver.rms_current(J1_CURRENT, J1_HOLD_MULTIPLIER);
    J1_driver.TPWMTHRS(J1_TPWMTHRS);


    Serial.println(F("\nTesting driver 1 connection... "));
    result= J1_driver.test_connection();
    if (result) 
    {
        Serial.println(F("Failed! ❌"));
        Serial.print(F("Likely cause: "));

        switch (result) 
        {
            case 1: Serial.println(F("Loose connection")); break;
            case 2: Serial.println(F("no power")); break;
        }
        Serial.println(F("Fix the problem and reset board."));
        // abort();
    }
    else
    {
        Serial.println("Driver 1 Initialized! ✅");
    }

    if(start_J1)
    {
        J1_driver.toff(5);
    }
    else
    {
        J1_driver.toff(0);
    }
    

    // ============== J2 =================
    J2_driver.begin();
    J2_driver.blank_time(20);
    J2_driver.en_spreadCycle(false);
    J2_driver.TCOOLTHRS(0);
    J2_driver.microsteps(J2_MICOSTEPS);
    J2_driver.intpol(true);
    J2_driver.I_scale_analog(0);
    J2_driver.rms_current(J2_CURRENT, J2_HOLD_MULTIPLIER);
    J2_driver.TPWMTHRS(J2_TPWMTHRS);
    

    Serial.println(F("\nTesting driver 2 connection... "));
    result= J2_driver.test_connection();
    if (result) 
    {
        Serial.println(F("Failed! ❌"));
        Serial.print(F("Likely cause: "));

        switch (result) 
        {
            case 1: Serial.println(F("Loose connection")); break;
            case 2: Serial.println(F("no power")); break;
        }
        Serial.println(F("Fix the problem and reset board."));
        // abort();
    }
    else
    {
        Serial.println("Driver 2 Initialized! ✅");
    }

    if(start_J2)
    {
        J2_driver.toff(2);
    }
    else
    {
        J2_driver.toff(0);
    }

}