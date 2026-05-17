#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#include "motor_init.h"
#include <math.h>
#include <ArduinoEigen.h>
#include <ruckig/ruckig.hpp>
#include <kinematics.hpp>

using namespace ruckig;
const int DOFs = 2;

// 1. Initialize Ruckig with a 1ms control cycle (0.001 seconds)
// The '8' is the maximum number of waypoints you plan to send at once
Ruckig<DOFs> ruck(0.01);
InputParameter<DOFs> input;
OutputParameter<DOFs> output;


const int TRAJECTORY_STEPS = 200; // 30 points for 30mm (1mm resolution)

struct Waypoint {
  float j1_angle;
  float j2_angle;
};
Waypoint path[TRAJECTORY_STEPS];


// ================= UART PINS =================
#define TMC2_RX 16
#define TMC2_TX 17

void setupRuckig() {
    // 1. ALL MATH IS NOW IN "REVOLUTIONS" INSTEAD OF DEGREES
    // 650 deg/s / 360 = 1.8 rev/s
    input.max_velocity = {1.8, 1.8};       
    
    // 3000 deg/s^2 / 360 = 8.3 rev/s^2
    input.max_acceleration = {2.3, 2.3}; 
    
    // 30000 deg/s^3 / 360 = 83.3 rev/s^3
    input.max_jerk = {20.3,23.3};         

    input.current_position = {0.0, 0.0};
    input.current_velocity = {0.0, 0.0};
    input.current_acceleration = {0.0, 0.0};

    // 3600 degrees / 360 = 10 Revolutions!
    input.target_position = {0.0, 0.0};
    input.target_velocity = {0.0, 0.0}; 
    input.target_acceleration = {0.0, 0.0};
    
    input.synchronization = Synchronization::Time;

    // Validation Check...
    try {
        ruck.validate_input(input);
        Serial.println("Ruckig Input Validation: SUCCESS");
    } catch (const std::exception& e) {
        Serial.print("FATAL RUCKIG ERROR: ");
        Serial.println(e.what());
        while (true) delay(1000);
    }
}


void setup()
{   
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Initialise
    delay(100);
    Serial.println("\n--- Initializing System ---");
    delay(100);

    initJoints(1, 1, 1); 


    engine.init();
    stepper1 = engine.stepperConnectToPin(J1_STEP_PIN);
    stepper1->setDirectionPin(J1_DIR_PIN, true);
    stepper1->setAutoEnable(true);
    stepper1->setSpeedInHz(1000);      
    stepper1->setAcceleration(6000);     


    stepper2 = engine.stepperConnectToPin(J2_STEP_PIN);
    stepper2->setDirectionPin(J2_DIR_PIN, true);
    stepper2->setAutoEnable(true);
    stepper2->setSpeedInHz(1000);      
    stepper2->setAcceleration(5000); 
    // stepper2->runBackward();

    stepper1->moveTo(15*J1_STEPS_PER_DEG);
    stepper2->moveTo(15*J2_STEPS_PER_DEG);
    Joints startConfig = {45, 45};
    Joints result;
    pose targetPose = {0.078033, 0.1311};
    initTrigTable(); // Initialize the sine lookup table
    // start time to calculate IK
    unsigned long startTime = micros(); // Record start time in microseconds
    int itr_counter;
    getInverseKinematics(startConfig, targetPose, result, itr_counter);
    unsigned long endTime = micros();   // Record end time in microseconds

    unsigned long duration = endTime - startTime;

    // Print the result in microseconds, or convert to milliseconds as a float
    Serial.printf("IK Calculation Time: %lu us\n", duration);
    Serial.printf("IK Calculation Time: %.6f ms\n", (float)duration / 1000.0);
    Serial.printf("Final Joint Angles: q1 = %f, q2 = %f\n", result.q1, result.q2);
    Serial.printf("Iterations: %d\n", itr_counter);
}


String inputString = "";

float j1_angle = 0;
float j2_angle = 0;

// Joint limits
const float J1_MIN = -180;
const float J1_MAX = 360;

const float J2_MIN = -180;
const float J2_MAX = 360;

float t = 0;
Result res;
Joints startConfig = {0, 0}; 
int itr_counter = 0;
void loop() 
{
    // Wait for incoming serial data from Python
    if (Serial.available() > 0) 
    {
        // Check if it's the start of our command format "<"
        if (Serial.read() == '<') 
        {
            // Parse the X and Y floats sent from Python
            float target_x = Serial.parseFloat();
            Serial.read(); // Consume the comma ','
            float target_y = Serial.parseFloat();
            
            // Consume the closing bracket or newline
            while(Serial.read() != '\n' && Serial.available()) {} 

            // Create our target pose
            pose targetPose = {target_x, target_y};
            
            // Run the incredibly fast IK solver!
            Joints result;
            // You can set this to the current joint angles if you have that info
            startConfig.q1 = stepper1->getCurrentPosition() / J1_STEPS_PER_DEG;
            startConfig.q2 = stepper2->getCurrentPosition() / J2_STEPS_PER_DEG;
            Serial.printf("Current Joint Angles: q1 = %f, q2 = %f\n", startConfig.q1, startConfig.q2);
            unsigned long startTime = micros();
            getInverseKinematics(startConfig, targetPose, result, itr_counter);
            unsigned long duration = micros() - startTime;
            
            // Optional: Print back to the terminal (Python will ignore this)
            Serial.printf("Target Received: X=%.4f, Y=%.4f | Time: %lu us | angle calculated: q1=%.4f, q2=%.4f in %d iterations\n", target_x, target_y, duration, result.q1, result.q2, itr_counter);
            
            // TODO: Command your stepper motors to move to result.q1 and result.q2
            stepper1->moveTo(result.q1 * J1_STEPS_PER_DEG);
            stepper2->moveTo(result.q2 * J2_STEPS_PER_DEG);
        }
    }
    
    // Call your engine/stepper run routines here continuously
    // engine.run();
}