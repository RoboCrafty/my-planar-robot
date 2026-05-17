#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#include "motor_init.h"
#include <math.h>
#include <ArduinoEigen.h>
#include <kinematics.hpp>
#include <ruckig-traj.hpp>


// ================= UART PINS AND OTHER VARIABLES =================
#define TMC2_RX 16
#define TMC2_TX 17
void handleRuckigLoop();


// Goal here is, we feed current Joint position, which gives us current TCP position using FW Kin, we then generate a smooth jerk limited trajectory with Rucking in cartesian space
// Then convert that to joint space with the numerical inverse kinematics solver
Joints      currentPoseJ, targetPoseIntermediateJ, resultInvKin;
Pose        currentPoseCart, targetPoseCart, targetPoseIntermediateCart;
TrigCache   trigCache1, trigCache2;
int         invKinItrTracker{0};

int32_t     j1_commanded_steps = 0;
int32_t     j2_commanded_steps = 0;

void setup()
{   
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, TMC2_RX, TMC2_TX);

    // Initialise
    delay(100);
    Serial.println("\n--- Initializing System ---");
    delay(100);

    initJoints(1, 1, 1); 
    initTrigTable(); // Initialize the sine lookup table
    setupRuckig(); 


    engine.init();
    stepper1 = engine.stepperConnectToPin(J1_STEP_PIN);
    stepper1->setDirectionPin(J1_DIR_PIN, true);
    stepper1->setAutoEnable(true);
    stepper1->setSpeedInHz(6000);      
    stepper1->setAcceleration(6000); // To ignore stepper library motion profile and use ruckig's profile instead


    stepper2 = engine.stepperConnectToPin(J2_STEP_PIN);
    stepper2->setDirectionPin(J2_DIR_PIN, true);
    stepper2->setAutoEnable(true);
    stepper2->setSpeedInHz(6000);      
    stepper2->setAcceleration(6000); 
    // stepper2->runBackward();

    stepper1->moveTo(15*J1_STEPS_PER_DEG);
    stepper2->moveTo(15*J2_STEPS_PER_DEG);
    delay(500); // Wait for the motors to reach the initial position so theyre not starting near singularity
    j1_commanded_steps = stepper1->getPositionAfterCommandsCompleted();
    j2_commanded_steps = stepper2->getPositionAfterCommandsCompleted();
    stepper1->setSpeedInHz(30000);      
    stepper1->setAcceleration(10000000); 
    stepper2->setSpeedInHz(30000);      
    stepper2->setAcceleration(10000000); 
    
    // // start time to calculate IK
    // unsigned long startTime = micros(); // Record start time in microseconds
    // int itr_counter;
    // unsigned long endTime = micros();   // Record end time in microseconds

    // unsigned long duration = endTime - startTime;

    // // Print the result in microseconds, or convert to milliseconds as a float
    // Serial.printf("IK Calculation Time: %lu us\n", duration);
    // Serial.printf("IK Calculation Time: %.6f ms\n", (float)duration / 1000.0);
    // Serial.printf("Final Joint Angles: q1 = %f, q2 = %f\n", result.q1, result.q2);
    // Serial.printf("Iterations: %d\n", itr_counter);

    // Define 1 target trajectory i.e, move 10cm left from current position in cartesian space in a straight line
    currentPoseJ.q1 = stepper1-> getCurrentPosition() / J1_STEPS_PER_DEG;
    currentPoseJ.q2 = stepper2-> getCurrentPosition() / J2_STEPS_PER_DEG;
    Serial.println("Current Joint Angles: q1 = " + String(currentPoseJ.q1) + ", q2 = " + String(currentPoseJ.q2));
    degToRad(currentPoseJ);
    evalTrig(currentPoseJ, trigCache1);

    // Calculate target pose in C space
    ForwardKinematics(trigCache1, targetPoseCart);
    Serial.println("Current Cartesian Position: x = " + String(targetPoseCart.x) + ", y = " + String(targetPoseCart.y));
    targetPoseCart.x -= 0.30; // Move 9 cm left
    Serial.println("Target Cartesian Position: x = " + String(targetPoseCart.x) + ", y = " + String(targetPoseCart.y));
    handleRuckigTargetUpdate(targetPoseCart, trigCache1);



}


String inputString = "";

Result res;
Joints startConfig = {0, 0}; 
int itr_counter = 0;
float t=0;

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
            targetPoseCart.x = target_x;
            targetPoseCart.y = target_y;
            // stepper2->moveTimed(target_x, target_y, NULL, true); // Placeholder for testing, replace with actual Ruckig handling
            degToRad(currentPoseJ);
            evalTrig(currentPoseJ, trigCache1);
            handleRuckigTargetUpdate(targetPoseCart, trigCache1);
            Serial.printf("Target Received: X=%.4f, Y=%.4f\n", target_x, target_y);
            std::cout << "t,q1,q2,q1dot,q2dot,q1ddot,q2ddot" << std::endl;
        }
    }
    
    if (stepper1->queueEntries() < 10 || stepper2->queueEntries() < 10) {
        handleRuckigLoop();
    }




}












void handleRuckigLoop() {  
    uint32_t duration_ticks = (TICKS_PER_S / 1000) * 5;   
  
    // 1. Advance Ruckig  
    ruck.update(input, output);  
    targetPoseIntermediateCart.x = output.new_position[0];  
    targetPoseIntermediateCart.y = output.new_position[1];  

    currentPoseJ.q1 = stepper1-> getCurrentPosition() / J1_STEPS_PER_DEG;
    currentPoseJ.q2 = stepper2-> getCurrentPosition() / J2_STEPS_PER_DEG;
    getInverseKinematics(currentPoseJ, &targetPoseIntermediateCart, &targetPoseIntermediateJ, invKinItrTracker, trigCache1);  
    std::cout << t << "," << targetPoseIntermediateJ.q1 << "," << targetPoseIntermediateJ.q2 <<  "," << output.new_velocity[0] << "," << output.new_velocity[1] << "," << output.new_acceleration[0] << "," << output.new_acceleration[1] << std::endl;
    t+=0.005;
    // 2. Calculate deltas  
    int32_t new_target_steps_1 = targetPoseIntermediateJ.q1 * J1_STEPS_PER_DEG;  
    int32_t new_target_steps_2 = targetPoseIntermediateJ.q2 * J2_STEPS_PER_DEG;  
    int16_t delta_1 = new_target_steps_1 - j1_commanded_steps;  
    int16_t delta_2 = new_target_steps_2 - j2_commanded_steps;  
  
    // 3. Queue moves with start=false to prepare for sync  
    MoveTimedResultCode r1 = stepper1->moveTimed(delta_1, duration_ticks, NULL, false);  
    MoveTimedResultCode r2 = stepper2->moveTimed(delta_2, duration_ticks, NULL, false);  
  
    // 4. Only update trackers and start if BOTH were accepted  
    if (r1 <= MOVE_TIMED_OK && r2 <= MOVE_TIMED_OK) {  
        j1_commanded_steps += delta_1;  
        j2_commanded_steps += delta_2;  
        output.pass_to_input(input);  
  
        // Synchronized trigger  
        noInterrupts();  
        stepper1->moveTimed(0, 0, NULL, true);  
        stepper2->moveTimed(0, 0, NULL, true);  
        interrupts();  
    }   
    // If BUSY, we don't call pass_to_input, so Ruckig will retry the same step next loop  
}