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
            degToRad(currentPoseJ);
            evalTrig(currentPoseJ, trigCache1);
            handleRuckigTargetUpdate(targetPoseCart, trigCache1);
            Serial.printf("Target Received: X=%.4f, Y=%.4f\n", target_x, target_y);
            std::cout << "t,q1,q2,q1dot,q2dot,q1ddot,q2ddot" << std::endl;
        }
    }
    
    unsigned long loopStartTime = micros();
    
    handleRuckigLoop();
    t += 0.005f; 

    // Wait for exactly 5ms (5000 us) to pass since loopStartTime
    while (micros() - loopStartTime < 5000) {
        // Yield to let ESP32 handle WiFi/background tasks if needed
        yield(); 
    }





}












void handleRuckigLoop()
{
    // Loop following
    // ruck.update()
    ruck.update(input,output);
    
    
    // Get output.new_pose
    targetPoseIntermediateCart.x = output.new_position[0];
    targetPoseIntermediateCart.y = output.new_position[1];
    // run inv kinematics to get target joint angles for that tcp pose
    currentPoseJ.q1 = stepper1-> getCurrentPosition() / J1_STEPS_PER_DEG;
    currentPoseJ.q2 = stepper2-> getCurrentPosition() / J2_STEPS_PER_DEG;
    getInverseKinematics(currentPoseJ, &targetPoseIntermediateCart, &targetPoseIntermediateJ,invKinItrTracker, trigCache1);

    // 1. Get the current and target steps for Joint 1
    long current_steps_1 = stepper1->getCurrentPosition();
    long target_steps_1 = targetPoseIntermediateJ.q1 * J1_STEPS_PER_DEG;
    long steps_to_go_1 = target_steps_1 - current_steps_1;

    long current_steps_2    = stepper2->getCurrentPosition();
    long target_steps_2     = targetPoseIntermediateJ.q2 * J2_STEPS_PER_DEG;
    long steps_to_go_2      = target_steps_2 - current_steps_2;

    // 2. Calculate required speed to arrive in exactly 5ms (0.005s)
    // Dividing by 0.005 is the same as multiplying by 200
    uint32_t required_hz_1 = abs(steps_to_go_1) * 200;
    uint32_t required_hz_2 = abs(steps_to_go_2) * 200;

    // 3. Command the motor (Only update if it actually needs to move)
    if (required_hz_1 > 0) {
        stepper1->setSpeedInHz(required_hz_1);
        stepper1->moveTo(target_steps_1);
    }
    if (required_hz_2 > 0) {
        stepper2->setSpeedInHz(required_hz_2);
        stepper2->moveTo(target_steps_2);
    }



    std::cout << t << "," << targetPoseIntermediateJ.q1 << "," << targetPoseIntermediateJ.q2 <<  "," << output.new_velocity[0] << "," << output.new_velocity[1] << "," << output.new_acceleration[0] << "," << output.new_acceleration[1] << std::endl;

    output.pass_to_input(input);
    // move steppers to those joint angles using moveto
    // wait(non blocking) for 5ms. 
}