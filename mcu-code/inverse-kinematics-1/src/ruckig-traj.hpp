#pragma once
#include <ruckig/ruckig.hpp>
#include <kinematics.hpp>   

using namespace ruckig;
const int DOFs = 2;
inline Ruckig<DOFs> ruck(0.005);
inline InputParameter<DOFs> input;
inline OutputParameter<DOFs> output;

inline void setupRuckig() 
{
    input.max_velocity = {1.0, 1.0};       
    input.max_acceleration = {3, 3}; 
    input.max_jerk = {5.5,5.5};         

    input.current_position = {0.0, 0.0};
    input.current_velocity = {0.0, 0.0};
    input.current_acceleration = {0.0, 0.0};

    // 3600 degrees / 360 = 10 Revolutions!
    input.target_position = {0.0, 0.0};
    input.target_velocity = {0.0, 0.0}; 
    input.target_acceleration = {0.0, 0.0};
    
    input.synchronization = ruckig::Synchronization::Time;

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

inline void handleRuckigTargetUpdate(const Pose& targetPose, const TrigCache& t)
{
    // Get current joint position
    // Get current TCP positon using current joint angles
    Pose currentPose;
    ForwardKinematics(t, currentPose);
    
    // Get target TCP position
    // Feed both into ruckig.input
    input.current_position  = {currentPose.x, currentPose.y};
    input.target_position   = {targetPose.x, targetPose.y}; 

    
}

