#pragma once

#include <math.h>
#include <Arduino.h>

#define L1 0.110356
#define L2 0.143077
#define FK_ITERATIONS 30
#define FK_TOLERANCE 0.0001
#define FK_STEP_SIZE 1.0

#define LUT_SIZE 3600
#define LUT_RESOLUTION 10.0f // 10 indexes per degree (0.1 deg precision)
inline float sin_table[LUT_SIZE];

struct Joints {
    float q1;
    float q2;
};

struct TrigCache{
    float s1, s2, c1, c2, s12, c12;
};

struct Jacobian {
    float j11, j12;
    float j21, j22;
};

struct pose{
    float x, y;
};


inline void initTrigTable() {
    for (int i = 0; i < LUT_SIZE; i++) {
        // Convert index (0 to 3599) back to degrees (0.0 to 359.9)
        float deg = i / LUT_RESOLUTION; 
        
        // Convert degrees to radians because math.h sinf() expects radians
        float rad = deg * (M_PI / 180.0f); 
        
        // Calculate and store
        sin_table[i] = sinf(rad);
    }
}

inline float fast_sin(float deg) {
    float float_idx = deg * LUT_RESOLUTION;
    
    // Handle negative angles cleanly
    if (float_idx < 0) {
        float_idx = fmodf(float_idx, LUT_SIZE) + LUT_SIZE;
    }
    
    int idx_low = (int)float_idx % LUT_SIZE;
    int idx_high = (idx_low + 1) % LUT_SIZE;
    
    float t = float_idx - (int)float_idx; // How far are we between the two indices?
    
    // Linear Interpolation (Lerp) formula: low + t * (high - low)
    return sin_table[idx_low] + t * (sin_table[idx_high] - sin_table[idx_low]);
}

inline float fast_cos(float deg) {
    return fast_sin(deg + 90.0f);
}

// Helper function to keep angles strictly between -PI and +PI (-180 to 180 deg)
inline float normalizeAngle(float rad) {
    rad = fmodf(rad + M_PI, 2.0f * M_PI);
    if (rad < 0.0f) rad += 2.0f * M_PI;
    return rad - M_PI;
}

inline void evalTrig (Joints& q, TrigCache& t)
{
    // 1. Convert internal radians to degrees locally just for the lookup table
    float deg1 = q.q1 * (180.0f / M_PI);
    float deg2 = q.q2 * (180.0f / M_PI);

    // 2. Feed degrees into the LUT
    t.s1 = fast_sin(deg1);
    t.c1 = fast_cos(deg1);
    t.s2 = fast_sin(deg2);
    t.c2 = fast_cos(deg2);
    
    // 3. Fast multiplication identities
    t.s12 = (t.s1 * t.c2) + (t.c1 * t.s2);
    t.c12 = (t.c1 * t.c2) - (t.s1 * t.s2);
}
inline void ForwardKinematics(const TrigCache& t, pose& pose)
{   
    pose.x = L1 * t.c1 + L2 * t.c12;
    pose.y = L1 * t.s1 + L2 * t.s12;
}

// Inverted the Jacobean in Matlab and hardcoding the formula for maximum speed
inline void invJacobean(Jacobian& J, const TrigCache& t)
{   
    // Safetly feature incase near singularity (i.e division by zero, coz matrix is singular), hacky but should work for now.
    float inv_s2 = 1.0f / (t.s2 + 1e-4f); 

    J.j11 = 9.06 * t.c12 * inv_s2;
    J.j12 = 9.06 * t.s12 * inv_s2;
    J.j21 = -63.3f * (0.143f * t.c12 + 0.11f * t.c1) * inv_s2;
    J.j22 = -63.3f * (0.143f * t.s12 + 0.11f * t.s1) * inv_s2;
}

inline void getInverseKinematics(Joints startConfig, pose targetPose, Joints& result, int& iterations)
{
    pose p1;
    TrigCache t1;
    Jacobian j;
    startConfig.q1 = startConfig.q1 * (M_PI / 180.0f); // Convert to radians
    startConfig.q2 = startConfig.q2 * (M_PI / 180.0f); // Convert to radians
    

    evalTrig(startConfig, t1); // Cache all the trig values for efficiency
    // Serial.printf("Trig Cache: s1 = %f, c1 = %f, s2 = %f, c2 = %f, s12 = %f, c12 = %f\n", t1.s1, t1.c1, t1.s2, t1.c2, t1.s12, t1.c12);
    ForwardKinematics(t1, p1); // Get the current end effector pose
    // Serial.printf("Initial Pose after FK: x = %f, y = %f\n", p1.x, p1.y);

    // Calculate the error between current pose and target pose
    pose error;
    error.x = targetPose.x - p1.x;
    error.y = targetPose.y - p1.y;

    float delta_q1, delta_q2;
    int itr = 0;
    while ((fabs(error.x) > FK_TOLERANCE || fabs(error.y) > FK_TOLERANCE) && itr < FK_ITERATIONS)    
    {
        // Serial.printf("Iteration %d: Error x = %f, y = %f\n", itr, error.x, error.y);
        invJacobean(j, t1); 
        delta_q1 = j.j11 * error.x + j.j12 * error.y;
        delta_q2 = j.j21 * error.x + j.j22 * error.y;
        startConfig.q1 += FK_STEP_SIZE * delta_q1;
        startConfig.q2 += FK_STEP_SIZE * delta_q2;
        startConfig.q1 = normalizeAngle(startConfig.q1);
        startConfig.q2 = normalizeAngle(startConfig.q2);
        evalTrig(startConfig, t1);
        ForwardKinematics(t1, p1);

        error.x = targetPose.x - p1.x;
        error.y = targetPose.y - p1.y;
        itr++;
    }
    result.q1 = startConfig.q1 * (180.0f / M_PI); // Convert back to degrees
    result.q2 = startConfig.q2 * (180.0f / M_PI); // Convert back to degrees
    iterations = itr;
    // Serial.printf("Final Joint Angles after %d iterations: q1 = %f, q2 = %f\n", itr, result.q1, result.q2);
    // Serial.printf("Final Pose after IK: x = %f, y = %f\n", p1.x, p1.y);

}