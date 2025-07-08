/*
Changelog - mainControlLoop.c

[Initial]
- Added controlTask() main loop for periodic robot control.
- Reads sensor data, computes inverse kinematics, generates trajectories, and sends motor commands.

[Added]
- Defined NUM_LEGS and JOINTS_PER_LEG constants for robot configuration.
- Created SensorData, JointAngles, and JointTrajectories structs for data handling.
- Implemented stub functions: readSensors(), computeInverseKinematics(), generateTrajectories(), sendMotorCommand().
- Added comments and zero-initialization for simulation/testing.
- Provided printf in sendMotorCommand() for debug output.

[Edit]
- Implemented readSensors() to simulate IMU and position sensor data for development.
- Populates bodyPose with example values (x, y, z, roll, pitch, yaw).

[Edit]
- Implemented a basic inverse kinematics function for a 3-DOF leg.
- Each leg is assigned a default standing pose based on body height and neutral orientation.
- Added comments to clarify the kinematic assumptions and placeholder logic.

[Edit]
- Implemented simple linear trajectory generation for smooth joint movement.
- Trajectories interpolate from current to desired angles over a fixed duration.
- Added velocity calculation for each joint.
- Updated comments for clarity.

[Edit]
- Implemented a more realistic sendMotorCommand() with value clamping and command formatting.
- Simulates sending commands to a motor driver interface.
- Added comments for clarity and future hardware integration.

[Edit]
- Added basic feedback integration: simulated current joint angles are updated each cycle.
- Trajectory generation now uses the simulated current joint angles for interpolation.
- This enables smooth, incremental movement toward the target pose.
- Added comments to clarify feedback simulation and its role in closed-loop control.

[Edit]
- Implemented a simple trot gait planner.
- Added gait phase tracking and foot trajectory generation for each leg.
- Gait planner updates target foot positions in a cyclic pattern for walking.
- Integrated gait planner with IK and trajectory generation.
- Added comments to clarify gait logic and integration.
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// Constants for robot configuration
#define NUM_LEGS 4
#define JOINTS_PER_LEG 3
    
// Example leg geometry (meters)
#define COXA_LENGTH 0.05f
#define FEMUR_LENGTH 0.10f
#define TIBIA_LENGTH 0.15f
#define DEFAULT_BODY_HEIGHT 0.25f           

// Trajectory timing (seconds)
#define TRAJECTORY_DURATION 0.5f // Move to target in 0.5 seconds

// Motor command limits (example values)
#define JOINT_MIN_ANGLE   (-M_PI/2)
#define JOINT_MAX_ANGLE   (M_PI/2)
#define JOINT_MAX_VELOCITY 2.0f // rad/s

// Gait parameters
#define GAIT_CYCLE_TIME 1.0f // seconds for a full gait cycle
#define GAIT_STEP_HEIGHT 0.05f // meters
#define GAIT_STEP_LENGTH 0.10f // meters

// Data structure for sensor data
typedef struct {
    float bodyPose[6]; // [x, y, z, roll, pitch, yaw]
} SensorData;

// Data structure for joint angles
typedef struct {
    float angle[NUM_LEGS][JOINTS_PER_LEG];
} JointAngles;

// Data structure for joint trajectories
typedef struct {
    float position[NUM_LEGS][JOINTS_PER_LEG];
    float velocity[NUM_LEGS][JOINTS_PER_LEG];
} JointTrajectories;

// Data structure for foot target positions (for gait planning)
typedef struct {
    float x, y, z;
} FootTarget;

static float simulatedCurrentAngles[NUM_LEGS][JOINTS_PER_LEG] = {0};

// Gait phase for each leg (0.0 to 1.0)
static float gaitPhase[NUM_LEGS] = {0.0f, 0.5f, 0.5f, 0.0f}; // Trot: diagonal pairs out of phase

// Gait phase increment per control loop
#define CONTROL_LOOP_HZ 200.0f
#define GAIT_PHASE_INC (1.0f / (GAIT_CYCLE_TIME * CONTROL_LOOP_HZ))

// Default foot positions relative to body center
static FootTarget defaultFootPos[NUM_LEGS] = {
    { 0.15f,  0.10f, -DEFAULT_BODY_HEIGHT}, // Front Left
    { 0.15f, -0.10f, -DEFAULT_BODY_HEIGHT}, // Front Right
    {-0.15f,  0.10f, -DEFAULT_BODY_HEIGHT}, // Rear Left
    {-0.15f, -0.10f, -DEFAULT_BODY_HEIGHT}  // Rear Right
};

// Simulate reading all relevant sensors and return their data
SensorData readSensors() {
    SensorData data;
    data.bodyPose[0] = 0.0f; // x position (meters)
    data.bodyPose[1] = 0.0f; // y position (meters)
    data.bodyPose[2] = DEFAULT_BODY_HEIGHT; // z position (meters, height above ground)
    data.bodyPose[3] = 0.0f; // roll (radians)
    data.bodyPose[4] = 0.0f; // pitch (radians)
    data.bodyPose[5] = 0.0f; // yaw (radians)
    return data;
}

/*
    Simple trot gait planner.
    Updates foot target positions for each leg based on gait phase.
    Swing phase: foot moves forward and up.
    Stance phase: foot moves backward and stays on ground.
*/
void updateGait(FootTarget footTargets[NUM_LEGS]) {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // Advance gait phase
        gaitPhase[leg] += GAIT_PHASE_INC;
        if (gaitPhase[leg] > 1.0f) gaitPhase[leg] -= 1.0f;

        float phase = gaitPhase[leg];
        FootTarget base = defaultFootPos[leg];

        if (phase < 0.5f) {
            // Swing phase: move foot forward and up
            float swing = phase / 0.5f;
            footTargets[leg].x = base.x + (GAIT_STEP_LENGTH / 2.0f) * (1.0f - cosf(M_PI * swing));
            footTargets[leg].z = base.z + GAIT_STEP_HEIGHT * sinf(M_PI * swing);
        } else {
            // Stance phase: move foot backward and keep on ground
            float stance = (phase - 0.5f) / 0.5f;
            footTargets[leg].x = base.x - (GAIT_STEP_LENGTH / 2.0f) * (1.0f - cosf(M_PI * stance));
            footTargets[leg].z = base.z;
        }
        // y position remains constant for this simple gait
        footTargets[leg].y = base.y;
    }
}

/*
    Basic inverse kinematics for a 3-DOF quadruped leg.
    Calculates joint angles for each leg to reach the target foot position.
    Assumes leg is mounted at (0,0,0) in body frame.
*/
JointAngles computeInverseKinematicsGait(const FootTarget footTargets[NUM_LEGS]) {
    JointAngles angles;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float x = footTargets[leg].x;
        float y = footTargets[leg].y;
        float z = footTargets[leg].z;

        // Coxa (hip yaw)
        angles.angle[leg][0] = atan2f(y, x);

        // Project to leg plane
        float legPlaneDist = sqrtf(x * x + y * y) - COXA_LENGTH;
        float d = sqrtf(legPlaneDist * legPlaneDist + z * z);

        // Law of cosines for knee
        float a = FEMUR_LENGTH;
        float b = TIBIA_LENGTH;
        float c = d;
        float knee = acosf((a*a + b*b - c*c) / (2*a*b));
        angles.angle[leg][2] = M_PI - knee;

        // Law of cosines for femur
        float alpha = atan2f(-z, legPlaneDist);
        float beta = acosf((a*a + c*c - b*b) / (2*a*c));
        angles.angle[leg][1] = alpha + beta;
    }
    return angles;
}

/*
    Linear trajectory generation using feedback.
    Interpolates from current to desired angle over TRAJECTORY_DURATION.
    Uses simulatedCurrentAngles as feedback for each joint.
*/
JointTrajectories generateTrajectories(JointAngles desiredAngles) {
    JointTrajectories traj;
    for (int i = 0; i < NUM_LEGS; i++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            float current = simulatedCurrentAngles[i][j];
            float target = desiredAngles.angle[i][j];
            traj.position[i][j] = target;
            traj.velocity[i][j] = (target - current) / TRAJECTORY_DURATION;
        }
    }
    return traj;
}

/*
    Simulate sending a command to a motor driver.
    Clamps position and velocity to safe limits.
    Updates simulatedCurrentAngles to reflect movement toward the target.
    Replace printf and simulation with actual hardware communication.
*/
void sendMotorCommand(int leg, int joint, float position, float velocity) {
    if (position < JOINT_MIN_ANGLE) position = JOINT_MIN_ANGLE;
    if (position > JOINT_MAX_ANGLE) position = JOINT_MAX_ANGLE;
    if (velocity < -JOINT_MAX_VELOCITY) velocity = -JOINT_MAX_VELOCITY;
    if (velocity > JOINT_MAX_VELOCITY) velocity = JOINT_MAX_VELOCITY;

    printf("[CMD] Leg %d, Joint %d | Pos: %.2f rad | Vel: %.2f rad/s\n", leg, joint, position, velocity);

    float step = velocity * (1.0f / CONTROL_LOOP_HZ);
    float error = position - simulatedCurrentAngles[leg][joint];
    if (fabsf(step) > fabsf(error)) {
        simulatedCurrentAngles[leg][joint] = position;
    } else {
        simulatedCurrentAngles[leg][joint] += step;
    }
}

// The main control task (already provided by you)
void controlTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Hz

    // Gait foot targets
    FootTarget footTargets[NUM_LEGS];

    while(1) {
        xLastWakeTime = xTaskGetTickCount();

        // Read sensor data
        SensorData sensorData = readSensors();

        // Update gait and get foot targets
        updateGait(footTargets);

        // Compute inverse kinematics for gait
        JointAngles desiredAngles = computeInverseKinematicsGait(footTargets);

        // Generate trajectories
        JointTrajectories trajectories = generateTrajectories(desiredAngles);

        // Send commands to motor drivers
        for(int i = 0; i < NUM_LEGS; i++) {
            for(int j = 0; j < JOINTS_PER_LEG; j++) {
                sendMotorCommand(i, j, trajectories.position[i][j], trajectories.velocity[i][j]);
            }
        }

        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
