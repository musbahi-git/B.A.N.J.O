void controlTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Hz

    while(1) {
        xLastWakeTime = xTaskGetTickCount();

        // Read sensor data
        SensorData sensorData = readSensors();

        // Compute inverse kinematics
        JointAngles desiredAngles = computeInverseKinematics(sensorData.bodyPose);

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
