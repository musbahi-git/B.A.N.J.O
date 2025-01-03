class SensorHAL {
public:
    virtual Vector3f getAcceleration() = 0;
    virtual Vector3f getAngularVelocity() = 0;
    virtual Vector3f getMagneticField() = 0;
};

class MPU9250SensorHAL : public SensorHAL {
private:
    MPU9250 mpu;
public:
    MPU9250SensorHAL() {
        Wire.begin();
        mpu.setup(0x68);
    }
    Vector3f getAcceleration() override {
        mpu.update();
        return Vector3f(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ());
    }
    // Implement other methods...
};
