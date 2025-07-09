#pragma once
#include <Wire.h>

class IMU10DOF {
public:
    IMU10DOF();
    bool begin();
    void update(float dt);

    float getPitch();
    float getRoll();
    float getYaw();

private:
    float pitch, roll, yaw;
    bool readAccelerometer(float &ax, float &ay, float &az);
    bool readGyroscope(float &gx, float &gy, float &gz);
    
    // Kalman filter variables
    struct KalmanFilter {
        float angle;      // The angle calculated by the Kalman filter
        float bias;       // The gyro bias calculated by the Kalman filter
        float P[2][2];    // Error covariance matrix
        float Q_angle;    // Process noise variance for the accelerometer
        float Q_bias;     // Process noise variance for the gyro bias
        float R_measure;  // Measurement noise variance
        float dt;         // Time step
    };
    
    KalmanFilter kalmanPitch, kalmanRoll;
    void initKalman(KalmanFilter &kalman);
    float kalmanUpdate(KalmanFilter &kalman, float newAngle, float newRate, float dt);
};
