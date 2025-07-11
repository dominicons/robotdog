// ...existing code...
#include "config.h"
#include <Adafruit_PWMServoDriver.h>
#include "robot_control.h"  
#include <Arduino.h>
#include "robot_globals.h"
// Tích hợp IMU
#include "IMU10DOF.h"
IMU10DOF imu;

void setup() {
    pwm.begin();
    pwm.setPWMFreq(50); // 50Hz cho servo
    Serial.begin(SERIAL_BAUD);
    imu.begin();
    // Khởi tạo phần cứng, cảm biến, v.v. nếu cần
    // ...
}

void loop() {
    // Hiệu ứng stepping: 2 cặp chân chéo luân phiên nâng/hạ beta, gamma
    static int stepState = 0; // 0-3: stepping tổng quát
    static unsigned long lastStepTime = 0;
    const int stepInterval = 400; // ms mỗi trạng thái lớn
    const double stepDelta = 25.0 * M_PI / 180.0; // 25 độ sang radian

    if (millis() - lastStepTime > stepInterval) {
        lastStepTime = millis();
        // Góc chuẩn
        double betaLF = LEG_ANGLE_BETA, betaRF = LEG_ANGLE_BETA, betaLH = LEG_ANGLE_BETA, betaRH = LEG_ANGLE_BETA;
        double gammaLF = LEG_ANGLE_GAMMA, gammaRF = LEG_ANGLE_GAMMA, gammaLH = LEG_ANGLE_GAMMA, gammaRH = LEG_ANGLE_GAMMA;

        switch (stepState) {
            case 0: // Bước 1: LF+RH (trước) +25, RF+LH (sau) -25
                betaLF  += stepDelta; gammaLF  += stepDelta; // LF trước
                betaRF  += stepDelta; gammaRF  += stepDelta; // RH trước
                betaRH  -= stepDelta; gammaRH  -= stepDelta; // RF sau
                betaLH  -= stepDelta; gammaLH  -= stepDelta; // LH sau
                break;
            case 1: // Bước 2: tất cả về 0
                // giữ nguyên góc chuẩn
                break;
            case 2: // Bước 3: RF+LH (trước) +25, LF+RH (sau) -25
                betaRH  += stepDelta; gammaRH  += stepDelta; // RF trước
                betaLH  += stepDelta; gammaLH  += stepDelta; // LH trước
                betaLF  -= stepDelta; gammaLF  -= stepDelta; // LF sau
                betaRF  -= stepDelta; gammaRF  -= stepDelta; // RH sau
                break;
            case 3: // Bước 4: tất cả về 0
                // giữ nguyên góc chuẩn
                break;
        }
        // Gửi lệnh servo: vai giữ nguyên, chỉ cẳng và khuỷu thay đổi
        pwm.setPWM(SERVO_SHOULDER_LF, 0, angleToPulse(LEG_ANGLE_ALPHA));
        pwm.setPWM(SERVO_SHOULDER_RF, 0, angleToPulse(LEG_ANGLE_ALPHA));
        pwm.setPWM(SERVO_SHOULDER_LH, 0, angleToPulse(LEG_ANGLE_ALPHA));
        pwm.setPWM(SERVO_SHOULDER_RH, 0, angleToPulse(LEG_ANGLE_ALPHA));

        pwm.setPWM(SERVO_ELBOW_LF, 0, angleToPulse(betaLF));
        pwm.setPWM(SERVO_ELBOW_RF, 0, angleToPulse(betaRF));
        pwm.setPWM(SERVO_ELBOW_LH, 0, angleToPulse(betaLH));
        pwm.setPWM(SERVO_ELBOW_RH, 0, angleToPulse(betaRH));

        pwm.setPWM(SERVO_KNEE_LF, 0, angleToPulse(gammaLF));
        pwm.setPWM(SERVO_KNEE_RF, 0, angleToPulse(gammaRF));
        pwm.setPWM(SERVO_KNEE_LH, 0, angleToPulse(gammaLH));
        pwm.setPWM(SERVO_KNEE_RH, 0, angleToPulse(gammaRH));

        stepState = (stepState + 1) % 4;
    }
    delay(10);
}
