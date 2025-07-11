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
    static int stepState = 0; // 0-15: stepping chi tiết
    static unsigned long lastStepTime = 0;
    const int stepInterval = 200; // ms mỗi trạng thái nhỏ
    const double stepDelta = 25.0 * M_PI / 180.0; // 25 độ sang radian

    if (millis() - lastStepTime > stepInterval) {
        lastStepTime = millis();
        // Góc chuẩn
        double betaLF = LEG_ANGLE_BETA, betaRF = LEG_ANGLE_BETA, betaLH = LEG_ANGLE_BETA, betaRH = LEG_ANGLE_BETA;
        double gammaLF = LEG_ANGLE_GAMMA, gammaRF = LEG_ANGLE_GAMMA, gammaLH = LEG_ANGLE_GAMMA, gammaRH = LEG_ANGLE_GAMMA;

        switch (stepState) {
            // LF+RH nâng lên
            case 0: // LF+RH khuỷu lên +25
                gammaLF += stepDelta; gammaRH += stepDelta;
                break;
            case 1: // LF+RH cẳng lên +25
                gammaLF += stepDelta; gammaRH += stepDelta;
                betaLF  += stepDelta;  betaRH  += stepDelta;
                break;
            case 2: // về 0
                break;
            case 3: // về 0
                break;
            // LF+RH hạ xuống
            case 4: // LF+RH khuỷu xuống -25
                gammaLF -= stepDelta; gammaRH -= stepDelta;
                break;
            case 5: // LF+RH cẳng xuống -25
                gammaLF -= stepDelta; gammaRH -= stepDelta;
                betaLF  -= stepDelta;  betaRH  -= stepDelta;
                break;
            case 6: // về 0
                break;
            case 7: // về 0
                break;
            // RF+LH nâng lên
            case 8: // RF+LH khuỷu lên +25
                gammaRF += stepDelta; gammaLH += stepDelta;
                break;
            case 9: // RF+LH cẳng lên +25
                gammaRF += stepDelta; gammaLH += stepDelta;
                betaRF  += stepDelta;  betaLH  += stepDelta;
                break;
            case 10: // về 0
                break;
            case 11: // về 0
                break;
            // RF+LH hạ xuống
            case 12: // RF+LH khuỷu xuống -25
                gammaRF -= stepDelta; gammaLH -= stepDelta;
                break;
            case 13: // RF+LH cẳng xuống -25
                gammaRF -= stepDelta; gammaLH -= stepDelta;
                betaRF  -= stepDelta;  betaLH  -= stepDelta;
                break;
            case 14: // về 0
                break;
            case 15: // về 0
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

        stepState = (stepState + 1) % 16;
    }
    delay(10);
}
