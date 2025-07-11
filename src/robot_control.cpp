#include <Arduino.h>
#include "robot_globals.h"
#include "config.h"
#include <Adafruit_PWMServoDriver.h>
#include "robot_control.h"
// Hàm chuyển đổi góc radian sang xung PWM cho PCA9685
int angleToPulse(double angleRad) {
    int minPulse = 500, maxPulse = 2500;
    double angleDeg = angleRad * 180.0 / M_PI;
    int pulse = minPulse + (int)((maxPulse - minPulse) * (angleDeg / 180.0));
    return (int)(pulse * 4096 / 20000);
}

// Hàm tripod gait chuyên nghiệp cho robot 4 chân
void robotTripodWalk(int numSteps, double stepLength, int stepDelay) {
    const int subSteps = 20;
    const double liftHeight = 30.0;
    for (int step = 0; step < numSteps; ++step) {
        // Pha 1: LF + RH nhấc lên, RF + LH stance
        for (int t = 0; t <= subSteps; ++t) {
            double progress = (double)t / subSteps;
            // LF
            point fromLF = legLF.foot;
            point toLF = fromLF; toLF.x += stepLength; toLF.z -= liftHeight;
            transitionParameters tpLF = {fromLF, toLF, liftHeight};
            transition trLF; trLF.set(tpLF);
            point lfPos = trLF.swing(progress);
            // RH
            point fromRH = legRH.foot;
            point toRH = fromRH; toRH.x += stepLength; toRH.z -= liftHeight;
            transitionParameters tpRH = {fromRH, toRH, liftHeight};
            transition trRH; trRH.set(tpRH);
            point rhPos = trRH.swing(progress);
            // stance
            point rfPos = legRF.foot;
            point lhPos = legLH.foot;
            // cập nhật
            legLF.foot = lfPos;
            legRF.foot = rfPos;
            legLH.foot = lhPos;
            legRH.foot = rhPos;
            // IK
            IK ikLF(legLF, body); ikLF.solve();
            IK ikRF(legRF, body); ikRF.solve();
            IK ikLH(legLH, body); ikLH.solve();
            IK ikRH(legRH, body); ikRH.solve();
            // Servo
            pwm.setPWM(SERVO_SHOULDER_LF, 0, angleToPulse(legLF.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_LF,    0, angleToPulse(legLF.angle.beta));
            pwm.setPWM(SERVO_KNEE_LF,     0, angleToPulse(legLF.angle.gamma));
            pwm.setPWM(SERVO_SHOULDER_RF, 0, angleToPulse(legRF.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_RF,    0, angleToPulse(legRF.angle.beta));
            pwm.setPWM(SERVO_KNEE_RF,     0, angleToPulse(legRF.angle.gamma));
            pwm.setPWM(SERVO_SHOULDER_LH, 0, angleToPulse(legLH.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_LH,    0, angleToPulse(legLH.angle.beta));
            pwm.setPWM(SERVO_KNEE_LH,     0, angleToPulse(legLH.angle.gamma));
            pwm.setPWM(SERVO_SHOULDER_RH, 0, angleToPulse(legRH.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_RH,    0, angleToPulse(legRH.angle.beta));
            pwm.setPWM(SERVO_KNEE_RH,     0, angleToPulse(legRH.angle.gamma));
            delay(stepDelay);
        }
        // Pha 2: RF + LH nhấc lên, LF + RH stance
        for (int t = 0; t <= subSteps; ++t) {
            double progress = (double)t / subSteps;
            // RF
            point fromRF = legRF.foot;
            point toRF = fromRF; toRF.x += stepLength; toRF.z -= liftHeight;
            transitionParameters tpRF = {fromRF, toRF, liftHeight};
            transition trRF; trRF.set(tpRF);
            point rfPos = trRF.swing(progress);
            // LH
            point fromLH = legLH.foot;
            point toLH = fromLH; toLH.x += stepLength; toLH.z -= liftHeight;
            transitionParameters tpLH = {fromLH, toLH, liftHeight};
            transition trLH; trLH.set(tpLH);
            point lhPos = trLH.swing(progress);
            // stance
            point lfPos = legLF.foot;
            point rhPos = legRH.foot;
            // cập nhật
            legLF.foot = lfPos;
            legRF.foot = rfPos;
            legLH.foot = lhPos;
            legRH.foot = rhPos;
            // IK
            IK ikLF(legLF, body); ikLF.solve();
            IK ikRF(legRF, body); ikRF.solve();
            IK ikLH(legLH, body); ikLH.solve();
            IK ikRH(legRH, body); ikRH.solve();
            // Servo
            pwm.setPWM(SERVO_SHOULDER_LF, 0, angleToPulse(legLF.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_LF,    0, angleToPulse(legLF.angle.beta));
            pwm.setPWM(SERVO_KNEE_LF,     0, angleToPulse(legLF.angle.gamma));
            pwm.setPWM(SERVO_SHOULDER_RF, 0, angleToPulse(legRF.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_RF,    0, angleToPulse(legRF.angle.beta));
            pwm.setPWM(SERVO_KNEE_RF,     0, angleToPulse(legRF.angle.gamma));
            pwm.setPWM(SERVO_SHOULDER_LH, 0, angleToPulse(legLH.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_LH,    0, angleToPulse(legLH.angle.beta));
            pwm.setPWM(SERVO_KNEE_LH,     0, angleToPulse(legLH.angle.gamma));
            pwm.setPWM(SERVO_SHOULDER_RH, 0, angleToPulse(legRH.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_RH,    0, angleToPulse(legRH.angle.beta));
            pwm.setPWM(SERVO_KNEE_RH,     0, angleToPulse(legRH.angle.gamma));
            delay(stepDelay);
        }
    }
}



#include "robot_control.h"
#include "leg.h"
#include "geometry.h"
#include "gait.h"
#include "planner.h"
#include "IK.h"
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

extern Adafruit_PWMServoDriver pwm;

// Giả sử các đối tượng này được khai báo ở đây hoặc ở nơi khác trong chương trình
extern leg legLF, legRF, legLH, legRH;
extern figure body;
extern gaitConfig walkGaitConfigLF, walkGaitConfigRF, walkGaitConfigLH, walkGaitConfigRH;

void robotStandStill() {
    // Đặt tất cả các chân về vị trí mặc định
    legLF.foot = legLF.defaultFoot;
    legRF.foot = legRF.defaultFoot;
    legLH.foot = legLH.defaultFoot;
    legRH.foot = legRH.defaultFoot;
    // Có thể thêm IK và gửi lệnh servo ở đây nếu cần
}

void robotWalkForward() {
    // Khởi tạo các đối tượng cần thiết
    gait gaitLF(walkGaitConfigLF, legLF);
    gait gaitRF(walkGaitConfigRF, legRF);
    gait gaitLH(walkGaitConfigLH, legLH);
    gait gaitRH(walkGaitConfigRH, legRH);

    IK ikLF(legLF, body);
    IK ikRF(legRF, body);
    IK ikLH(legLH, body);
    IK ikRH(legRH, body);

    // Giả sử planner đã được cấu hình phù hợp
    moveVector mv = { {10, 0, 0}, {0, 0, 0} }; // Tiến về phía trước 10mm mỗi bước
    planner plan(mv, body, legLF, legRF, legLH, legRH);

    // Số bước cho mỗi chu kỳ đi
    const int steps = 20;
    for (int i = 0; i < steps; ++i) {
        // Dự đoán vị trí tiếp theo
        plan.predictPosition(1);

        // Lấy vị trí mục tiêu cho từng chân
        point nextLF = plan.getLegPosition(LEGLF);
        point nextRF = plan.getLegPosition(LEGRF);
        point nextLH = plan.getLegPosition(LEGLH);
        point nextRH = plan.getLegPosition(LEGRH);

        // Di chuyển từng chân theo chu kỳ gait
        gaitLF.start(legLF.foot, nextLF);
        gaitRF.start(legRF.foot, nextRF);
        gaitLH.start(legLH.foot, nextLH);
        gaitRH.start(legRH.foot, nextRH);

        // Lặp lại cho đến khi chân đến vị trí mới
        for (int t = 0; t < 10; ++t) {
            gaitLF.next();
            gaitRF.next();
            gaitLH.next();
            gaitRH.next();

            // Tính toán IK cho từng chân
            ikLF.solve();
            ikRF.solve();
            ikLH.solve();
            ikRH.solve();

            // Hàm chuyển đổi góc radian sang xung PWM cho PCA9685
            auto angleToPulse = [](double angleRad) {
                int minPulse = 500, maxPulse = 2500;
                double angleDeg = angleRad * 180.0 / M_PI;
                int pulse = minPulse + (int)((maxPulse - minPulse) * (angleDeg / 180.0));
                return (int)(pulse * 4096 / 20000);
            };

            // Gửi lệnh servo cho từng khớp của từng chân
            pwm.setPWM(SERVO_SHOULDER_LF, 0, angleToPulse(legLF.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_LF,    0, angleToPulse(legLF.angle.beta));
            pwm.setPWM(SERVO_KNEE_LF,     0, angleToPulse(legLF.angle.gamma));

            pwm.setPWM(SERVO_SHOULDER_RF, 0, angleToPulse(legRF.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_RF,    0, angleToPulse(legRF.angle.beta));
            pwm.setPWM(SERVO_KNEE_RF,     0, angleToPulse(legRF.angle.gamma));

            pwm.setPWM(SERVO_SHOULDER_LH, 0, angleToPulse(legLH.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_LH,    0, angleToPulse(legLH.angle.beta));
            pwm.setPWM(SERVO_KNEE_LH,     0, angleToPulse(legLH.angle.gamma));

            pwm.setPWM(SERVO_SHOULDER_RH, 0, angleToPulse(legRH.angle.alpha));
            pwm.setPWM(SERVO_ELBOW_RH,    0, angleToPulse(legRH.angle.beta));
            pwm.setPWM(SERVO_KNEE_RH,     0, angleToPulse(legRH.angle.gamma));

            // Gửi lệnh servo ở đây nếu cần
            delay(20); // delay nhỏ để mô phỏng thời gian thực
        }
    }
}

