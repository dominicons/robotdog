/**
 * Configuration file
 */

/* main loop */
#define LOOP_TIME 20   // milliseconds

// Serial
#define SERIAL_BAUD 115200 

/*
 * I2C devices
 *  - PWM PCA9685
 *  - IMU 10DOF tự viết
 */

#define PWM_CONTROLLER_ADDRESS 0x40
#define PWM_CONTROLLER_TYPE    PCA9685

// Servo channel mapping for PCA9685
#define SERVO_SHOULDER_LF  0  // Vai trước trái
#define SERVO_SHOULDER_RF  1  // Vai trước phải
#define SERVO_SHOULDER_LH  2  // Vai sau trái
#define SERVO_SHOULDER_RH  3  // Vai sau phải

#define SERVO_ELBOW_LF     4  // Khuỷu trước trái
#define SERVO_ELBOW_RF     5  // Khuỷu trước phải
#define SERVO_ELBOW_LH     6  // Khuỷu sau trái
#define SERVO_ELBOW_RH     7  // Khuỷu sau phải

#define SERVO_KNEE_LF      8  // Đầu gối trước trái
#define SERVO_KNEE_RF      9  // Đầu gối trước phải
#define SERVO_KNEE_LH     10  // Đầu gối sau trái
#define SERVO_KNEE_RH     11  // Đầu gối sau phải

#define IMU10DOF_ADDRESS       0x68  // Địa chỉ I2C nếu cần cho IMU của bạn

#define SLOW_SDA             17
#define SLOW_SCL             16


// Robot config
#define LEG_NUM 4

// === KÍCH THƯỚC CÁC PHẦN CHÂN (mm) ===
// Đo từ tâm trục xoay này đến tâm trục xoay tiếp theo
// L1: Vai -> Bắp (xương trên, gắn vào thân)
// L2: Bắp -> Cẳng (xương giữa)
// L3: Cẳng -> Mặt đất (xương dưới)
#define LEG_SIZE_L1   52.8   // mm
#define LEG_SIZE_L2  111.0   // mm
#define LEG_SIZE_L3  104.0   // mm

// === GÓC MẶC ĐỊNH (radian) ===
// Đặt robot đứng thẳng, đo từng khớp so với phương ngang/thẳng đứng
#define LEG_ANGLE_ALPHA M_PI_2 // Góc vai (ra/vào thân)
#define LEG_ANGLE_BETA  M_PI_2 // Góc bắp (gập/thẳng chân)
#define LEG_ANGLE_GAMMA M_PI_2 // Góc cẳng (gập/thẳng cổ chân)

// === GIỚI HẠN GÓC (radian) ===
// Đo từng khớp đến giới hạn vật lý, đổi sang radian (1 rad ≈ 57.3°)
#define LEG_ANGLE_ALPHA_MIN 0
#define LEG_ANGLE_ALPHA_MAX M_PI
#define LEG_ANGLE_BETA_MIN  0
#define LEG_ANGLE_BETA_MAX  M_PI
#define LEG_ANGLE_GAMMA_MIN 0
#define LEG_ANGLE_GAMMA_MAX M_PI

// === VỊ TRÍ GẮN CHÂN TRÊN THÂN ROBOT (mm) ===
// Đo từ tâm robot (hoặc board mạch) đến từng trục vai theo X, Y, Z
#define LEG_BODY_X      82.0   // Khoảng cách từ tâm robot đến vai (trục X)
#define LEG_BODY_Y_F   143.0   // Khoảng cách từ tâm robot đến vai trước (trục Y)
#define LEG_BODY_Y_H    95.0   // Khoảng cách từ tâm robot đến vai sau (trục Y)
#define LEG_BODY_Z       0.0   // Độ cao trục vai so với mặt đất (nếu có)

// === VỊ TRÍ MẶC ĐỊNH CỦA CHÂN (mm) ===
// Đặt robot đứng thẳng, đo từ trục vai đến điểm tiếp đất của mỗi chân
#define LEG_POINT_X    118.0   // Khoảng cách từ vai đến điểm tiếp đất (trục X)
#define LEG_POINT_Y_F  143.0   // Khoảng cách từ vai trước đến điểm tiếp đất (trục Y)
#define LEG_POINT_Y_H   95.0   // Khoảng cách từ vai sau đến điểm tiếp đất (trục Y)
#define LEG_POINT_Z    275.0   // Độ cao từ vai đến điểm tiếp đất (trục Z)

#define LEG_TRIM_INC   0.002   // radian
#define LEG_TRIM_LIMIT  0.24   // See settingsUint8ToDouble()

//Fais safe
#define FS_WS_THR 20  // 1 second = FS_WS_THR*LOOP_TIME
