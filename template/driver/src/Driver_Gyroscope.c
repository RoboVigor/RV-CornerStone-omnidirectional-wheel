#define __DRIVER_GYROSCOPE_GLOBALS
#include "math.h"
#include "Driver_Filter.h"
#include "config.h"
#include "MadgwickAHRS.h"
#include "Driver_Gyroscope.h"
#include "arm_math.h"

static float          rollAngle;
static float          pitchAngle;
static float          yawAngle;
static float          xSpeed;
static float          ySpeed;
static float          zSpeed;
static float          xAcc;
static float          yAcc;
static float          zAcc;
static float          xMag;
static float          yMag;
static float          zMag;
extern volatile float beta;
static int16_t        debug_pitch = 0;

double yawoffset_add=0;

Filter_Type Filter_Yaw = {.count = 0, .thresholdLB = GYROSCOPE_YAW_FILTER_THRESHOLD};

extern ImuData_Type ImuData;

void Gyroscope_Init(GyroscopeData_Type *GyroscopeData, uint16_t startupDelay) {
    GyroscopeData->startupCounter = 0;
    GyroscopeData->modification = 0;
#ifdef STM32F427_437xx
    MPU6500_Initialize();
    MPU6500_EnableInt();
#endif
#ifdef STM32F40_41xxx
    while (BMI088_init()) {
    }
    ist8310_init();
#endif
    if (startupDelay != 0) {
#ifdef STM32F427_437xx
        beta = 5;
        while (1) {
            LED_Set_Progress(GyroscopeData->startupCounter / (startupDelay / 7) + 1);
            if (GyroscopeData->startupCounter >= startupDelay) {
                beta = 0.1;
                break;
            }
        }
#endif
#ifdef STM32F40_41xxx
        beta = 5;
        while (1) {
            LED_Set_Colour(GyroscopeData->startupCounter / startupDelay * 255, 0, 0);
            if (GyroscopeData->startupCounter >= startupDelay) {
                beta = 0.5;
                break;
            }
        }
#endif
    }
}

// MPU6500数据读取,成功返回1  失败返回0
int Gyroscope_Update(GyroscopeData_Type *GyroscopeData) {
#ifdef STM32F427_437xx
    static uint8_t mpu_buf[20];

    //尝试读取数据
    if (IIC_ReadData(MPU_IIC_ADDR, MPU6500_ACCEL_XOUT_H, mpu_buf, 14) == 0xff) return 0;

    //成功的话进行赋值
    ImuData.temp = (((int16_t) mpu_buf[6]) << 8) | mpu_buf[7];
    ImuData.az = (((int16_t) mpu_buf[4]) << 8) | mpu_buf[5];
    ImuData.gz = ((((int16_t) mpu_buf[12]) << 8) | mpu_buf[13]) - ImuData.gz_bias;
    ImuData.ax = (((int16_t) mpu_buf[0]) << 8) | mpu_buf[1];
    ImuData.ay = (((int16_t) mpu_buf[2]) << 8) | mpu_buf[3];
    ImuData.gx = ((((int16_t) mpu_buf[8]) << 8) | mpu_buf[9]) - ImuData.gx_bias;
    ImuData.gy = ((((int16_t) mpu_buf[10]) << 8) | mpu_buf[11]) - ImuData.gy_bias;
#endif
#ifdef STM32F40_41xxx
    static uint8_t buf[8];

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    ImuData.ax = (int16_t)((buf[1] << 8) | buf[0]);
    ImuData.ay = (int16_t)((buf[3] << 8) | buf[2]);
    ImuData.az = (int16_t)((buf[5] << 8) | buf[4]);

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        ImuData.gx = (int16_t)((buf[3] << 8) | buf[2]);
        ImuData.gy = (int16_t)((buf[5] << 8) | buf[4]);
        ImuData.gz = (int16_t)((buf[7] << 8) | buf[6]);
    }

    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    if ((int16_t)((buf[0] << 3) | (buf[1] >> 5)) > 1023) {
        ImuData.temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5)) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    } else {
        ImuData.temp = ((int16_t)((buf[0] << 3) | (buf[1] >> 5)) - 2048) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    ist8310_IIC_read_muli_reg(0x03, buf, 6);
    ImuData.mx = (int16_t)((buf[1] << 8) | buf[0]);
    ImuData.my = (int16_t)((buf[3] << 8) | buf[2]);
    ImuData.mz = (int16_t)((buf[5] << 8) | buf[4]);
#endif

    // 读取完成进行解算
    Gyroscope_Solve(GyroscopeData);

    // 返回成功值
    return 1;
}

void Gyroscope_Solve(GyroscopeData_Type *GyroscopeData) {
    float ImuData_temp[3][3];
    ImuData_temp[0][0] = (float) ((ImuData.gx / GYROSCOPE_LSB) * PI / 180.0);
    ImuData_temp[0][1] = (float) ((ImuData.gy / GYROSCOPE_LSB) * PI / 180.0);
    ImuData_temp[0][2] = (float) ((ImuData.gz / GYROSCOPE_LSB) * PI / 180.0);
    ImuData_temp[1][0] = (float) (ImuData.ax / ACCELERATE_LSB);
    ImuData_temp[1][1] = (float) (ImuData.ay / ACCELERATE_LSB);
    ImuData_temp[1][2] = (float) (ImuData.az / ACCELERATE_LSB);
	
	
#ifdef STM32F40_41xxx
    ImuData_temp[2][0] = (float) (ImuData.mx / MAGNETIC_LSB);
    ImuData_temp[2][1] = (float) (ImuData.my / MAGNETIC_LSB);
    ImuData_temp[2][2] = (float) (ImuData.mz / MAGNETIC_LSB);
#endif

    //坐标系转换
    Gyroscope_axis_trans(ImuData_temp);

    // GD算法或Madgwick算法,梯度算法,网上开源
#ifdef STM32F427_437xx
    MadgwickAHRSupdateIMU(xSpeed, ySpeed, zSpeed, xAcc, yAcc, zAcc);
#endif
#ifdef STM32F40_41xxx
    MadgwickAHRSupdate(xSpeed, ySpeed, zSpeed, xAcc, yAcc, zAcc, xMag, yMag, zMag);
#endif

    // 四元数->欧拉角
    rollAngle = atan2(2.0f * (q0 * q1 + q2 * q3), 1 - 2*(q2*q2 + q3*q3)) * 180 / PI;
    pitchAngle  = asin(2.0f * (q0 * q2 - q1 * q3)) * 180 / PI;
    yawAngle   = atan2(2.0f * (q1 * q2 + q0 * q3), 1 - 2*(q1*q1 + q2*q2)) * 180 / PI;

    // 更新滤波器
    Filter_Update(&Filter_Yaw, yawAngle);

    // 计算连续 Yaw 角
    if (Filter_Yaw.diff > 300) {
        Filter_Yaw.offset -= 360;
    } else if (Filter_Yaw.diff < -300) {
        Filter_Yaw.offset += 360;
    }

    //前馈
    GyroscopeData->modification += GYROSCOPE_YAW_MODIFICATION;

    // 应用滤波
    GyroscopeData->yaw = Filter_Apply_Limit_Breadth(&Filter_Yaw) + GyroscopeData->yawoffset            //+GyroscopeData->modification;重复零飘修正，在滤波中已有零漂限幅补偿

    // 开机时yaw轴转动角度补偿，用于正式启动时的yaw轴零点确定
    #if GYROSCOPE_START_UP_DELAY_ENABLED
    if (GyroscopeData->startupCounter == GYROSCOPE_START_UP_DELAY - 1) {
        GyroscopeData->yawoffset = -GyroscopeData->yaw;
    }
    #endif

    GyroscopeData->pitch = -pitchAngle;  //在右手坐标系z轴指向天空下，pitch向上转时为负数，此处加负号以符合人的直觉
    GyroscopeData->roll  = rollAngle;
    debug_pitch          = GyroscopeData->pitch;

    // 开机延迟计数
#if GYROSCOPE_START_UP_DELAY_ENABLED
    if (GyroscopeData->startupCounter < GYROSCOPE_START_UP_DELAY) {
        GyroscopeData->startupCounter += 1;
    }
#endif
}

float Gyroscope_Get_Filter_Diff(void) {
    return Filter_Yaw.diff;
}

void Gyroscope_Set_Bias(ImuData_Type *ImuData, int16_t gx_bias, int16_t gy_bias, int16_t gz_bias) {
    ImuData->gx_bias = gx_bias;
    ImuData->gy_bias = gy_bias;
    ImuData->gz_bias = gz_bias;
}

void Gyroscope_axis_trans( float *ImuData_temp) {
    float transMatrix[9] = trans_matrix;
    arm_matrix_instance_f32 matrixTrans;
    arm_mat_init_f32(&matrixTrans, 3, 3, (float32_t *)transMatrix);
    #ifdef STM32F427_437xx
        arm_matrix_instance_f32 ImuData_Src;
        arm_matrix_instance_f32 ImuData_Dst;
        float ImuData_trans[2][3];
        arm_mat_init_f32(&ImuData_Src, 2, 3, (float32_t *)ImuData_temp);
        arm_mat_init_f32(&ImuData_Dst, 2, 3, (float32_t *)ImuData_trans);
        arm_mat_mult_f32(&ImuData_Src, &matrixTrans, &ImuData_Dst);
        xSpeed = ImuData_trans[0][0];
        ySpeed = ImuData_trans[0][1];
        zSpeed = ImuData_trans[0][2];
        xAcc = ImuData_trans[1][0];
        yAcc = ImuData_trans[1][1];
        zAcc = ImuData_trans[1][2];
    #endif

    #ifdef STM32F40_41xxx
        arm_matrix_instance_f32 ImuData_Src;
        arm_matrix_instance_f32 ImuData_Dst;
        float ImuData_trans[3][3];
        arm_mat_init_f32(&ImuData_Src, 3, 3, (float32_t *)ImuData_temp);
        arm_mat_init_f32(&ImuData_Dst, 3, 3, (float32_t *)ImuData_trans);
        arm_mat_mult_f32(&ImuData_Src, &matrixTrans, &ImuData_Dst);
        xSpeed = ImuData_trans[0][0];
        ySpeed = ImuData_trans[0][1];
        zSpeed = ImuData_trans[0][2];
        xAcc = ImuData_trans[1][0];
        yAcc = ImuData_trans[1][1];
        zAcc = ImuData_trans[1][2];
        xMag = ImuData_trans[2][0];
        yMag = ImuData_trans[2][1];
        zMag = ImuData_trans[2][2];
    #endif 
}