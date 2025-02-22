#include "Driver_Chassis.h"
#include "macro.h"
#include "config.h"
#include "handle.h"
#include "vegmath.h"
#include "math.h"

void Chassis_Init(ChassisData_Type *cd) {
    cd->vx                 = 0;
    cd->vy                 = 0;
    cd->vw                 = 0;
    cd->power              = 0;
    cd->referencePower     = 0;
    cd->lastReferencePower = 0;
    cd->targetPower        = 80;
    cd->maxPower           = 80;
    cd->maxPowerBuffer     = 60;
    cd->powerScale         = 1;
    PID_Init(&(cd->PID_Power), 1, 0, 0, 500, 10);
}

void Chassis_Update(ChassisData_Type *cd, float vx, float vy, float vw) {
    cd->vx = vx;
    cd->vy = vy;
    cd->vw = vw;
}

void Chassis_Updata_FT(ChassisData_Type * cd, float Fx, float Fy, float T){
    cd->Fx = Fx;
    cd->Fy = Fy;
    cd->T  = T;
}

void Chassis_Fix(ChassisData_Type *cd, float angle) {
    float sinYaw = vegsin(angle); // 1
    float cosYaw = vegcos(angle); // 0
    float vx     = cd->vx;
    float vy     = cd->vy;
    cd->vy       = vy * cosYaw - vx * sinYaw;
    cd->vx       = vy * sinYaw + vx * cosYaw;
}
//此处运动学逆解已修改为全向轮地盘
void Chassis_Calculate_Rotor_Speed(ChassisData_Type *cd) {
    float coefficient = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE;
    cd->rotorSpeed[0] = coefficient * ((cd->vy - cd->vx)*0.707f + cd->vw * CHASSIS_RADIUS);
    cd->rotorSpeed[1] = coefficient * ((cd->vy + cd->vx)*0.707f + cd->vw * CHASSIS_RADIUS);
    cd->rotorSpeed[2] = -coefficient * ((cd->vy - cd->vx)*0.707f + cd->vw * CHASSIS_RADIUS);
    cd->rotorSpeed[3] = -coefficient * ((cd->vy + cd->vx)*0.707f + cd->vw * CHASSIS_RADIUS);
}
//此处为运动学正结算
void Chassis_Calculate_Real_Speed(ChassisData_Type *cd, int16_t * motor_Speed) {
    float coefficient = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE;
    cd->realvx = (-motor_Speed[0]-motor_Speed[1]+motor_Speed[2]+motor_Speed[3])*0.354f/coefficient;
    cd->realvy = (motor_Speed[0]-motor_Speed[1]-motor_Speed[2]+motor_Speed[3])*0.354f/coefficient;
    cd->realvw = (motor_Speed[0]+motor_Speed[1]+motor_Speed[2]+motor_Speed[3])*0.25f/coefficient/CHASSIS_RADIUS;
}
//此处为动力学逆解算
void Chassis_Calculate_Rotor_Torgue(ChassisData_Type *cd){
    cd->rotorTorgue[0] = ((-cd->Fx + cd->Fy)*0.354f + cd->T * 0.25f / CHASSIS_RADIUS) / CHASSIS_INVERSE_WHEEL_RADIUS;
    cd->rotorTorgue[1] = ((-cd->Fx - cd->Fy)*0.354f + cd->T * 0.25f / CHASSIS_RADIUS) / CHASSIS_INVERSE_WHEEL_RADIUS;
    cd->rotorTorgue[2] = ((cd->Fx - cd->Fy)*0.354f + cd->T * 0.25f / CHASSIS_RADIUS) / CHASSIS_INVERSE_WHEEL_RADIUS;
    cd->rotorTorgue[3] = ((cd->Fx + cd->Fy)*0.354f + cd->T * 0.25f / CHASSIS_RADIUS) / CHASSIS_INVERSE_WHEEL_RADIUS;
}

void Chassis_Limit_Rotor_Speed(ChassisData_Type *cd, float maxRotorSpeed) {
    float   maxSpeed = 0;
    float   scale    = 0;
    int16_t absSpeed = 0;
    uint8_t i        = 0;

    // 打擂台获得麦轮速度最大值
    for (; i < 4; i++) {
        absSpeed = ABS(cd->rotorSpeed[i]);
        if (absSpeed > maxSpeed) {
            maxSpeed = absSpeed;
        }
    }

    // 进行限幅
    if (maxSpeed > maxRotorSpeed) {
        scale = maxRotorSpeed / maxSpeed;
        Chassis_Scale_Rotor_Speed(cd, scale);
    }
}

void Chassis_Scale_Rotor_Speed(ChassisData_Type *cd, float scale) {
    cd->rotorSpeed[0] = cd->rotorSpeed[0] * scale;
    cd->rotorSpeed[1] = cd->rotorSpeed[1] * scale;
    cd->rotorSpeed[2] = cd->rotorSpeed[2] * scale;
    cd->rotorSpeed[3] = cd->rotorSpeed[3] * scale;
}

void Chassis_Limit_Power(ChassisData_Type *cd, float targetPower, float referencePower, float referencePowerBuffer, float interval) {
    PID_Type *PID_Power = &(cd->PID_Power);

    // 更新状态
    cd->targetPower    = targetPower;
    cd->referencePower = referencePower;
    cd->interval       = interval;

    // 功率拟合
    if (cd->referencePower != cd->lastReferencePower) {
        // 更新功率及缓冲
        cd->power              = cd->referencePower;
        cd->lastReferencePower = cd->referencePower;
        cd->powerBuffer        = referencePowerBuffer;
        // 重置拟合和缩放
        cd->fittingCounter = 0;
        cd->powerScale     = 1;
    } else {
        float stable;
        float ePow;
        cd->fittingCounter++;
        stable = cd->powerScale * cd->power;
        ePow   = pow(2.71828, -(cd->interval) * (1.0 / 0.04)); // 40
        // ePow      = pow(2.71828, -(cd->interval) * ((float) cd->fittingCounter) / 0.035); // 40
        cd->power = stable + (cd->power - stable) * ePow;
        // 模拟功率缓冲
        cd->powerBuffer -= (cd->referencePower - cd->maxPower) * cd->interval;
        MIAO(cd->powerBuffer, 0, cd->maxPowerBuffer);
    }

    // if (referencePowerBuffer < 30) {
    //     cd->powerBuffer = referencePowerBuffer;
    // }

    // 功率PID
    PID_Calculate(PID_Power, cd->targetPower, cd->power);
    cd->powerScale = (cd->power + PID_Power->output) / cd->power;
    MIAO(cd->powerScale, 0, 1);
    // Chassis_Scale_Rotor_Speed(cd, cd->powerScale);
}

void Chassis_Current_Output_Integrate(float *motorCurrentOutput, ChassisData_Type* cd){
    for(int i =0; i<4; i++){
        motorCurrentOutput[i] += (cd->rotorTorgue[i]*3.333f) ;
    }
}

void Chassis_Calculate_Power_Limit(float* motorCurrentOutput, int16_t* MCO_With_PowerLimit, float *realMotorSpeed, float targetPower){
    float coefficient[6] = SECOND_MACLAURIN_COEFFICIENT;
    float totalPower = 0;
    for(int i =0; i < 4; i++) {
        totalPower += (coefficient[0] \
                       + coefficient[1]*motorCurrentOutput[i] \
                       + coefficient[2]*realMotorSpeed[i] \
                       + coefficient[3]*motorCurrentOutput[i]*realMotorSpeed[i] \
                       + coefficient[4]*motorCurrentOutput[i]*motorCurrentOutput[i] \
                       + coefficient[5]*realMotorSpeed[i]*realMotorSpeed[i] \
                    );
    }

    if(totalPower <= targetPower) {
        for(int i=0; i<4; i++){
            MCO_With_PowerLimit[i] = motorCurrentOutput[i] * CurrentMap_C620_Inverse;
        }
    }
    else{
        float a =0, b =0, c =0, ETA =0;
        for(int i =0; i < 4; i++){
            a += (coefficient[4]*motorCurrentOutput[i]*motorCurrentOutput[i]);
        }
        for(int i =0; i <4; i++){
            b += (coefficient[3]*motorCurrentOutput[i]*realMotorSpeed[i] + coefficient[1]*motorCurrentOutput[i]);
        }
        for(int i=0; i< 4; i++){
            c += (coefficient[0] + coefficient[5]*realMotorSpeed[i]*realMotorSpeed[i]);
        }
        if(!a == 0){
            float DELTA, DELTA_SQRT;
            DELTA = b*b - 4*a*c;
            if(DELTA >= 0){
                arm_sqrt_f32(DELTA, &DELTA_SQRT);
                float root1 = (-b + DELTA_SQRT)/(2*a);
                float root2 = (-b - DELTA_SQRT)/(2*a);
                if(root1 >0 && root1 <1 && root2 >0 && root2 <1){
                    ETA = root1 > root2? root1 : root2;
                }else if (root1 >0 && root1 <1)
                {
                    ETA = root1;
                }else if (root2 >0 && root2 <1)
                {
                    ETA = root2;
                }
            }
        }
        for(int i=0; i<4; i++){
            MCO_With_PowerLimit[i] = motorCurrentOutput[i] * ETA * CurrentMap_C620_Inverse;
        }
    }
}