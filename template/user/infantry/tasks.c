/* @brief 任务*/
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "math.h"

int qian = 0 ,zuo = 0 ;

void Task_Control(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY); 
    TickType_t LastWakeTime = xTaskGetTickCount();
    //LASER_ON;

    while (1) {
        ControlMode = LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_BOTTOM ? 2 : 1;
        if (ControlMode == 1) {
            //遥控器模式
            // PsAimEnabled  = LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP;
            //FastmoveMode  = LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP;
            MagzineOpened = LEFT_SWITCH_MIDDLE && RIGHT_SWITCH_TOP;
            FrictEnabled  = 1;
            StirEnabled   = LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_TOP;
					
            // unused
            // FastShootMode = StirEnabled;
            PsShootEnabled = 0;
            SwingMode     = LEFT_SWITCH_MIDDLE && RIGHT_SWITCH_MIDDLE;
            SafetyMode = LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP;
        } else if (ControlMode == 2) {
            //键鼠模式
            PsShootEnabled = 0;
            StirEnabled    = mouseData.pressLeft;
            PsAimEnabled   = mouseData.pressRight;
            //摩擦轮
            if (keyboardData.G && !keyboardData.Ctrl) {
                FrictEnabled = 1;
            } else if (keyboardData.G && keyboardData.Ctrl) {
                FrictEnabled = 0;
                Key_Disable(&keyboardData, KEY_G, 100);
            }
					FrictEnabled = 1;
            // 弹舱盖
            MagzineOpened = keyboardData.F;
            // 小陀螺
            if (keyboardData.C) {
                SwingMode = 1;
            } 
            if(keyboardData.V){
                SwingMode = 0;
            }
            // 高射速模式
            if(keyboardData.E) ShootMode = fastShoot;
            //高速移动模式(关闭底盘功率上限，飞坡用)
            if(keyboardData.Shift) moveMode = fastMove;
        }
        // 调试视觉用
        // FrictEnabled   = (remoteData.switchLeft == 2) || (remoteData.switchLeft == 1) && (remoteData.switchRight != 2);
        // PsAimEnabled   = (remoteData.switchLeft == 1) && (remoteData.switchRight != 3);
        // PsShootEnabled = (remoteData.switchLeft == 1) && (remoteData.switchRight == 1);
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

void Task_Can_Send(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY); 
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms
    while (1) {
        Bridge_Send_Motor(&BridgeData, SafetyMode);
        vTaskDelayUntil(&LastWakeTime, intervalms); // 发送频率
    }
    vTaskDelete(NULL);
}

void Task_Gimbal(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY); 
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int16_t    intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 反馈值
    float yawAngle, yawSpeed, pitchAngle, pitchSpeed, chassisAngle, motorYawSpeed;

    // 目标值
    float pitchAngleTarget          = 0; // 目标Pitch
    float yawAngleTarget            = 0; // 目标Yaw
  
    float pitchT = 0;
    float yawAngleTargetControl     = 0; // 遥控器输入
    float pitchAngleTargetControl   = 0; // 遥控器输入
    float pitchAngleTargetFix       = 0; // 上坡补偿
    float pitchAngleTargetFixStable = 0; // 上坡补偿
    float yawAngleTargetPs          = 0; // 视觉辅助
    float pitchAngleTargetPs        = 0; // 视觉辅助
    int8_t pitchInit                  = 0; // pitch启动初始校准

    // 输出量
    int16_t yawCurrent   = 0;
    int16_t pitchCurrent = 0;

    // Pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = Gyroscope_EulerData.pitch;
    float pitchAngleTargetRamp = 0;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 7, 1, 0, 4000, 10);
    PID_Init(&PID_Cloud_YawSpeed, 270, 2, 0, 23000, 40);
    PID_Init(&PID_Cloud_PitchAngle, 10, 0, 0, 16000, 1000);
    PID_Init(&PID_Cloud_PitchSpeed, 65, 1, 5,  20000, 16000);
    PID_Init(&PID_Cloud_MotorYawSpeed, 3, 1, 0, 23000, 0);

    while (1) {
        // 重置目标
        yawAngleTargetControl = 0;
        pitchAngleTargetControl = 0;

        // 设置反馈
        yawAngle     = Gyroscope_EulerData.yaw;    // 逆时针为正
        yawSpeed     = Gyroscope_EulerData.yawSpeed;      // 逆时针为正
        pitchAngle   = Gyroscope_EulerData.pitch;  // 
        pitchSpeed   = abs((int)Gyroscope_EulerData.pitchSpeed) > 20 ? Motor_Pitch.speed : Gyroscope_EulerData.pitchSpeed; // 
        chassisAngle = - Motor_Pitch.angle + pitchAngle; //计算底盘的旋转角度，此处要求一点，电机角度和imu要保持一致
        motorYawSpeed = Motor_Yaw.speed*RPM2RPS;

        // 遥控器输入角度目标
		if(ControlMode==1)
		{
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl = -remoteData.rx / 660.0f * 360 * interval * 0.4;
        if (ABS(remoteData.ry) > 30) pitchAngleTargetControl = remoteData.ry / 660.0f * 360 * interval * 0.1;
		}
		else if (ControlMode==2)
		{
        yawAngleTargetControl = -mouseData.x * 0.5 * interval; // 0.005
        pitchAngleTargetControl = mouseData.y * 0.3 * interval;
		}
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;


        // 视觉辅助
        yawAngleTargetPs = HostAutoaimData.yaw_angle_diff;
        pitchAngleTargetPs = HostAutoaimData.pitch_angle_diff;
        if (PsAimEnabled) {
             yawAngleTarget = yawAngle + yawAngleTargetPs;
             pitchAngleTarget = pitchAngle + pitchAngleTargetPs;
        }

        // 限制云台运动范围即斜坡补偿
        MIAO(pitchAngleTarget, GIMBAL_PITCH_MIN + chassisAngle, GIMBAL_PITCH_MAX + chassisAngle);;  

        // 开机时pitch轴匀速抬起
        if(!pitchInit){
            pitchAngleTarget = RAMP(pitchRampStart, 0, pitchRampProgress);
            if (pitchRampProgress < 1) {
            pitchRampProgress += 0.005f;  
            }else {
                Motor_Set_Angle_Bias(&Motor_Pitch, Motor_Pitch.angle);
                pitchInit = 1;
            }
        }



        // if(!Motor_Pitch.online){
        //     pitchRampProgress = 0;
        //     pitchRampStart = Gyroscope_EulerData.pitch;
        //     pitchInit = 0;
        // }
 
        // 计算PID
        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, Gyroscope_EulerData.yaw);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);
        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTarget, pitchAngle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchSpeed);
        PID_Calculate(&PID_Cloud_MotorYawSpeed, -1 *ChassisData.realvw, motorYawSpeed);


        // 输出电流
        if(SwingMode){
            yawCurrent = PID_Cloud_YawSpeed.output;
            // yawCurrent = (1 - pow(2.71828, -2.23*abs(PID_Cloud_YawAngle.error)))*PID_Cloud_YawSpeed.output + pow(2.71828, -2.23*abs(PID_Cloud_YawAngle.error)) * PID_Cloud_MotorYawSpeed.output;   //动态权重融合
        }else{
            yawCurrent = PID_Cloud_YawSpeed.output;
        }
        pitchCurrent = PID_Cloud_PitchSpeed.output; //-8500 * cos((pitchAngle * PI /180.0f))
        Motor_Yaw.input   = yawCurrent;
        Motor_Pitch.input = pitchCurrent;

        //任务间隔
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY); 
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 底盘运动
    float vx             = 0;
    float vy             = 0;
    float vw             = 0;
    float vwRamp         = 0;
    float vwRampProgress = 0;
    float targetPower    = 0;
    float motorCurrentOutput[4];
    int16_t MCO_With_PowerLimit[4];

    // 反馈值
    float motorAngle, motorSpeed;
    float lastMotorAngle = Motor_Yaw.angle;
    float filter[6]      = {0, 0, 0, 0, 0, 0};
    int   filterp        = 0;
    float power          = 0;
    float powerBuffer    = 0;
    float realMotorSpeed[4] = {0,0,0,0};

    // 小陀螺
    float   swingSpeed       = 0;
    uint8_t swingModeEnabled = 0;

    // 底盘跟随PID
    float followDeadRegion = 3.0;
    PID_Init(&PID_Follow_Angle, 0.5, 0.1, 0, 100, 50);
    PID_Init(&PID_Follow_Speed, 2, 0, 1, 100, 0);

    // 速度环PID
    PID_Init(&PID_LFCM, 1, 0, 0, 6000, 0);
    PID_Init(&PID_LBCM, 1, 0, 0, 6000, 0);
    PID_Init(&PID_RBCM, 1, 0, 0, 6000, 0);
    PID_Init(&PID_RFCM, 1, 0, 0, 6000, 0);

    //扭矩前馈PID
    PID_Init(&PID_Fx, 4, 0, 0.5, 0, 0);
    PID_Init(&PID_Fy, 4, 0, 0.5, 0, 0);
    PID_Init(&PID_T, 5, 0, 0.5, 0, 0);

    //闭环功率PID
    PID_Init(&PID_Power, 0.5, 0, 0, 40, 0);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    // 底盘运动斜坡函数
    float xRampProgress = 0;
    float xRampStart    = 0;
    float xTargetRamp   = 0;
    float yRampProgress = 0;
    float yRampStart    = 0;
    float yTargetRamp   = 0;

    if(fricEnabled) moveMode = frictMove;

    while (1) {
        // 设置反馈值
        motorAngle  = Motor_Yaw.angle;                                 // 电机角度
        motorSpeed  = Motor_Yaw.speed * RPM2RPS;                       // 电机角速度
        power       = ProtocolData.powerHeatData.chassis_power;        // 裁判系统功率
        powerBuffer = ProtocolData.powerHeatData.chassis_power_buffer; // 裁判系统功率缓冲
        targetPower = ProtocolData.gameRobotstatus.chassis_power_limit;// 裁判系统功率限制
        realMotorSpeed[0] = Motor_LF.speed *RPM2RPS;
        realMotorSpeed[1] = Motor_LB.speed *RPM2RPS;
        realMotorSpeed[2] = Motor_RB.speed *RPM2RPS;
        realMotorSpeed[3] = Motor_RF.speed *RPM2RPS;

        // 视觉专属follow PID
        if (PsAimEnabled) {
            PID_Follow_Angle.p = 1.5;
        } else {
            PID_Follow_Angle.p = 1.3;
        }

        //小陀螺模式
        switch (SwingMode)
        {
        case 1:
            swingSpeed = 80 * RPM2RPS;
            swingModeEnabled = 1;
            break;
        
        default:
            swingModeEnabled = 0;
            break;
        }

        // 设置底盘总体移动速度
        vx = 0;
        vy = 0;
        vw = 0;
        if (ControlMode == 1) {
			vy = -remoteData.lx / 660.0f * 6.0;
			vx = remoteData.ly / 660.0f * 6.0;

        } else if (ControlMode == 2) {
            xTargetRamp = RAMP(xRampStart, 660, xRampProgress);
            if (xRampProgress <= 0.5) {
                xRampProgress += 0.1f;
            } else if (xRampProgress > 0.5 && xRampProgress < 1) {
                xRampProgress += 0.05f;
            }
            yTargetRamp = RAMP(yRampStart, 660, yRampProgress);
            if (yRampProgress <= 0.5) {
                yRampProgress += 0.1f;
            } else if (yRampProgress > 0.5 && yRampProgress < 1) {
                yRampProgress += 0.05f;
            }
			vy = (keyboardData.A - keyboardData.D) * xTargetRamp / 660.0f * 12;
			vx = (keyboardData.W - keyboardData.S) * yTargetRamp / 660.0f * 8;
		}            

        if (keyboardData.W == 0 && keyboardData.S == 0) {
            yRampProgress = 0;
            yRampStart    = 0;
        }
        if (keyboardData.A == 0 && keyboardData.D == 0) {
            xRampProgress = 0;
            xRampStart    = 0;
        }
    
        //运动学正解算底盘真实速度
        Chassis_Calculate_Real_Speed(&ChassisData, realMotorSpeed);

        //地盘跟随和小陀螺
        if(!swingModeEnabled){
            //底盘跟随云台
            vw += Gyroscope_EulerData.yawSpeed * DPS2RPS * 0.75; //前馈
            int fbAngle1 = (((int)motorAngle % 360) > 0 ? (((int)motorAngle % 360) - 360): (((int)motorAngle % 360)  +  360));
            int fbAngle2 = ((int)motorAngle % 360);
            int fbAngle =abs(((int)motorAngle % 360) / 180) ? fbAngle1: fbAngle2;
            PID_Calculate(&PID_Follow_Angle, 0, -fbAngle);
            if(abs((int)PID_Follow_Angle.error) > followDeadRegion) {
                PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output, ChassisData.realvw); //此处本质是为了让已有速度前馈下还存在的error进行一个速度小补偿，此处的速度环pid仅作scalar的作用并无反馈
                // if(Motor_Yaw.online) {
                    vw += PID_Follow_Speed.output * DPS2RPS;
                // }else {
                    // vw = 0;
                // }
            }
            // VofaData->debug1 = 0;
            // VofaData->debug2 = fbAngle;
            // VofaData->debug3 = fbAngle;
            // VofaData->debug4 = abs(((int)motorAngle % 360) / 180);
            // VofaData->debug5 = PID_Follow_Speed.output;
            // VofaData->debug6 = abs((int)PID_Follow_Angle.error);
        }else{
            // 小陀螺
            vw = swingSpeed;
        }

        // 开机时底盘匀速回正
        vwRamp = RAMP(0, vw, vwRampProgress);
        if (vwRampProgress < 1) {
            vwRampProgress += 0.002f;
        }

        switch (moveMode)
        {
        case frictMove:
            PID_Calculate(&PID_Power, 40, ProtocolData.powerHeatData.chassis_power_buffer);
            targetPower = ProtocolData.gameRobotstatus.chassis_power_limit - PID_Power.output;
            break;
        
        case fastMove:
            targetPower = 360;
            break;

        default:
            targetPower = 80;
            break;
        }

        Chassis_Update(&ChassisData, vx, vy, vwRamp); // 更新麦轮转速
        Chassis_Fix(&ChassisData, motorAngle);        // 修正旋转后底盘的前进方向
        Chassis_Calculate_Rotor_Speed(&ChassisData);  // 麦轮解算

        PID_Calculate(&PID_Fx, ChassisData.vx, ChassisData.realvx);
        PID_Calculate(&PID_Fy, ChassisData.vy, ChassisData.realvy);
        PID_Calculate(&PID_T, vwRamp, ChassisData.realvw);
        Chassis_Updata_FT(&ChassisData, PID_Fx.output, PID_Fy.output, PID_T.output);
        Chassis_Calculate_Rotor_Torgue(&ChassisData);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);
        motorCurrentOutput[0] = PID_LFCM.output * CurrentMap_C620;
        motorCurrentOutput[1] = PID_LBCM.output * CurrentMap_C620;
        motorCurrentOutput[2] = PID_RBCM.output * CurrentMap_C620;
        motorCurrentOutput[3] = PID_RFCM.output * CurrentMap_C620;

        Chassis_Current_Output_Integrate(motorCurrentOutput, &ChassisData);
        Chassis_Calculate_Power_Limit(motorCurrentOutput, MCO_With_PowerLimit, realMotorSpeed, targetPower);

        // 输出电流值到电调 功率限制已修改完成
        Motor_LF.input = MCO_With_PowerLimit[0];
        Motor_LB.input = MCO_With_PowerLimit[1];    
        Motor_RB.input = MCO_With_PowerLimit[2];
        Motor_RF.input = MCO_With_PowerLimit[3];

        // 调试信息
        // VofaData->debug1 = vwRamp;
        // VofaData->debug2 = ChassisData.rotorSpeed[0];
        // VofaData->debug3 = realMotorSpeed[0];
        // VofaData->debug4 = ChassisData.realvw;
        // VofaData->debug5 = ChassisData.Fx;
        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Host(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY); 
    TickType_t         LastWakeTime = xTaskGetTickCount();
    ProtocolInfo_Type *protocolInfo;
    int16_t            lastReceiveSeq = 0;
    int64_t            sinceReceive;
    float      interval     = 0.1;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    uint16_t targetPower = 0;
    uint16_t refereePower = 0;
    uint8_t sendBuffer[8] = {0,0,0,0,0x12,0x20,0x12,0x07};
    while (1) {

        //SuperCap
        if(fricEnabled){
        targetPower =  ProtocolData.gameRobotstatus.chassis_power_limit - 10; //留点余量；
        }else
        {
            targetPower = 50;
        }
        refereePower = ProtocolData.gameRobotstatus.chassis_power_limit;
        sendBuffer[0] = targetPower&0xff;
        sendBuffer[1] = targetPower >> 8;
        sendBuffer[2] = refereePower & 0xff;
        sendBuffer[3] = refereePower >> 8;
        Can_Send_Msg(CAN1, 0x4ff, sendBuffer, 8);

        // transmit
        ProtocolData.gyroscopeData.pitch = Gyroscope_EulerData.pitch;
        ProtocolData.gyroscopeData.yaw   = Gyroscope_EulerData.yaw;
        ProtocolData.gyroscopeData.roll  = Gyroscope_EulerData.roll;
        memcpy(ProtocolData.dbusData.dbusBuffer, remoteBuffer, 19);

        // receive autoaim data
        protocolInfo = Protocol_Get_Info_Handle(0x401);
        if (protocolInfo->lastReceiveSeq != protocolInfo->receiveSeq) {
            memcpy(HostAutoaimData.data, ProtocolData.autoaimData.data, protocolInfo->length);
            protocolInfo->lastReceiveSeq = protocolInfo->receiveSeq;
        } else {
            memset(HostAutoaimData.data, 0, protocolInfo->length);
        }
        FacingEnemyMode = HostAutoaimData.yaw_angle_diff != 0 || HostAutoaimData.pitch_angle_diff != 0;
        // DebugData.debug5 = protocolInfo->receiveSeq;

        // // receive chassis data
        // protocolInfo = Protocol_Get_Info_Handle(0x402);
        // sinceReceive = xTaskGetTickCount() - protocolInfo->receiveTime;
        // if (sinceReceive > 0 && sinceReceive < 200) {
        //     memcpy(HostChassisData.data, ProtocolData.chassisData.data, protocolInfo->length);
        // } else {
        //     memset(HostChassisData.data, 0, protocolInfo->length);
        // }

        // debug
        // DebugData.debug1 = HostAutoaimData.yaw_angle_diff * 1000;
        // DebugData.debug2 = ProtocolData.autoaimData.yaw_angle_diff * 1000;
        // DebugData.debug3 = HostAutoaimData.pitch_angle_diff*1000;
        // DebugData.debug4 = ProtocolData.autoaimData.pitch_angle_diff*1000;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

 void Task_UI(void *Parameters) {
     TickType_t LastWakeTime  = xTaskGetTickCount();
     uint8_t    isInitialized = 0;
     while (1) {
         //if (isInitialized) {
            // ProtocolData.client_custom_delete
            // Bridge_Send_Protocol_Once();
         //}
         ProtocolData.client_custom_graphicSingle.data_cmd_id = 0x102;
         ProtocolData.client_custom_graphicSingle.send_id     = ProtocolData.gameRobotstatus.robot_id;
         ProtocolData.client_custom_graphicSingle.receiver_id = 0x100 + ProtocolData.gameRobotstatus.robot_id;
         // graphic 1
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.graphic_name[0] = 'p';
         //ProtocolData.client_custom_graphicSingle.grapic_data_struct.operate_type       = 1;
         //ProtocolData.client_custom_graphicSingle.grapic_data_struct.graphic_type       = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.layer              = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.color              = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.start_angle        = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.end_angle          = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.width              = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.start_x            = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.start_y            = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.radius             = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.end_x              = 1;
         ProtocolData.client_custom_graphicSingle.grapic_data_struct.end_y              = 1;
         vTaskDelayUntil(&LastWakeTime, 20);
     }
     vTaskDelete(NULL);
 }

/**
 * @brief 发射机构 (拨弹轮)
 */

void Task_Fire_Stir(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY); 
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;
		float targetSpeed = 0;	// 任务运行间隔 ms

    // 射击模式
    enum shootMode_e { shootIdle = 0, shootToDeath }; // 停止, 连发
    enum shootMode_e shootMode = shootIdle;

    // 热量控制
    int   shootNum        = 0;
    int   mayShootNum     = 0;
    int   maxBulletSpeed  = 0;
    float lastBulletSpeed = 0;
    float maxShootHeat    = 0;
    int   stirSpeed       = 0;
    int   stirAngle       = 0;

    // 视觉系统
    int16_t lastSeq = 0;

    // PID 初始化
    PID_Init(&PID_StirAngle, 1, 0, 0, 9000, 6000);  // 拨弹轮角度环
    PID_Init(&PID_StirSpeed, 10, 0, 0, 6000, 1000); // 拨弹轮速度环
	
	// TEST
//	PID_Init(&PID_FireL, 3, 0, 0, 16384, 1000);
//	PID_Init(&PID_FireR, 3, 0, 0, 16384, 1000);

    // 开启激光
    // LASER_ON;

    while (1) {
        // 弹舱盖开关
        if (ROBOT_MIAO) {
            PWM_Set_Compare(&PWM_Magazine_Servo, MagzineOpened ? 10 : 5);
        } else if (ROBOT_WANG) {
            PWM_Set_Compare(&PWM_Magazine_Servo, MagzineOpened ? 16 : 6);
        } else if (ROBOT_SHARK) {
            PWM_Set_Compare(&PWM_Magazine_Servo, MagzineOpened ? 6 : 15);
        }
        // 拨弹速度
        stirSpeed = 110;
        if (ProtocolData.gameRobotstatus.shooter_id1_17mm_cooling_rate == 20) {
            stirSpeed = 110;
        } else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_cooling_rate == 30) {
            stirSpeed = 140;
        } else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_cooling_rate == 40) {
            stirSpeed = 160;
        }
        stirSpeed * 3;

        // stirSpeed = 143; // 热量：120
        // stirSpeed = 120; // 热量：240
        // stirSpeed = 120; // 热量：360

        // X模式

        //热量控制
        maxShootHeat = ProtocolData.gameRobotstatus.shooter_id1_17mm_cooling_limit - ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit * 2;

        // 输入射击模式
        shootMode = shootIdle;

        if (StirEnabled) {
            shootMode = shootToDeath;
        }
        // 视觉辅助
        // if (PsShootEnabled && lastSeq != Ps.autoaimData.seq && Ps.autoaimData.biu_biu_state) {
        //     shootMode = shootToDeath;
        // }
        // lastSeq = Ps.autoaimData.seq;

        if (ProtocolData.powerHeatData.shooter_id1_17mm_cooling_heat > maxShootHeat) {
            shootMode = shootIdle;
        }
		
		
		
        // 控制拨弹轮
        if (shootMode == shootIdle) {
            // 停止
            stirSpeed = 0;
//						targetSpeed = 0;
//						PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
//						PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
//						Motor_FL.input = PID_FireL.output;
//						Motor_FR.input = PID_FireR.output;

        } else if (shootMode == shootToDeath) {
            // 连发
            stirSpeed = 330;
            if(mouseData.pressRight){
                stirSpeed = 400;
            }
//						targetSpeed = 1000;
//						PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
//						PID_Calculate(&PID_FireR, targetSpeed, Motor_FR.speed);
//					
//						Motor_FL.input = -1*PID_FireL.output;
//						Motor_FR.input = 1*PID_FireR.output;
        }
		
		// stirSpeed=0;
		
        PID_Calculate(&PID_StirSpeed, stirSpeed, Motor_Stir.speed * RPM2RPS);
        Motor_Stir.input = PID_StirSpeed.output;
		

        // DebugData.debug1 = PID_StirSpeed.output;
        // DebugData.debug2 = shootMode;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Fire_Frict(void *Parameters) {
    xEventGroupWaitBits(InitEventGroup, INIT_EVENT_ALL, pdFALSE, pdTRUE, portMAX_DELAY);    
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    float motorLSpeed;
    float motorRSpeed;
    float targetSpeed = 0;
	
	    // 射击模式
    enum shootMode_e { shootIdle = 0, shootToDeath }; // 停止, 连发
    enum shootMode_e shootMode = shootIdle;

    PID_Init(&PID_FireL, 3, 0, 0, 16384, 1200);
    PID_Init(&PID_FireR, 3, 0, 0, 16384, 1200);

    while (1) {

        if (FrictEnabled) {
            LASER_ON;
        } else {
            LASER_OFF;
        }

        if (1) {
            if (ROBOT_MIAO) {
								//targetSpeed = 4000;
                if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 15)
                    targetSpeed = 4000;
                else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 18)
                    targetSpeed = 5000;
                else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 22)
                    targetSpeed = 7000;
                else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 30)
                    targetSpeed = 1000;								
//						PID_Calculate(&PID_FireL, -1*targetSpeed, Motor_FL.speed);
//            PID_Calculate(&PID_FireR, targetSpeed, Motor_FR.speed);
            } else if (ROBOT_WANG) {
				//targetSpeed = 1500;
                if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 15)
                    targetSpeed = 4500;
                else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 18)
                    targetSpeed = 4000;
                else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 22)
                    targetSpeed = 4000;
                else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 30)
                    targetSpeed = 4000;
            } else if (ROBOT_SHARK) { //还没测
							targetSpeed = 10000;
//                if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 15)
//                    targetSpeed = 4450;
//                else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 18)
//                    targetSpeed = 5100;	
//                else if (ProtocolData.gameRobotstatus.shooter_id1_17mm_speed_limit == 30)
//                    targetSpeed = 7700;
							
            }
//			if(ROBOT_SHARK)
//				{
//					PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
//					PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
//				}else if(ROBOT_MIAO)
//				{
//					PID_Calculate(&PID_FireL, -1*targetSpeed, Motor_FL.speed);
//					PID_Calculate(&PID_FireR, targetSpeed, Motor_FR.speed);
//				}	
//			
        } 
//		if (StirEnabled) {
//            shootMode = shootToDeath;
//        }
//		if (shootMode == shootIdle) {
//            // 停止
//						targetSpeed = 0;
//						PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
//						PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
//						Motor_FL.input = PID_FireL.output;
//						Motor_FR.input = PID_FireR.output;

//        } else if (shootMode == shootToDeath) {
//            // 连发
//			PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
//			PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
//			Motor_FL.input = PID_FireL.output;
//			Motor_FR.input = PID_FireR.output;
//        }
		targetSpeed = -4000;
		PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
		PID_Calculate(&PID_FireR, -2*targetSpeed, Motor_FR.speed);
		Motor_FL.input = PID_FireL.output;
		Motor_FR.input = PID_FireR.output;
		
				/*else {
            targetSpeed = 4450;
					  PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
            PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
					
        }
        if (ROBOT_SHARK) {
            PID_Calculate(&PID_FireL, targetSpeed, Motor_FL.speed);
            PID_Calculate(&PID_FireR, -1*targetSpeed, Motor_FR.speed);
        } else {
            PID_Calculate(&PID_FireL, -1*targetSpeed, Motor_FL.speed);
            PID_Calculate(&PID_FireR, -1 * targetSpeed, Motor_FR.speed);
        }*/
		

        // DebugData.debug1 = Motor_FL.speed;
        // DebugData.debug2 = targetSpeed;
        // DebugData.debug3 = PID_FireR.output;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) 
	{
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        LED_Run_Horse_XP(); // XP开机动画,建议延时200ms
        // LED_Run_Horse(); // 跑马灯,建议延时20ms
        vTaskDelayUntil(&LastWakeTime, 200);
    }

    vTaskDelete(NULL);
}

void Task_Startup_Music(void *Parameters){
	TickType_t LastWakeTime = xTaskGetTickCount();
	while (1)
	{
			if (KTV_Play(Music_Earth)) break;
			vTaskDelayUntil(&LastWakeTime, 120);
	}
	vTaskDelete(NULL);
}

void Task_Wait(void *Parameters)
{
	TickType_t LastWakeTime = xTaskGetTickCount();	
	
	while(1)//老6躲到基地后开陀螺代码，2023上理RoboVigor哨兵核心科技
	{
//		if (ProtocolData.gameStatus.game_progress == 3)//裁判系统信号控制
//		//if (LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_BOTTOM)//遥控器控制（双朝下开始）
//		{
//			delay_ms(6000);//delay3000毫秒，等裁判系统（！！！最后改成6000！！！）
//			//SwingMode = 6;//测试裁判系统信号用的陀螺开启代码，常为被注释状态
//			while(1){qian=27; zuo=0; vTaskDelay(4000); break;}//向前速度（可为负，建议绝对值20-40）、向左速度、持续时间（毫秒）
//			qian=0; zuo=0;//向前向左速度初始化
//			while(1){qian=0; zuo=-25; vTaskDelay(2700); break;}//向前速度（可为负，建议绝对值20-40）、向左速度、持续时间（毫秒）
//			qian=0; zuo=0;//向前、向左速度初始化
//			while(1){qian=-20; zuo=0; vTaskDelay(600); break;}//向前速度（可为负，建议绝对值20-40）、向左速度、持续时间（毫秒）
//			qian=0; zuo=0;//向前向左速度初始化
//			while(1){qian=0; zuo=0; vTaskDelay(200); break;}//向前速度（可为负，建议绝对值20-40）、向左速度、持续时间（毫秒）
//			qian=0; zuo=0;//向前向左速度初始化
//				while(1){
//				SwingMode = 3;//小陀螺6，大陀螺3
//				vTaskDelay(2300);
//				SwingMode = 0;//小陀螺6，大陀螺3
//				vTaskDelay(500);
//				SwingMode = 5;//小陀螺6，大陀螺3
//				vTaskDelay(2300);
//				if (LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP){SwingMode = 0;break;}//遥控器控制（双朝上停止陀螺）
//				}
//		}
		if (LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP){SwingMode = 0;}//遥控器控制（双朝上停止陀螺）

		
		vTaskDelayUntil(&LastWakeTime,120);
	}
	vTaskDelete(NULL);
}

void Task_IWDG(void *Parameters){
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1)
    {
        //IWDG
        BSP_IWDG_Feed();
        vTaskDelayUntil(&LastWakeTime,intervalms);
        /* code */
    }
	vTaskDelete(NULL);
}

void Task_Imu(void *Parameters){
    ImuDataReady = xSemaphoreCreateBinary();
    configASSERT(ImuDataReady != NULL);
    BSP_IMU_Init();
    Gyroscope_Init(&Gyroscope_EulerData, 0);
    uint16_t counter = 1000;
    while (1)
    {   
        if(xSemaphoreTake(ImuDataReady,portMAX_DELAY) == pdTRUE){
            Gyroscope_Update(&Gyroscope_EulerData);
            if (!counter)
            {
                continue;
            }
            
        }

        if(!(--counter)){
            xEventGroupSetBits(InitEventGroup, INIT_EVENT_IMU);
        }
    }
}