/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
//#define VISION_USE_UART // 使用串口发送视觉数据
#define ROBOT_USE_REMOTE_TEST_MODE 0 // 0=USB虚拟串口控制底盘；1=天地飞遥控
#define ROBOT_HOST_ONLY_SPEECH 1 // 关闭下位机自动播报，由上位机负责 RFID 语音
#define ROBOT_ENABLE_HOST_TTS_COMMAND 1 // 接收上位机 TTS 文本并通过 UART7 播报
#define CHASSIS_SPEED_LOOP_TUNING_MODE 0 // 0=启用增量式BMI088航向环
#define CHASSIS_JLINK_TUNING_MODE 0    // 架空J-Link电机标定入口，地面测试必须为0
#define CHASSIS_USE_INS 0             // 旧INS任务不参与本车导航控制
#define CHASSIS_USE_BMI088_YAW_HOLD 1 // 直行测试启用BMI088航向角环

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 4082 // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 1 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 30           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE -30           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 两轮差速底盘实测参数,单位为mm(毫米)
#define CHASSIS_LENGTH 133.0f       // 整车外廓长度，当前记录值
#define CHASSIS_WIDTH 142.0f        // 轮外距/整车外廓宽度
#define WHEEL_WIDTH 23.0f           // 单侧轮宽
#define TRACK_WIDTH (CHASSIS_WIDTH - WHEEL_WIDTH) // 左右轮中心距=119 mm
/* Empirical host angular-command calibration from the 2026-10-12 19:21
 * sweep: wheel travel followed its target, but gyro yaw rate was ~1.13x.
 * Keep physical geometry separate; verify absolute angle on the next run. */
#define CHASSIS_TURN_RATE_SCALE 0.885f
/* Normal zero commands retain velocity control while braking, then disable
 * after both wheels stay still. Invalid input/stale feedback bypass braking. */
#define CHASSIS_STOP_SETTLE_MS 200U
#define CHASSIS_STOP_BRAKE_TIMEOUT_MS 1500U
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 30.2f // 轮胎外径保持实测前的60.4 mm
/* 2026-10-11 最新标定：左右轮各做 3 次、每次转 10 圈。 */
#define MOTOR_ENCODER_COUNTS_PER_REV 8192.0f
#define LEFT_ENCODER_COUNTS_PER_WHEEL_REV 10232.7667f
#define RIGHT_ENCODER_COUNTS_PER_WHEEL_REV 10235.2333f
#define LEFT_REDUCTION_RATIO_WHEEL \
    (LEFT_ENCODER_COUNTS_PER_WHEEL_REV / MOTOR_ENCODER_COUNTS_PER_REV)
#define RIGHT_REDUCTION_RATIO_WHEEL \
    (RIGHT_ENCODER_COUNTS_PER_WHEEL_REV / MOTOR_ENCODER_COUNTS_PER_REV)
/* Rebase the old degree/s tuning to the measured DM wheel reference scale. */
#define CHASSIS_REFERENCE_SCALE \
    (3.14159265359f / (180.0f * \
     ((LEFT_REDUCTION_RATIO_WHEEL + RIGHT_REDUCTION_RATIO_WHEEL) * 0.5f)))
#define REAL_WZ_RAT   1000
#define CHASSIS_YAW_HOLD_DIR (-1.0f)    // 当前左正/右负安装方向下，负航向误差需给正w纠偏
#define CHASSIS_YAW_HOLD_KP (2.00f * CHASSIS_REFERENCE_SCALE)
#define CHASSIS_YAW_HOLD_KI (0.003f * CHASSIS_REFERENCE_SCALE)
#define CHASSIS_YAW_HOLD_KD (0.05f * CHASSIS_REFERENCE_SCALE)
#define CHASSIS_YAW_HOLD_DEADBAND 0.15f // 小角度死区，减少测速抖动
#define CHASSIS_YAW_HOLD_ENCODER_ASSIST_GAIN 0.0f  // 编码器相位只记录，不抵消BMI088航向误差
#define CHASSIS_YAW_HOLD_ENCODER_ASSIST_MAX_DEG 12.0f
#define CHASSIS_YAW_HOLD_FUSED_ERROR_MAX_DEG 12.0f
#define CHASSIS_YAW_HOLD_MAX_RATIO 0.05f
#define CHASSIS_YAW_HOLD_MAX_REF (250.0f * CHASSIS_REFERENCE_SCALE)
#define CHASSIS_YAW_HOLD_I_MAX_RATIO 0.003f
#define CHASSIS_YAW_HOLD_I_MAX_REF (100.0f * CHASSIS_REFERENCE_SCALE)
/* Wheel phase sync is a secondary correction.  Limit how much it can
 * oppose an active BMI088 heading correction. */
#define CHASSIS_YAW_HOLD_OPPOSING_SYNC_RATIO 0.25f
#define CHASSIS_WHEEL_SYNC_KP 0.45f
#define CHASSIS_WHEEL_SYNC_KI 0.10f
#define CHASSIS_WHEEL_SYNC_MAX_RATIO 0.08f
/* The DM2325 velocity field and its unwrapped position field did not agree
 * in the latest straight test.  Use the encoder phase rate for the sync
 * loop, with a small position term to pull a slowly accumulated phase error
 * back toward zero. */
#define CHASSIS_WHEEL_SYNC_PHASE_POSITION_KP 0.12f
#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH -1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL -1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反
#define HUBS_NUMBER 2 // 两轮差速底盘


// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移  //普通模式
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_DEBUG_MODE,
    GIMBAL_NAV_MODE
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;


typedef struct
{ // 功率控制
    float chassis_power_mx;
    float motor_current_up[HUBS_NUMBER]; // 电机电流正上限
    float motor_current_down[HUBS_NUMBER]; // 电机电流负上限
} Chassis_Power_Data_s;


/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float v;           // Average output-shaft speed, rad/s.
    float w;           // Half right-minus-left speed, rad/s (differential turn).
} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float pitch_v;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    uint8_t fair_flag;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    float real_wz;
    float rest_heat;           // 剩余枪口热量
    float motor_speed[HUBS_NUMBER]; // 电机速度
    float motor_current[HUBS_NUMBER]; // 电机电流
    uint8_t chassis_level;
    uint16_t chassis_power_limit;
    uint16_t chassis_power;
    uint16_t chassis_cap_current;
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red
} Chassis_Upload_Data_s;

typedef struct
{
   uint16_t life;
   uint8_t color;
   uint16_t allowance_17mm;
   uint8_t game_progress;

} Referee_Upload_Data_s;

typedef struct
{
    attitude_t gimbal_imu_data;
    float yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    shoot_mode_e shoot_mode;
    uint8_t fair_flag;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

typedef union float_uint16
{
    float f[1];
    uint16_t u[2];
    /* data */
}float_uint16_t;

#endif // !ROBOT_DEF_H
