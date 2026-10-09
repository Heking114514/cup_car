
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buffer.h"
#include "referee_task.h"
#if ROBOT_USE_REMOTE_TEST_MODE
#include "remote_control.h"
#endif

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include <math.h>

// 私有宏,自动将编码器转换成角度值
// @todo 8191转换成360的精度太低,会损失精度
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360
#define NAV_LEFT_REF_PER_MPS \
    (360.0f * LEFT_REDUCTION_RATIO_WHEEL * 1000.0f / (2.0f * PI * RADIUS_WHEEL))
#define NAV_RIGHT_REF_PER_MPS \
    (360.0f * RIGHT_REDUCTION_RATIO_WHEEL * 1000.0f / (2.0f * PI * RADIUS_WHEEL))
#define REMOTE_TEST_ROCKER_MAX 500.0f
#define REMOTE_TEST_ROCKER_DEADBAND 90.0f
#define REMOTE_TEST_LINEAR_MIN_REF 3300.0f
#define REMOTE_TEST_LINEAR_MAX_REF 6800.0f
#define REMOTE_TEST_ANGULAR_MIN_REF 2400.0f
#define REMOTE_TEST_ANGULAR_MAX_REF 3800.0f
#define REMOTE_TEST_MOVING_TURN_RATIO 0.75f
#define REMOTE_TEST_LINEAR_SLEW_REF_PER_S 5500.0f
#define REMOTE_TEST_ANGULAR_SLEW_REF_PER_S 4200.0f
#define REMOTE_TEST_CMD_PERIOD_S 0.01f


#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Subscriber_t *Referee_data_sub; // 底盘反馈信息订阅者

#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据
#if ROBOT_USE_REMOTE_TEST_MODE
static RC_ctrl_t *remote_ctrl;
static float remote_v_ref;
static float remote_w_ref;
static uint8_t remote_test_armed;
#endif


static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
static buf_t *buffer_yaw, *buffer_pitch, *buffer_delay_yaw;
static Robot_Status_e robot_state; // 机器人整体工作状态
#if CHASSIS_USE_INS
static INS_t INS_CMD;
#endif

static uint8_t flag = 1;
static float aligned_total_yaw, aligned_total_pitch, delayed_total_yaw, fitter_vision_recv_data_yaw;
static float send_first, send_first_pitch, send_second;


#if ROBOT_USE_REMOTE_TEST_MODE
static float RemoteClamp(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static float RemoteNormalizeChannel(int16_t raw)
{
    float value = (float)raw;
    float abs_value = fabsf(value);
    float normalized;

    if (abs_value <= REMOTE_TEST_ROCKER_DEADBAND)
        return 0.0f;

    if (abs_value > REMOTE_TEST_ROCKER_MAX)
        abs_value = REMOTE_TEST_ROCKER_MAX;

    normalized = (abs_value - REMOTE_TEST_ROCKER_DEADBAND) /
                 (REMOTE_TEST_ROCKER_MAX - REMOTE_TEST_ROCKER_DEADBAND);
    if (value < 0.0f)
        normalized = -normalized;

    return RemoteClamp(normalized, 1.0f);
}

static float RemoteShape(float normalized)
{
    return 0.65f * normalized + 0.35f * normalized * normalized * normalized;
}

static float RemoteApplyMinRef(float shaped, float min_ref, float max_ref)
{
    float magnitude = fabsf(shaped);
    float ref;

    if (magnitude <= 0.0f)
        return 0.0f;

    ref = min_ref + magnitude * (max_ref - min_ref);
    if (shaped < 0.0f)
        ref = -ref;
    return ref;
}

static uint8_t RemoteAxesNeutral(const RC_ctrl_t *ctrl)
{
    return abs(ctrl[TEMP].rc.rocker_r1) <= REMOTE_TEST_ROCKER_DEADBAND &&
           abs(ctrl[TEMP].rc.rocker_l_) <= REMOTE_TEST_ROCKER_DEADBAND;
}

static float RemoteSlew(float current, float target, float rate_ref_per_s)
{
    float step = rate_ref_per_s * REMOTE_TEST_CMD_PERIOD_S;
    float delta = target - current;

    if (delta > step)
        return current + step;
    if (delta < -step)
        return current - step;
    return target;
}
#endif


void syncWithVisionSystem()
{
    static uint8_t flag___ = 0;
    flag___++;
    flag___ %= 2;
    // aligned_total_yaw = BUFUpdata(buffer_yaw, gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, 1);
    // aligned_total_pitch = BUFUpdata(buffer_pitch, gimbal_fetch_data.gimbal_imu_data.Roll, 1);
}

void RobotCMDInit()
{
    vision_recv_data = VisionInit(&huart10, syncWithVisionSystem); // 视觉通信串口
    VisionSetNavigationMode(1U);
#if ROBOT_USE_REMOTE_TEST_MODE
    remote_ctrl = RemoteControlInit(&huart5);
#endif
    // Referee_ToVision_data=RefereeInit(&huart1);  不需要在chassic的UI初始化中嵌套的有
    buffer_yaw = BUFRegister();
    buffer_pitch = BUFRegister();
    buffer_delay_yaw = BUFRegister();
#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

#endif // ONE_BOARD

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}


/**
 * @brief 使用上位机速度指令设置导航控制量
 *
 */
static void NavigationControlSet()
{
#if ROBOT_USE_REMOTE_TEST_MODE
    (void)vision_recv_data;
    if (remote_ctrl != NULL && RemoteControlIsOnline())
    {
        if (!remote_test_armed)
        {
            if (RemoteAxesNeutral(remote_ctrl))
                remote_test_armed = 1U;
            remote_v_ref = RemoteSlew(remote_v_ref, 0.0f,
                                      REMOTE_TEST_LINEAR_SLEW_REF_PER_S * 4.0f);
            remote_w_ref = RemoteSlew(remote_w_ref, 0.0f,
                                      REMOTE_TEST_ANGULAR_SLEW_REF_PER_S * 4.0f);
        }
        else
        {
            int16_t forward_raw = remote_ctrl[TEMP].rc.rocker_r1;
            int16_t turn_raw = remote_ctrl[TEMP].rc.rocker_l_;
            float forward = RemoteShape(RemoteNormalizeChannel(forward_raw));
            float turn = RemoteShape(RemoteNormalizeChannel(turn_raw));
            float target_v = RemoteApplyMinRef(forward,
                                               REMOTE_TEST_LINEAR_MIN_REF,
                                               REMOTE_TEST_LINEAR_MAX_REF);
            float target_w = -RemoteApplyMinRef(turn,
                                                REMOTE_TEST_ANGULAR_MIN_REF,
                                                REMOTE_TEST_ANGULAR_MAX_REF);

            if (target_v != 0.0f)
                target_w = RemoteClamp(target_w,
                                       fabsf(target_v) *
                                       REMOTE_TEST_MOVING_TURN_RATIO);

            remote_v_ref = RemoteSlew(remote_v_ref, target_v,
                                      REMOTE_TEST_LINEAR_SLEW_REF_PER_S);
            remote_w_ref = RemoteSlew(remote_w_ref, target_w,
                                      REMOTE_TEST_ANGULAR_SLEW_REF_PER_S);
        }
    }
    else
    {
        remote_test_armed = 0U;
        remote_v_ref = RemoteSlew(remote_v_ref, 0.0f,
                                  REMOTE_TEST_LINEAR_SLEW_REF_PER_S * 4.0f);
        remote_w_ref = RemoteSlew(remote_w_ref, 0.0f,
                                  REMOTE_TEST_ANGULAR_SLEW_REF_PER_S * 4.0f);
    }

    chassis_cmd_send.v = remote_v_ref;
    chassis_cmd_send.w = remote_w_ref;
#else
    {
        float half_track_m = TRACK_WIDTH / 2000.0f;
        float left_ref = (-vision_recv_data->v +
                          vision_recv_data->w * half_track_m) *
                         NAV_LEFT_REF_PER_MPS;
        float right_ref = (-vision_recv_data->v -
                           vision_recv_data->w * half_track_m) *
                          NAV_RIGHT_REF_PER_MPS;

        chassis_cmd_send.v = (left_ref + right_ref) * 0.5f;
        chassis_cmd_send.w = (right_ref - left_ref) * 0.5f;
    }
#endif

#if CHASSIS_USE_INS
    INS_GetAttitude(&INS_CMD.Yaw, &INS_CMD.Pitch, &INS_CMD.Roll);
    VisionSetAltitude(INS_CMD.Yaw,
                      INS_CMD.Pitch,
                      INS_CMD.Roll);
#endif
    // 云台参数,确定云台控制数据

}


/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
    VisionProcess();

    NavigationControlSet();

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
    VisionSend();
    
}
