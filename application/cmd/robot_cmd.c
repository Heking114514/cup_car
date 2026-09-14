
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

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
// @todo 8191转换成360的精度太低,会损失精度
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360
#define NAV_LINEAR_REF_PER_MPS \
    (360.0f * REDUCTION_RATIO_WHEEL * 1000.0f / (2.0f * PI * RADIUS_WHEEL))
#define NAV_ANGULAR_REF_PER_RADPS \
    (NAV_LINEAR_REF_PER_MPS * TRACK_WIDTH / 2000.0f)


#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Subscriber_t *Referee_data_sub; // 底盘反馈信息订阅者

#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据


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
    chassis_cmd_send.v = vision_recv_data->v * NAV_LINEAR_REF_PER_MPS;
    chassis_cmd_send.w = vision_recv_data->w * NAV_ANGULAR_REF_PER_RADPS;

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
