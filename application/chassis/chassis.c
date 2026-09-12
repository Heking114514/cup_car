#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "ins_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "buzzer.h"
#include "power_meter.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD

static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Subscriber_t *chassis_power_sub;         // 用于订阅底盘的反馈数据
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Power_Data_s chassis_power_recv;
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据


static DJIMotorInstance *motor_l, *motor_r; // left right forward back

static PIDInstance chassis_follow_to_yaw_pid;
static BuzzzerInstance *buzzerc;
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息
static attitude_t *chassis_IMU_data;
float v_debug=4000.0;

#define CHASSIS_MOTOR_LEFT_ID 1u
#define CHASSIS_MOTOR_RIGHT_ID 2u

void ChassisInit()
{
    chassis_IMU_data = INS_Init();
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = CHASSIS_MOTOR_LEFT_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 1000, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 10000,
                .MaxOut_ = -10000
            },
            .speed_PID = {
                .Kp = 0.6f,
                .Ki = 0.02f,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 2500,
                .MaxOut = 7000,
                .MaxOut_ = -7000
            },
            .current_PID = {
                .Kp = 0.5f,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_IMPROVE_NONE,
                .IntegralLimit = 0,
                .MaxOut = 10000,
                .MaxOut_ = -10000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    // 实际电调 ID：左轮 1，右轮 2。两台电机共用 0x200，分别使用第 1、2 个电流槽位。
    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_LEFT_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_l = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_RIGHT_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_r = DJIMotorInit(&chassis_motor_config);

    Buzzer_config_s buzzer = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .loudness = 0,
        .octave = OCTAVE_2,
    };
    buzzerc = BuzzerRegister(&buzzer);
#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

#endif // ONE_BOARD
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif

    if(fabsf(chassis_cmd_recv.v)<50&&fabsf(chassis_cmd_recv.w)<50)
    {
        DJIMotorStop(motor_l);
        DJIMotorStop(motor_r);
    }
    else
    {
        DJIMotorEnable(motor_l);
        DJIMotorEnable(motor_r);
    }

    DJIMotorSetRef(motor_l,chassis_cmd_recv.v-chassis_cmd_recv.w);
    DJIMotorSetRef(motor_r,chassis_cmd_recv.v+chassis_cmd_recv.w);
}
