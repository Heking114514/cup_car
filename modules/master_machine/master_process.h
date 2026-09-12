#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "bsp_usb.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 36u
#define VISION_SEND_SIZE 36u
#define Nav_RECV_SIZE 22u

#pragma pack(1)

typedef enum
{
    NO_FIRE = 0,
    AUTO_FIRE = 1,
    AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
    NO_TARGET_NUM = 0,
    HERO1 = 1,
    ENGINEER2 = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    OUTPOST = 6,
    SENTRY = 7,
    BASE = 8
} Target_Type_e;

// ✅ 你要求的枚举放这里
typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef enum
{
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef struct
{
    Fire_Mode_e fire_mode;
    Target_State_e target_state;
    Target_Type_e target_type;
    float v;
    float w;
} Vision_Recv_s;

typedef struct
{
    Fire_Mode_e fire_mode;
    Target_State_e target_state;
    Target_Type_e target_type;
    float yaw;
    float pitch;
    uint8_t shoot;
    float vx;
    float vy;
    float wz;
    float spin;
} Nav_Recv_s;

typedef struct
{
    Enemy_Color_e enemy_color;
    Work_Mode_e work_mode;
    Bullet_Speed_e bullet_speed;
    float yaw;
    float pitch;
    float roll;
} Vision_Send_s;

typedef struct
{
    float battery;
    float life;
    float color;
    float bullet;
    float game_mode;
} Vision_Refree_Send_s;
#pragma pack()

// 初始化接口，返回接收数据结构体指针
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle, void (*application_callback)(void));
Nav_Recv_s *NavInit(UART_HandleTypeDef *_handle);
void VisionSend(void);
void VisionSetAltitude(float yaw, float pitch, float roll);
void VisionRefree_SetAltitude(float battery, float life, float color, float bullet, float game_mode);
void Vision_Refree_Send(void);
void Vision_Send_All(void);

#endif // MASTER_PROCESS_H