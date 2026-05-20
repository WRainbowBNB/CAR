#ifndef __LINETRACKING_H__
#define __LINETRACKING_H__

#include <stdint.h>
#include "main.h"
#include "Motor.h"
#include "imu.h"
#include "encoder.h"
#include "yaw_filter.h"

#define Sub_SMTableS_LEN (sizeof(Sub_SMTableS) / sizeof(Sub_SMTableS[0]))

extern int8_t Motor1_Speed;
extern int8_t Motor2_Speed;

//子状态机状态处理函数指针类型
typedef void(*sub_handlestate)();

//子状态机状态枚举
typedef enum{
    TRACK_LINE,         //0 黑线
    TRACK_RUSH,         //1 起步冲桥
    TRACK_LINE_BREAK,   //2 断点
    TRACK_CROSS_STOP,   //3 十字停止
    TRACK_SELECT_TURN,  //4 选择直角弯
    TRACK_RIGHT_TURN,   //5 右直角弯
    TRACK_LEFT_TURN,    //6 左直角弯
    TRACK_FINISH_STOP,  //7 完成停止
    TRACK_STOP,         //8 偏离停止
    TRACK_NONE,         //9 无状态
    EXIT_TRACKING,      //10 退出循迹
    TRACK_COUNT         //11 子状态总数
}TraceState;

//子状态机事件枚举
typedef enum{
    SUBEVENT_LINE,          //0 黑线
    SUBEVENT_RUSH,          //1 起步冲桥
    SUBEVENT_LINE_BREAK,    //1 断点
    SUBEVENT_CROSS_STOP,    //2 十字停止
    SUBEVENT_SELECT_TURN,   //3 选择直角弯
    SUBEVENT_RIGHT_TURN,    //4 右直角弯
    SUBEVENT_LEFT_TURN,     //5 左直角弯
    SUBEVENT_FINISH_STOP,   //6 完成停止
    SUBEVENT_STOP,          //7 偏离
    SUBEVENT_EXIT,          //8 退出循迹
    SUBEVENT_NONE,          //9 无事件
    SUBEVENT_COUNT          //10 事件总数
}TraceEvent;

struct Sub_SMTable{
    TraceState current_substate;
    TraceEvent subevent;
    TraceState new_substate;
    sub_handlestate Sub_StateHandler;
};

//void LineTracking(uint8_t X1, uint8_t X2, uint8_t X3, uint8_t X4, uint8_t X5);
void Sub_State_Transmission(void);
void Sub_State_Run(void);
static TraceEvent Map_Sensor_to_event(void);
void Trace_Init(void);
int Trace_Run(void);
void handle_right_turn(void);
void handle_rush(void);
void handle_cross_stop(void);
void TraceSensor_Read(void);

static TraceEvent White_Return_Event(void);
static TraceEvent Black_Return_Event(void);
static TraceEvent Map_Sensor_to_event(void);

extern uint8_t yaw_angle;
extern uint8_t yaw_flag;
extern uint8_t sensor_status;
extern uint8_t line_cnt;
extern int trackflag;
#endif

