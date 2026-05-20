#ifndef __SM_H__
#define __SM_H__

#include "stdio.h"
#include "PS2.h"
#include "Controller.h"
#include "Motor.h"
#include "LineTracking.h"
#include "robotic_arm.h"

#define SMTables_LEN (sizeof(SMTables) / sizeof(SMTables[0]))

//定义函数指针
typedef void(*handlestate)();

//状态枚举
typedef enum{
    STATE_IDLE,     //0 空闲
    STATE_REMOTE,   //1 遥控
    STATE_TRACE,    //2 循迹
    STATE_COUNT     //3 状态总数
}FSMState;

//事件枚举
typedef enum{
    EVENT_NONE ,		 	 //0 没有事件
	EVENT_SELECT,            //1 按SELECT
    EVENT_TRIANGLE,          //2 按三角
    EVENT_CROSS, 			 //3 按叉
	EVENT_COUNT              //4 事件总数
}FSMEvent;

struct SMTable
{
    FSMState currentstate;
    FSMEvent event;
    FSMState newstate;
    handlestate StateHandler;
};

void State_Transmission(void);
void handle_idle(void);
void handle_remote(void);
void handle_trace(void);
void State_Run(void);
FSMEvent Map_Button_To_Event(uint16_t button);

extern FSMEvent events;
extern FSMState currentstates;
extern uint8_t test_status;
extern uint8_t claw_status;

#endif 


// //将上述函数的地址存入该指针数组
// handlestate StateHandler[STATE_COUNT] ={
//     handle_idle,        //0
//     handle_remote,      //1
//     handle_trace        //2
// };
// FSMState SMTable[STATE_COUNT][EVENT_COUNT] = {
//     //行 ：状态 ，列 ：事件        
//     {STATE_REMOTE ,  STATE_TRACE,   STATE_REMOTE},
//     {STATE_TRACE,    STATE_REMOTE,  STATE_TRACE},
//     {STATE_REMOTE,   STATE_TRACE,   STATE_REMOTE}
// };
// void State_Transmission(currentstate, event){
//     if(currentstate != newstate){
//         sprintf(message, "状态转换：%d -> %d", currentstate, newstate);
//         currentstate = newstate;
//     }
//     StateHandler[currentstate]();
// }
