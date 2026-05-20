#include "SM.h"

uint8_t claw_status = 0;
uint8_t test_status = 0;
//******主状态机*******heihei

FSMState currentstates = STATE_REMOTE;
FSMEvent events = EVENT_NONE;
FSMState laststates = STATE_IDLE;
// 将 PS2 按键转换为状态机事件
FSMEvent Map_Button_To_Event(uint16_t key) {
    if (key & PSB_TRIANGLE) return EVENT_TRIANGLE;
    if (key & PSB_CROSS)    return EVENT_CROSS;
    if (key & PSB_SELECT)   return EVENT_SELECT;
    //机械臂控制
    if (key & PSB_L2) claw_status = 1;  //爪子闭合
    else if (key & PSB_R2) claw_status = 2;  //爪子张开
    //调试（搞这个是为了解决lora模块高频发接收不到的问题）
    if (key & PSB_L1) test_status = 1;//开启发送串口数据，看波形
    else if (key & PSB_R1) test_status = 2;//关闭发送串口数据，调参用
    //菜单控制在OLED_UI_Driver中实现

    return EVENT_NONE; // 没有按这仨键
}


void handle_idle(){
	Motor_Stop();
}

void handle_remote(){
	Controller2(Data);
    arm_controller();
    //爪子
    if(claw_status == 2){
        claw_open();
    }
    else if(claw_status == 1){
        claw_close();
    }
}

void handle_trace(){
    if(Trace_Run()){
        currentstates = STATE_REMOTE;
        Motor_Stop();
    }
}


//状态转换表
static const struct SMTable SMTables[] = {
    //当前状态      //事件              //下一个状态        //回调
    {STATE_IDLE,    EVENT_TRIANGLE,    STATE_REMOTE,      handle_remote},     //0
    {STATE_REMOTE,  EVENT_CROSS,       STATE_TRACE,       handle_trace},      //1  
    {STATE_TRACE,   EVENT_TRIANGLE,    STATE_REMOTE,      handle_remote},     //2
    {STATE_IDLE,    EVENT_CROSS,       STATE_TRACE,       handle_trace},      //3
    {STATE_REMOTE,  EVENT_SELECT,      STATE_IDLE,        handle_idle},       //4
    {STATE_TRACE,   EVENT_SELECT,      STATE_IDLE,        handle_idle},       //5
    {STATE_IDLE,    EVENT_SELECT,      STATE_IDLE,        handle_idle},       //6
    {STATE_REMOTE,  EVENT_TRIANGLE,    STATE_REMOTE,      handle_remote},     //7
    {STATE_TRACE,   EVENT_CROSS,       STATE_TRACE,       handle_trace},      //8
};

//状态转换函数
void State_Transmission(){
    for (int i = 0; i < SMTables_LEN; i++){
        if(currentstates == SMTables[i].currentstate && events == SMTables[i].event){
            currentstates = SMTables[i].newstate;
            events = EVENT_NONE;
            break;
        }
    };
}

void State_Run(){
    if(currentstates != laststates){
        if(currentstates == STATE_TRACE){
            Trace_Init();
        }
        laststates = currentstates;
    }
        switch(currentstates){
          case STATE_IDLE:
            handle_idle();
            break;
          case STATE_REMOTE:
            handle_remote();
            break;
          case STATE_TRACE:
            handle_trace();
            break;
          default:
            handle_idle();
        }
}




