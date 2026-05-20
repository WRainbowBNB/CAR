# 基于 STM32 的麦克纳姆轮全向移动小车 🚗

## 0. 现阶段需求与项目概述 (Project Overview)
此项目为能成功参赛E唯杯而设计，需满足以下需求：

1. **全向移动：** 能够在任意方向上自由移动，包括平移、旋转和斜向运动。
2. **自动循迹：** 能够根据五路循迹传感器反馈，自动调整运动方向，实现自动导航。
3. **遥控模式：** 能够通过 PS2 无线手柄遥控小车运动，包括全向移动、速度调整和模式切换，同时能够操纵机械臂。
4. **姿态保持：** 能够基于 JY62 陀螺仪传感器，保持小车的稳定姿态，避免侧滑和大幅度漂移。
5. **故障保护：** 能够在检测到异常情况（如电机故障、小车冲出循迹赛道、小车突然抽风（逻辑待改进）等）时，及时触发紧急刹车，保护小车和用户。

## 1. 底层电控与驱动架构 (Low-Level Control & Driver Architecture)

* **1.1 引脚分配 (System Architecture):** 

[引脚分配如图](./images/引脚分配.png)
* **1.2 PS2 无线手柄通信系统 (PS2 Communication System):**
  * 1.2.1 按键及其对应功能

[按键对应功能如表](./images/PS2.png)
<style>
table {
    width: 100% !important;
}
</style>
| 按键 | 功能 |
| :--- | :--- | 
| 三角 | 遥控模式 | 
|  叉  | 循迹模式 | 
| SELECT | 空闲模式 | 
| 左摇杆 | 全向移动 |
| 右摇杆| 自旋 |
| L2 | 爪子闭合 |
| R2 | 爪子张开 |
| 十字前进 | 机械臂上旋 |
| 十字后退 | 机械臂下旋 |

  * 1.2.2 伪双端通信协议实现（纯软件模拟 SPI 时序的底层实现机制）
  *注：PS2 接收器 DAT 引脚必须配置为上拉输入，CMD、CS、CLK 引脚为推挽输出。本项目中为方便使用，启动即为遥控模式*

  详情见PS2.c和PS2.h，好使，能用。

* **1.3 麦轮儿运动学解算 (Kinematics Solution):** 麦轮采用O形安装，目前假定为重心与几何中心重合，如果不重合，则需要修改相应参数，即修改重心偏移几何中心的X轴和Y轴分量。同时为尽可能降低上桥侧滑的风险，本项目引入了串级pid，详情见1.4。 X、Y、yaw 三轴期望速度转化为四个电机的目标速度的原理都在代码里，详情见Motor.c和Motor.h。
```
    //Motor.h中相关参数的宏定义
    #define DX 0.0  //重心X偏移量
    #define DY 0.0  //重心Y偏移量
    #define A 7.5 //单位：cm 车轴到几何中心X轴距离
    #define B 6   //单位：cm 车轴到几何中心Y轴距离
    #define k_fl ((A + DX) + (B - DY)) //左前
    #define k_fr ((A - DX) + (B - DY)) //右前
    #define k_rl ((A + DX) + (B + DY)) //左后
    #define k_rr ((A - DX) + (B + DY)) //右后

    // 麦轮逆运动学解算
    float fl_speed = (target_vy + target_vx - target_pid_vz * k_fl);  //左前轮
	float fr_speed = (target_vy - target_vx + target_pid_vz * k_fr);  //右前轮
	float rl_speed = (target_vy - target_vx - target_pid_vz * k_rl);  //左后轮
	float rr_speed = (target_vy + target_vx + target_pid_vz * k_rr);  //右后轮
```

* **1.4 姿态与速度闭环控制 (Closed-loop Control System) 🌟:**
1ms采一次样，10ms一次pid，手写基于定时器输入捕获的T法和M法结合的测速代码，纯手工调参。上位机和主控通过蓝牙串口通信，采用VOFA+实现实时监控实际速度和目标速度并修改PID参数，加了限幅滤波和中值滤波，以此来拦截不必要的尖刺，避免电机突然ceng扭一下。
  * 1.4.1 内环：基于JGB37-520电机自带的霍尔编码器的电机速度 PID 控制。
  * 1.4.2 外环：基于 JY62 Yaw 轴（偏航角）的姿态 PID 控制。

调的时候我的想法是先带负载调内环，把四个电机的参数都调好，然后再调外环。

实际效果有待测试。

* **1.5 OLED 人机交互显示 (OLED UI Display):** 实时状态监控与参数可视化。嗯，如果还有时间想做个菜单儿。

[OLED样图](./images/OLED.jpg)
## 2. 上层应用与状态调度 (Upper-Level Application & State Scheduling)
* **2.1 系统状态机调度 (Finite State Machine, FSM):** 采用内外表驱动状态机实现状态切换，大状态机包括“遥控模式 / 循迹模式 / 空闲模式”三个状态，小状态机则为循迹模式。
* **2.2 遥控模式 (Remote Control Mode):**
  * 2.2.1全向移动映射逻辑：将 PS2 摇杆模拟量映射到期望的 X、Y、yaw 三轴速度。
```
  //叫Controller2是因为我之前写了一个差速普通轮儿的Controller1

  void Controller2(uint8_t *Data)
{
    //为实现速度线性变化，将PS2摇杆模拟量映射到期望的 X、Y、Z（yaw） 三轴速度
    int16_t X = Data[7] - 128;
    int16_t Y = 128 - Data[8];
    int16_t Z = Data[5] - 128;

    //死区处理
    if(abs(X) < 20) X = 0;
    if(abs(Y) < 20) Y = 0;
    if(abs(Z) < 20) Z = 0;

    //车体期望速度
    float vx = (float)X / 128.0f * MAX_SPEED;    //左右
    float vy = (float)Y / 128.0f * MAX_SPEED;    //前后
    float vz = (float)Z / 128.0f * MAX_ANGULAR;  //旋转

    //传入解算函数，计算电机的目标速度
    Mecanum_SetSpeed(vx, vy, vz);`

}
```
  * 2.2.2 机械臂控制：搭配机械臂实体食用，实现机械臂的旋动和爪子开合。
  * 2.2.2.1 机械臂旋动：根据 PS2 上下十字键确定机械臂上下旋动的角度。
    ```
    arm_up();   //上旋
    arm_down(); //下旋
    ```
  * 2.2.2.2爪子开合：用 PS2 的L2和L1控制爪子的开合。
    ```
    claw_open();  //爪开
    claw_close(); //爪合
    ```
* **2.3 循迹模式 (Trace Mode):** 传感器数据处理与循迹算法逻辑。

*普通黑线循迹（TRACK_LINE）：* 利用黑线传感器检测黑线，根据黑线位置调整小车运动方向。
```
void handle_line(){
	int16_t vz = 10 * X1 + 5 * X2  - 5 * X4 - 10 * X5;
	Mecanum_SetSpeed(0, 20, vz);
}
```

*注意，因为没实际测过，所以参数啥的有待调整，而且其移动可能也不太平滑，此处有待改进*

*起步冲桥（TRACK_RUSH）：* 当小车检测到起点水平黑线时，立即开始锁头冲桥，此处为避免与断点区域冲突，故起步状态上锁，直至到达指定时间后解锁（需要实测）。
```
//处理拱桥（直接冲）
void handle_rush(){
	float yaw_speed = yaw_angle_PD(cur_imu.yaw);
	//以下是冲桥逻辑
	static uint8_t waiting_flag = 0;
	static uint32_t stop_start_time = 0;
	if(waiting_flag == 0){
		waiting_flag = 1;
		stop_start_time = HAL_GetTick();//开始冲桥计时
		Mecanum_SetSpeed(0, 50, yaw_speed);
	}else{
		Mecanum_SetSpeed(0, 50, yaw_speed);
		if(HAL_GetTick() - stop_start_time >= 3000){		//具体冲桥时间记得实测
		current_substate = TRACK_LINE;
		waiting_flag = 0;
		rush_flag = 1;
		yaw_flag = 0;
		}
	}
}
```

*断点（TRACK_LINE_BREAK）：* 当小车检测到全白区域时，开始盲冲，速度低于冲桥速度，避免直接冲出赛道，直至再次遇到黑线。
```
//处理断点
void handle_line_break(){
	Mecanum_SetSpeed(0, 30, 0);
}
```
*十字停止（TRACK_CROSS_STOP）：* 当小车检测到十字区域时，停止运动3s。
```
//处理十字停车点（3s）
void handle_cross_stop(){
	static uint8_t waiting_flag = 0;
	static uint32_t stop_start_time = 0;
	if(waiting_flag == 0){
		waiting_flag = 1;
		stop_start_time = HAL_GetTick();
	}else{
		Mecanum_SetSpeed(0, 0, 0);
		if(HAL_GetTick() - stop_start_time >= 3000){
		Mecanum_SetSpeed(0, 40, 0);	//避免一直停在十字停车点
		current_substate = TRACK_LINE;
		waiting_flag = 0;
		}
	}
}
```
*选择直角弯（TRACK_SELECT_TURN）：* 当小车检测到直角区域时，根据抽签结果，选择左转或右转。
```
//要是没问题想试试用JY62的盲转
//处理选择直角弯（根据抽签结果）
void handle_select_turn(){
	static uint8_t waiting_flag = 0;
	static uint32_t stop_start_time = 0;
	if(waiting_flag == 0){
		waiting_flag = 1;
		stop_start_time = HAL_GetTick();
	}else{
		Mecanum_SetSpeed(0, 0, -30);	//这个看到时候抽签决定正负号
		if(HAL_GetTick() - stop_start_time >= 500){
		current_substate = TRACK_LINE;
		waiting_flag = 0;
		}
	}
}
```
*右直角弯（TRACK_RIGHT_TURN）：* 盲右转，直到到达指定时间（需要实测）。
```
//要是没问题想试试用JY62的盲转
//处理中途直角弯--右转
void handle_right_turn(){
	static uint8_t waiting_flag = 0;
	static uint32_t stop_start_time = 0;
	if(waiting_flag == 0){
		waiting_flag = 1;
		stop_start_time = HAL_GetTick();
		Mecanum_SetSpeed(0, 0 , -30);	//这个盲转要看具体实测时间
	}else{
		if(HAL_GetTick() - stop_start_time >= 500){
		current_substate = TRACK_LINE;
		waiting_flag = 0;
		}
	}
}
```
*左直角弯（TRACK_LEFT_TURN）：* 盲左转，代码逻辑同上。

*完成停止（TRACK_FINISH_STOP）：* 当小车检测到终点水平黑线时，停车。
```
//循迹完毕
void handle_finish_stop(){
	Mecanum_SetSpeed(0, 0 ,0);
}
```
*偏离停止（TRACK_STOP）：* 超过一定时间未检测到黑线，停车，超时逻辑在事件返回函数里面，具体函数有些长了，就不在这里展开。

*退出循迹（EXIT_TRACKING）：* 当小车完成循迹或在任意时刻按下PS2的其他模式按键，退出循迹。

详情见LineTracking.c和.h文件

*注意循迹逻辑整体有待测试，尤其是在直角弯的处理上，需要根据实际情况调整时间参数。*


## 3. 可改进之处 (Improvement)
1. 调pid的时候可以将蓝牙换成lora模块，使得通信更加稳定。
2. 循迹整体都有待测试，在普通黑线的处理上，应该可以更加圆滑一点，直角弯和冲桥逻辑的相关参数都有待实测调整，其状态下的代码逻辑都有待改进。
3. 如果循迹选择直角弯的抽签结果定了的话，其回调完全可以用普通直角弯的。
4. 初始速度置零逻辑有待改进，避免PS2和接收器没配对儿的情况下小车发疯，目前代码貌似还是会疯。
5. 关于PS2断联处理逻辑上也有待改进，其改进逻辑同3.。
6. 外环yaw轴如果能跑通的话，可以考虑加上x和y轴，来实现更精确更稳的控制。
7. 如果还有时间可以给oled加菜单儿功能，从而显示更多的实时数据，比如三轴速度、欧拉角等。
8. 电机堵转检测逻辑有待考察。

## 4. 学长们的建议
1. 机械臂可以改成模拟量控制，而不是用十字键
2. 闭yaw可以结合编码器和JY62，搞一个置信区间，因为JY62容易飘
3. 字符串可以写成数据帧的形式？可以研究一下
3. IMU模块的数据接收可以写成队列，会更好一点？而且如果丢包了或者数据有残缺这个应该予以解决，逻辑待加