#include "encoder.h"

float en_x_speed = 0.0f;
float en_y_speed = 0.0f;
float X_world = 0.0f;
float Y_world = 0.0f;

//定义空结构体
Motor_Handle motor1 = {0};
Motor_Handle motor2 = {0};
Motor_Handle motor3 = {0};
Motor_Handle motor4 = {0};
PIDController wz = {0};
PIDController Y = {0};
PIDController X = {0};
Position_PID position_pid = {0};
Position position = {0};
/**
 * @brief 限幅滤波函数
 * @param new_speed 本次实测的原始速度值（带尖刺噪声）
 * @return 滤波后的有效速度值
 */
static float LimitFilter(PIDController *pid, float new_speed)
{
    float diff = fabs(new_speed - pid->last_valid_speed);
	if(diff >= STOP_THRESHOLD){
		pid->last_valid_speed = new_speed;
        pid->filter_noise_cnt = 0; // 重置异常计数
		return new_speed; //允许急停
	}
    else if (diff > MAX_SPEED_DIFF) {
			pid->filter_noise_cnt++;
            if(pid->filter_noise_cnt >= 3){
                pid->last_valid_speed = new_speed;  // 可能是急速的速度变化，更新有效速度
                pid->filter_noise_cnt = 0; // 重置异常计数
		        return new_speed;
            }
            return pid->last_valid_speed;  // 未到阈值，判定为异常值，返回上一次有效速度
		}
    //其他情况，认为是正常值，更新有效速度
    pid->last_valid_speed = new_speed;
    pid->filter_noise_cnt = 0; // 重置异常计数
    return new_speed;
        
}


/**
 * @brief 中值滤波函数
 * @param new_speed 本次实测的原始速度值（带尖刺噪声）
 * @return 滤波后的有效速度值
 */
static float MedianFilter(PIDController *pid, float new_speed)
{
	//缓冲区
	pid->speed_buffer[pid->speed_buffer_index] = new_speed;
	pid->speed_buffer_index = (pid->speed_buffer_index + 1) % MEDIAN_WINDOW_SIZE;
	// 对缓冲区中的值进行排序(选择排序)
    float temp[MEDIAN_WINDOW_SIZE];
    for(int i = 0; i < MEDIAN_WINDOW_SIZE; i++){
        temp[i] = pid->speed_buffer[i];
    }
	for(int i = 0; i < MEDIAN_WINDOW_SIZE - 1; i++){
		int min_idx = i;//记录最小索引
        for(int j = i + 1; j < MEDIAN_WINDOW_SIZE; j++){
            if(temp[j] < temp[min_idx]){
                min_idx = j;    //更新最小索引
            }
        }
        //交换
        float t = temp[i];
        temp[i] = temp[min_idx];
        temp[min_idx] = t;
    }
    // 取中值
    return temp[MEDIAN_WINDOW_SIZE / 2];
}


/**
 * @brief 计算基础PWM
 * @param target_speed 目标速度
 * @return float 基础PWM值
 */

float Get_Base_PWM(float target_speed){
    float base_pwm = 0.0f;
    if(fabs(target_speed) < 0.1f) return 0.0f;
    //斜率a
    float a = (80.0f - MOTOR_DEADZONE) / MAX_SPEED;
    //计算基础PWM
    if(target_speed >= 0.0f){
        base_pwm = MOTOR_DEADZONE + a * target_speed;
    }else{
        base_pwm = -MOTOR_DEADZONE + a * target_speed;
    }
    return base_pwm;
}
//********公共接口函数*********//

//PID参数初始化
void PID_Init(PIDController *pid, float kp, float ki, float kd, float max_output, float max_integral)
{
    if(pid == NULL) return; //空指针判断
    memset(pid, 0, sizeof(PIDController));  //清空结构体内容
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->MAXOutput = max_output;
    pid->MAXIntegral = max_integral;
}

//电机硬件初始化
void Motor_Init(Motor_Handle *motor, TIM_HandleTypeDef *PWMtim, uint32_t pwm_channel, GPIO_TypeDef *dir_port1, uint16_t dir_pin1, GPIO_TypeDef *dir_port2, uint16_t dir_pin2, 
    TIM_HandleTypeDef *enc_tim, uint32_t enc_channel, GPIO_TypeDef *enc_port1, uint16_t enc_pin1, GPIO_TypeDef *enc_port2, uint16_t enc_pin2)
{
    motor->PWMtim = PWMtim;
    motor->pwm_channel = pwm_channel;
    motor->dir_port1 = dir_port1;
    motor->dir_pin1 = dir_pin1;
    motor->dir_port2 = dir_port2;
    motor->dir_pin2 = dir_pin2;
    motor->enc_tim = enc_tim;
    motor->enc_channel = enc_channel;
    motor->enc_port1 = enc_port1;
    motor->enc_pin1 = enc_pin1;
    motor->enc_port2 = enc_port2;
    motor->enc_pin2 = enc_pin2;
    //启动PWM
    HAL_TIM_PWM_Start(motor->PWMtim, motor->pwm_channel);
}

//通用PID计算
float PID_compute(PIDController *pid, float raw_speed){
    //1.滤波处理
    //float filtered_speed = LimitFilter(pid, raw_speed);
    //float filtered_speed = MedianFilter(pid, raw_speed);
    if(fabs(pid->target_speed) <= 0.1f) pid->integral_sum = 0.0f;
    pid->current_speed = raw_speed;

    //2.计算误差
    pid->error = pid->target_speed - pid->current_speed;
    
    //3.计算积分项
    pid->integral_sum += pid->error * PID_DT; //周期10ms
    //积分限幅
	if(pid->integral_sum > pid->MAXIntegral){
		pid->integral_sum = pid->MAXIntegral;
	}else if(pid->integral_sum < -pid->MAXIntegral){
		pid->integral_sum = -pid->MAXIntegral;
	}
    
    //4.计算微分项（带低通滤波）
    // 微分一阶低通滤波
    pid->speed_kd = (pid->error - pid->lasterror) / PID_DT;    //周期10ms
    pid->lasterror = pid->error;
	pid->filtered_speed_kd = 0.9f * pid->filtered_speed_kd + 0.1f * pid->speed_kd;
	//微分限幅
	if(pid->filtered_speed_kd > 5.0f){
		pid->filtered_speed_kd = 5.0f;
	}else if(pid->filtered_speed_kd < -5.0f){
		pid->filtered_speed_kd = -5.0f;
	}

    //5.计算PID输出
    if(pid->target_speed > 0.1f || pid->target_speed < -0.1f){
    pid->output = Get_Base_PWM(pid->target_speed) + pid->error * pid->kp + pid->integral_sum * pid->ki + pid->filtered_speed_kd * pid->kd;
    }else{
        pid->output = pid->error * pid->kp + pid->integral_sum * pid->ki + pid->filtered_speed_kd * pid->kd;
    }

    //6.输出限幅
	if(pid->output > pid->MAXOutput){
		pid->output = pid->MAXOutput;
	}else if(pid->output < -pid->MAXOutput){
		pid->output = -pid->MAXOutput;
	}

	return pid->output;
}

float Axis_PID_compute(PIDController *pid){
    //1.滤波处理
    //float filtered_speed = LimitFilter(pid, pid->current_speed);
    //filtered_speed = MedianFilter(pid, filtered_speed);
    //pid->current_speed = filtered_speed;

    //2.计算误差
    pid->error = pid->target_speed - pid->current_speed;
    
    //3.计算积分项
    pid->integral_sum += pid->error * PID_DT; //周期10ms
    //积分限幅
	if(pid->integral_sum > pid->MAXIntegral){
		pid->integral_sum = pid->MAXIntegral;
	}else if(pid->integral_sum < -pid->MAXIntegral){
		pid->integral_sum = -pid->MAXIntegral;
	}
    
    //4.计算微分项（带低通滤波）
    // 微分一阶低通滤波
    pid->speed_kd = (pid->error - pid->lasterror) / PID_DT;    //周期10ms
    pid->lasterror = pid->error;
	pid->filtered_speed_kd = 0.9f * pid->filtered_speed_kd + 0.1f * pid->speed_kd;
	//微分限幅
	if(pid->filtered_speed_kd > 5.0f){
		pid->filtered_speed_kd = 5.0f;
	}else if(pid->filtered_speed_kd < -5.0f){
		pid->filtered_speed_kd = -5.0f;
	}

    //5.计算PID输出
    if(pid->target_speed > 0.1f || pid->target_speed < -0.1f){
    pid->output = PID_T *(pid->target_speed - pid->current_speed) / RESPONSE_TIME + pid->error * pid->kp + pid->integral_sum * pid->ki + pid->filtered_speed_kd * pid->kd;
    }else{
        pid->output = 0.0f;
    }

    //6.输出限幅
	if(pid->output > pid->MAXOutput){
		pid->output = pid->MAXOutput;
	}else if(pid->output < -pid->MAXOutput){
		pid->output = -pid->MAXOutput;
	}

	return pid->output;
}

Position Position_PID_compute(Position_PID *pid){
    //1.滤波处理
    //float filtered_speed = LimitFilter(pid, pid->current_speed);
    //filtered_speed = MedianFilter(pid, filtered_speed);
    //pid->current_speed = filtered_speed;

    
    for(int i = 0; i <= 1; i++){
        //2.计算误差
        pid->error[i] = pid->target_pos[i] - pid->current_pos[i];
        
        //3.计算积分项
        pid->integral_sum[i] += pid->error[i] * PID_DT;
        
        //积分限幅
        if(pid->integral_sum[i] > pid->MAXIntegral[i]){
            pid->integral_sum[i] = pid->MAXIntegral[i];
        }else if(pid->integral_sum[i] < -pid->MAXIntegral[i]){
            pid->integral_sum[i] = -pid->MAXIntegral[i];
        }
        
        //4.计算微分项（带低通滤波）
        // 微分一阶低通滤波
        pid->speed_kd[i] = (pid->error[i] - pid->lasterror[i]) / PID_DT;    //周期10ms
        pid->lasterror[i] = pid->error[i];
        pid->filtered_speed_kd[i] = 0.9f * pid->filtered_speed_kd[i] + 0.1f * pid->speed_kd[i];
        if(pid->filtered_speed_kd[i] > 5.0f){
            pid->filtered_speed_kd[i] = 5.0f;
        }else if(pid->filtered_speed_kd[i] < -5.0f){
            pid->filtered_speed_kd[i] = -5.0f;
        }

        //5.计算PID输出
        pid->output[i] = pid->error[i] * pid->kp[i] + pid->integral_sum[i] * pid->ki[i] + pid->filtered_speed_kd[i] * pid->kd[i];
    }
    return pid->position;
}

//电机闭环控制更新（在定时器中调用）
void Motor_Update_Control(Motor_Handle *motor){
    int32_t pulse_cnt;
    float interval;

    //1.获取当前脉冲数和时间间隔(临界区保护)
    __disable_irq();
    pulse_cnt = motor->pulse_cnt;
    interval = motor->interval;
    motor->pulse_cnt = 0;
    motor->interval = 0.0f;
    motor->overflow_cnt = 0; //重置溢出计数
    __enable_irq();

    //2.计算速度（单位：cm/s）
    float raw_speed = 0;
    if(interval <= 0.001f || pulse_cnt == 0){
    // 如果没有脉冲或间隔太小，保持上一次的speed，避免突变
        raw_speed = motor->pid.last_valid_speed;
        if(pulse_cnt == 0) {
            motor->pid.stop_timeout_cnt++;
            if(motor->pid.stop_timeout_cnt >= 3) {
                raw_speed = 0.0f;   //如果异常周期超过3个，更新实际速度为0，嗯，防幽灵
                motor->pid.capture_flag = 0; //重置捕获标志，等待下一次捕获
                motor->pid.stop_timeout_cnt = 3; //锁死异常状态，防止溢出
            }
        }      
    }else{
        raw_speed = (pulse_cnt  / (0.01f * ENCODER_LINES * MOTOR_REDUCTION * 2.0f)) * WHEEL_CIRCUMFERENCE; //单位：cm/s
		motor->pid.stop_timeout_cnt = 0; //重置异常周期数
    }
    motor->pid.current_speed = raw_speed;

    //3.计算PID输出
    //float pid_output = 30.0f;
    float pid_output = PID_compute(&motor->pid, motor->pid.current_speed); 
    //4.设置PWM和方向
    if(pid_output >= 0){
        //正转
        HAL_GPIO_WritePin(motor->dir_port1, motor->dir_pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->dir_port2, motor->dir_pin2, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->PWMtim, motor->pwm_channel, (uint32_t)(pid_output));
    }else{
        //反转
        HAL_GPIO_WritePin(motor->dir_port1, motor->dir_pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->dir_port2, motor->dir_pin2, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->PWMtim, motor->pwm_channel, (uint32_t)(-pid_output));
    }
}

//在输入捕获回调中调用此回调更新函数
void Motor_encoder_callback(Motor_Handle *motor, float interval){
    //更新脉冲数和时间间隔
    motor->pulse_cnt ++;
    motor->interval += interval;
}


//interval的处理在中断回调里

void All_Motors_Init(void){
    Motor_Init(&motor1, 
            &htim1, 
            TIM_CHANNEL_1,
            AIN1_GPIO_Port, 
            AIN1_Pin, 
            AIN2_GPIO_Port, 
            AIN2_Pin,
            &htim5,
            TIM_CHANNEL_3,
            Encoder1_A_GPIO_Port, 
            Encoder1_A_Pin,
            Encoder1_B_GPIO_Port, 
            Encoder1_B_Pin
            );
    PID_Init(&motor1.pid, 0.0f, 0.0f, 0.0f, 90.0f, 300.0f); // PID参数初始化
    Motor_Init(&motor2, 
           &htim1, 
           TIM_CHANNEL_2,
           BIN1_GPIO_Port, 
           BIN1_Pin, 
           BIN2_GPIO_Port, 
           BIN2_Pin,
           &htim3,
           TIM_CHANNEL_1,
           Encoder2_A_GPIO_Port, 
           Encoder2_A_Pin,
           Encoder2_B_GPIO_Port, 
           Encoder2_B_Pin
        );
  PID_Init(&motor2.pid, 0.0f, 0.0f, 0.0f, 90.0f, 300.0f); // PID参数初始化
  Motor_Init(&motor3, 
           &htim1, 
           TIM_CHANNEL_3,
           CIN1_GPIO_Port, 
           CIN1_Pin, 
           CIN2_GPIO_Port, 
           CIN2_Pin,
           &htim4,
           TIM_CHANNEL_1,
           Encoder3_A_GPIO_Port,
           Encoder3_A_Pin,
           Encoder3_B_GPIO_Port, 
           Encoder3_B_Pin
        );
  PID_Init(&motor3.pid, 0.0f, 0.0f, 0.0f, 90.0f, 300.0f); // PID参数初始化
  Motor_Init(&motor4, 
           &htim1, 
           TIM_CHANNEL_4,
           DIN1_GPIO_Port, 
           DIN1_Pin, 
           DIN2_GPIO_Port, 
           DIN2_Pin,
           &htim8,
           TIM_CHANNEL_1,
           Encoder4_A_GPIO_Port,
           Encoder4_A_Pin,
           Encoder4_B_GPIO_Port, 
           Encoder4_B_Pin
        );
  PID_Init(&motor4.pid, 0.0f, 0.0f, 0.0f, 90.0f, 300.0f); // PID参数初始化    
  //这个是旋转PID参数初始化
  PID_Init(&wz, 0.28f, 0.0f, 0.36f, 90.0f, 1000.0f); //这些的数值也需要实测调整
}

void PID_Callback1(Motor_Handle *motor, TIM_HandleTypeDef *htim, uint32_t channel){
         __disable_irq();
        if(motor->pid.capture_flag == 0){
            motor->pid.capture_flag = 1;
            motor->last_capture_val = HAL_TIM_ReadCapturedValue(htim, channel); //读取捕获值
        }else{
            motor->cur_capture_val = HAL_TIM_ReadCapturedValue(htim, channel);
            motor->interval += ((motor -> cur_capture_val - motor->last_capture_val) + 10000 * motor->overflow_cnt) * 1.0f / 1000.0f; // 计算时间间隔，单位ms
            motor->last_capture_val = motor->cur_capture_val; // 更新上一次捕获值
        //结果非零为高电平，置1，否则为0
        uint8_t state_a  = (motor->enc_port1->IDR & motor->enc_pin1) != 0;
        uint8_t state_b  = (motor->enc_port2->IDR & motor->enc_pin2) != 0;
        //判断是正传还是反转    
        if(state_a != state_b){
            motor->cur_direction = 1;
            motor->pulse_cnt ++;
        }else{
            motor->cur_direction = -1;
            motor->pulse_cnt --;
        }
    }
        motor->overflow_cnt = 0; // 捕获成功，重置溢出计数
        __enable_irq();    
}

void PID_Callback2(Motor_Handle *motor, TIM_HandleTypeDef *htim, uint32_t channel){
         __disable_irq();
        if(motor->pid.capture_flag == 0){
            motor->pid.capture_flag = 1;
            motor->last_capture_val = HAL_TIM_ReadCapturedValue(htim, channel); //读取捕获值
        }else{
            motor->cur_capture_val = HAL_TIM_ReadCapturedValue(htim, channel);
            motor->interval += ((motor -> cur_capture_val - motor->last_capture_val) + 10000 * motor->overflow_cnt) * 1.0f / 1000.0f; // 计算时间间隔，单位ms
            motor->last_capture_val = motor->cur_capture_val; // 更新上一次捕获值
            //结果非零为高电平，置1，否则为0
            uint8_t state_a  = (motor->enc_port1->IDR & motor->enc_pin1) != 0;
            uint8_t state_b  = (motor->enc_port2->IDR & motor->enc_pin2) != 0;
            //判断是正传还是反转    
            if(state_a == state_b){
                motor->cur_direction = 1;
                motor->pulse_cnt ++;
            }else{
                motor->cur_direction = -1;
                motor->pulse_cnt --;
            }
     
    }
        motor->overflow_cnt = 0; // 捕获成功，重置溢出计数
        __enable_irq();
}

void PID_Callback3(Motor_Handle *motor, TIM_HandleTypeDef *htim, uint32_t channel){
         __disable_irq();
        if(motor->pid.capture_flag == 0){
            motor->pid.capture_flag = 1;
            motor->last_capture_val = HAL_TIM_ReadCapturedValue(htim, channel); //读取捕获值
        }else{
            motor->cur_capture_val = HAL_TIM_ReadCapturedValue(htim, channel);
            motor->interval += ((motor -> cur_capture_val - motor->last_capture_val) + 10000 * motor->overflow_cnt) * 1.0f / 1000.0f; // 计算时间间隔，单位ms
            motor->last_capture_val = motor->cur_capture_val; // 更新上一次捕获值
        //结果非零为高电平，置1，否则为0
        uint8_t state_a  = (motor->enc_port1->IDR & motor->enc_pin1) != 0;
        uint8_t state_b  = (motor->enc_port2->IDR & motor->enc_pin2) != 0;
        //判断是正传还是反转    
        if(state_a == state_b){
            motor->cur_direction = 1;
            motor->pulse_cnt ++;
        }else{
            motor->cur_direction = -1;
            motor->pulse_cnt --;
        }    
    }
        motor->overflow_cnt = 0; // 捕获成功，重置溢出计数
        __enable_irq();
}

void PID_Callback4(Motor_Handle *motor, TIM_HandleTypeDef *htim, uint32_t channel){
         __disable_irq();
        if(motor->pid.capture_flag == 0){
            motor->pid.capture_flag = 1;
            motor->last_capture_val = HAL_TIM_ReadCapturedValue(htim, channel); //读取捕获值
        }else{
            motor->cur_capture_val = HAL_TIM_ReadCapturedValue(htim, channel);
            motor->interval += ((motor -> cur_capture_val - motor->last_capture_val) + 10000 * motor->overflow_cnt) * 1.0f / 1000.0f; // 计算时间间隔，单位ms
            motor->last_capture_val = motor->cur_capture_val; // 更新上一次捕获值
        //结果非零为高电平，置1，否则为0
        uint8_t state_a  = (motor->enc_port1->IDR & motor->enc_pin1) != 0;
        uint8_t state_b  = (motor->enc_port2->IDR & motor->enc_pin2) != 0;
        //判断是正传还是反转    
        if(state_a == state_b){
            motor->cur_direction = 1;
            motor->pulse_cnt ++;
        }else{
            motor->cur_direction = -1;
            motor->pulse_cnt --;
        }
          
    }
        motor->overflow_cnt = 0; // 捕获成功，重置溢出计数
        __enable_irq();
}

//外环控制函数，计算旋转PID输出
//yaw轴角速度
void wz_PID(PIDController *pid){
    pid->target_speed = target_vz; //更新目标速度为全局目标旋转速度
    pid->current_speed = encoder_yaw_speed(); //更新当前速度为IMU旋转速度

    //注意这里可以改为imu和编码器数据结合之后的，因为纯靠imu容易飘
    //pid->current_speed = yaw_speed_filter(encoder_yaw_speed());
    //pid->current_speed = Kalman_Filter(encoder_yaw_speed());
    pid->output = Axis_PID_compute(pid);//计算PID输出
    //注意这里是角速度闭环 
}
//Y轴速度
// void Y_PID(PIDController *pid){
//     pid->target_speed = target_vy; //更新目标速度为全局目标Y速度
//     Mecanum_ReadSpeed(); //更新全局变量en_fl_speed等为当前编码器速度
//     float en_speed = ((en_fl_speed + en_fr_speed + en_rl_speed + en_rr_speed) - (k_fr + k_rr - k_fl - k_rl) * Kalman_yaw) / 4.0f; //更新当前速度为编码器 Y速度
//     en_y_speed = en_speed;
//     pid->current_speed = Y_Kalman_Filter(en_speed); //这里对Y轴速度进行卡尔曼滤波，进一步减少噪声的影响
//     pid->output = Axis_PID_compute(pid);//计算PID输出

// }
//X轴速度
// void X_PID(PIDController *pid){
//     pid->target_speed = target_vx; //更新目标速度为全局目标X速度
//     Mecanum_ReadSpeed(); //更新全局变量en_fl_speed等为当前编码器速度
//     float en_speed = ((en_fl_speed - en_fr_speed - en_rl_speed + en_rr_speed) - (-k_fl - k_fr + k_rl + k_rr) * Kalman_yaw) / 4.0f; //更新当前速度为编码器 X速度 
//     en_x_speed = en_speed;
//     pid->current_speed = X_Kalman_Filter(en_speed); //这里对X轴速度进行卡尔曼滤波，进一步减少噪声的影响
//     pid->output = Axis_PID_compute(pid);//计算PID输出
// }

float yaw_angle_PD(float yaw_angle){
    static float yaw_error_last = 0;
	//锁头角度
	if(yaw_flag == 0){
		yaw_flag = 1;
		//yaw_angle =  cur_imu.yaw;
		yaw_error_last = 0;
	}
	//计算角度差
	float yaw_error = yaw_angle - cur_imu.yaw;  //注意这里可改为和编码器数据结合之后的，因为纯靠imu会飘
    //角度差边界处理
	if(yaw_error > 180){
		yaw_error -= 360;
	}else if(yaw_error < -180){
		yaw_error += 360;
	}
	//计算角度差变化
	float yaw_error_diff = yaw_error - yaw_error_last;
	yaw_error_last = yaw_error;
	//计算锁头修正速度
	float yaw_speed = yaw_error * 5 + yaw_error_diff * 5;	//这两个数值需要实测来调整

	//输出限幅
	//这个数也需要实测！
	if(yaw_speed > 50){
		yaw_speed = 50;
	}else if(yaw_speed < -50){
		yaw_speed = -50;
	}

    return yaw_speed * PI / 180.0f;
}

//逻辑有问题，需要检查
// void Position_Loop(Position_PID *pid){
//     //将车体坐标系位移转换为世界坐标系位移
//     pid->dx_world = pid->dx_body * cos((double)pid->angle) - pid->dy_body * sin((double)pid->angle);
//     pid->dy_world = pid->dx_body * sin((double)pid->angle) + pid->dy_body * cos((double)pid->angle);
//     pid->current_pos[0] += pid->dx_world;  //X轴
//     pid->current_pos[1] += pid->dy_world;  //Y轴
//     pid->position.X_world = Position_PID_compute(pid).X_world;
//     pid->position.Y_world = Position_PID_compute(pid).Y_world;
// }