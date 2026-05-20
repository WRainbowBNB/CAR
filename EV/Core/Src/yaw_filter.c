#include "yaw_filter.h"

float Kalman_yaw = 0.0f;

Kalman yaw_filter = {
    .P = 1.0f,
    .P_pre = 0.0f,
    .Q_last = 0.01f,    //底噪
    .Q_new = 0.0f,
    .w_k = 0.0f,
    .w_pre = 0.0f,
    .error1 = 0.0f,
    .error2 = 0.0f,
    .residual = 0.0f,
};

Axis_Kalman x_filter = {
    .target_v = 0.0f,
    .P_pre = 0.0f,
    .P_last = 1.0f,
    .Q_last = 0.01f,
    .Q_new = 0.0f,
    .v_pre = 0.0f,
    .v_k = 0.0f,
};

Axis_Kalman y_filter = {
    .target_v = 0.0f,
    .P_pre = 0.0f,
    .P_last = 1.0f,
    .Q_last = 0.01f,
    .Q_new = 0.0f,
    .v_pre = 0.0f,
    .v_k = 0.0f,
};

Axis_Kalman yaw_filter_en = {
    .target_v = 0.0f,
    .P_pre = 0.0f,
    .P_last = 1.0f,
    .Q_last = 0.01f,
    .Q_new = 0.0f,
    .v_pre = 0.0f,
    .v_k = 0.0f,
};

float encoder_yaw_speed(){
    Mecanum_ReadSpeed(); //更新全局变量en_fl_speed等为当前编码器速度
    //通过编码器计算yaw速度
    float en_yaw_speed = (-en_fl_speed + en_fr_speed - en_rl_speed + en_rr_speed) / ((k_fl + k_fr + k_rl + k_rr));
    return en_yaw_speed;
}

float yaw_speed_filter(float en_yaw_speed){
    float fil_yaw_speed = K_ENCODER1 * en_yaw_speed + K_IMU1 * cur_imu.wz;
    return fil_yaw_speed;
}

/**
 * @brief 卡尔曼滤波函数
 * @param en_yaw_speed 编码器计算出的角速度值
 * @param cur_imu_wz 当前imu的角速度值
 * @return 滤波后的有效角速度值
 */
float Kalman_Filter(float en_yaw_speed){
    float cur_imu_wz = cur_imu.wz * PI / 180.0f;
    //预测阶段
    yaw_filter.w_pre = yaw_filter.w_k + PID_T *(target_vz - yaw_filter.w_k) / RESPONSE_TIME;
    yaw_filter.P_pre = yaw_filter.P + yaw_filter.Q_new;
    yaw_filter.error1 = en_yaw_speed - yaw_filter.w_pre;
    yaw_filter.error2 = cur_imu_wz - yaw_filter.w_pre;
    //计算卡尔曼增益
    yaw_filter.M = 1/(yaw_filter.P_pre * (R_ENCODER + R_IMU) + R_ENCODER * R_IMU + 1e-6f); //加小值防止除零
    yaw_filter.K_ENCODER = yaw_filter.M * yaw_filter.P_pre * R_IMU;
    yaw_filter.K_IMU = yaw_filter.M * yaw_filter.P_pre * R_ENCODER;
    yaw_filter.residual = yaw_filter.K_ENCODER * yaw_filter.error1 + yaw_filter.K_IMU * yaw_filter.error2;
    yaw_filter.Q_new = (1- ALPHA) * yaw_filter.Q_last + ALPHA * yaw_filter.residual * yaw_filter.residual; 
    //修正阶段
    yaw_filter.w_k = yaw_filter.w_pre + yaw_filter.residual;
    yaw_filter.P = (1 - yaw_filter.K_ENCODER - yaw_filter.K_IMU) * yaw_filter.P_pre;
    yaw_filter.Q_last = yaw_filter.Q_new;
    return yaw_filter.w_k;
}

float X_Kalman_Filter(float vx){
    //预测阶段
    x_filter.v_pre = x_filter.v_k;
    x_filter.P_pre = x_filter.P_last + x_filter.Q_last;
    float residual = vx - x_filter.v_pre;
    x_filter.Q_new = (1- X_ALPHA) * x_filter.Q_last + X_ALPHA * residual * residual;
    if(x_filter.Q_new < 0.00001f) x_filter.Q_new = 0.00001f;
    //修正阶段
    float K = x_filter.P_pre / (x_filter.P_pre + X_R);
    x_filter.v_k = x_filter.v_pre + K * residual;
    x_filter.P_last = (1 - K) * x_filter.P_pre;
    x_filter.Q_last = x_filter.Q_new;
    return x_filter.v_k;
}

float Y_Kalman_Filter(float vy){
    //预测阶段
    y_filter.v_pre = y_filter.v_k;
    y_filter.P_pre = y_filter.P_last + y_filter.Q_last;
    float residual = vy - y_filter.v_pre;
    y_filter.Q_new = (1- Y_ALPHA) * y_filter.Q_last + Y_ALPHA * residual * residual;
    if(y_filter.Q_new < 0.00001f) y_filter.Q_new = 0.00001f;
    //修正阶段
    float K = y_filter.P_pre / (y_filter.P_pre + Y_R);
    y_filter.v_k = y_filter.v_pre + K * residual;
    y_filter.P_last = (1 - K) * y_filter.P_pre;
    y_filter.Q_last = y_filter.Q_new;
    return y_filter.v_k;
}

float YAW_Kalman_Filter(float vz){
    //预测阶段
    yaw_filter_en.v_pre = yaw_filter_en.v_k;
    yaw_filter_en.P_pre = yaw_filter_en.P_last + yaw_filter_en.Q_last;
    float residual = vz - yaw_filter_en.v_pre;
    yaw_filter_en.Q_new = (1- Z_ALPHA) * yaw_filter_en.Q_last + Z_ALPHA * residual * residual;
    if(yaw_filter_en.Q_new < 0.00001f) yaw_filter_en.Q_new = 0.00001f;
    //修正阶段
    float K = yaw_filter_en.P_pre / (yaw_filter_en.P_pre + Z_R);
    yaw_filter_en.v_k = yaw_filter_en.v_pre + K * residual;
    yaw_filter_en.P_last = (1 - K) * yaw_filter_en.P_pre;
    yaw_filter_en.Q_last = yaw_filter_en.Q_new;
    return yaw_filter_en.v_k;
}