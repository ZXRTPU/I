#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "pid.h"

typedef struct
{
    uint16_t can_id;		//ID号
    int16_t  set_current;		//发送信息
    uint16_t rotor_angle;		//现在的角度
    int16_t  rotor_speed;		//现在的转速
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
	
	  float gyro_angle;//陀螺仪解算出的当前角度
	  int16_t  gyro_omega;	//陀螺仪解算出的当前角速度
	
	  float target_angle;//目标角度
	  int16_t target_speed;  //目标速度
	
	  float ZERO_gyro;
	  
	  //pid_struct_t gimbal_pid; //陀螺仪pid参数
	
}gimbal_motor_info_t;

//=============================变量定义=========================
//gimbal_motor_info_t yaw_motor;
//gimbal_motor_info_t pitch_motor;
//gimbal_motor_info_t roll_motor;

//=============================函数声明模块======================
void yaw_pid_init();

void gimbal_task(void const * argument);

void Gimbal_gyro_speed_current();

void Gimbal_gyro_angle_current();

void lock_gimbal_yaw();

void detel_gimbal(fp32* angle);

fp32 GXY_CXY(fp32 yaw_ground,fp32 chassis_ground);

void chassis_follow_yaw_mode();

#endif





