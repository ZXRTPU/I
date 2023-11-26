#include "gimbal_task.h"
#include "Chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_map.h"
#include "encoder_map.h"
#include "drv_can.h"

gimbal_motor_info_t yaw_motor;
gimbal_motor_info_t pitch_motor;
gimbal_motor_info_t roll_motor;

extern motor_info_t  motor_info_chassis[4];       //电机信息结构体

fp32 yaw_speed_pid [3]={30,0.5,10};   //用的原来的pid
fp32 yaw_angle_pid[3]={2,0,0.3};

pid_struct_t yaw_motor_pid;

void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
		lock_gimbal_yaw();
		
		Gimbal_gyro_speed_current();
		
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}


//=======================================云台坐标系与底盘坐标系=================================================
fp32 GXY_CXY(fp32 yaw_ground,fp32 chassis_ground)
{
	fp32 yaw_chassis;
	yaw_chassis=yaw_ground-chassis_ground;
	
	return yaw_chassis;
}


//==========================================锁云台模式===========================================================
fp32 ZERO_yaw=0;  //上电时陀螺仪角度,在锁云台模式中可以看作云台的目标朝向
extern int16_t ZERO_pos[4];//要获取零位校准时电机的编码值

//结合角度环计算出yaw轴电机的目标速度
void lock_gimbal_yaw()
{
	fp32 err_yaw=0;
	
	ZERO_yaw=ZERO_yaw+(get_xy_angle_8191(ZERO_pos[2])/4096*180);
	
	detel_gimbal(&ZERO_yaw);
	
	err_yaw=ZERO_yaw- yaw_motor.gyro_angle;
	
	//零飘处理和目标角度计算
	if(err_yaw>1||err_yaw<-1)
	{
		pid_init(&yaw_motor_pid, yaw_angle_pid, 10000, 10000);  
		
		//此处求出的是云台相对与地面的速度
		yaw_motor.target_speed=pid_calc(&yaw_motor_pid,yaw_motor.gyro_angle,ZERO_yaw);
	}
	else 
	{
		yaw_motor.target_speed=0;
	}
}

//越界处理
void detel_gimbal(fp32* angle)
{
	if(*angle>180)
	{
		*angle=*angle-360;
	}
	else if(*angle<-180)
	{
		*angle=*angle+360;
	}
}

//=========================================底盘跟随云台模式=====================================================
//陀螺仪yaw轴的数据是相对于地面坐标系的绝对角度数据
//底盘跟随云台模式可以通过对底盘的PID控制（yaw轴是主动，底盘是从动）使yaw_chassis保持为零
void chassis_yaw_mode()
{
	
}


//=======================================云台电机PID输出计算及控制电流发送===============================================
fp32 gimbal_current=0;
//在已经获取到电机实时速度和由相应功能需求解算出目标速度的情况下，计算电机速度的PID输出和发送电流
void Gimbal_gyro_speed_current()
{
	  pid_init(&yaw_motor_pid, yaw_speed_pid, 6000, 6000);
				
		gimbal_current=pid_calc(&yaw_motor_pid,yaw_motor.rotor_speed,yaw_motor.target_speed);
				
	  //gimbal_current的值校验合格，才赋值给yaw轴电机结构体
		if(gimbal_current>yaw_motor.target_speed)
		{
			gimbal_current=0;
		}
				
		yaw_motor.set_current = gimbal_current;	
    		
	  set_motor_current_chassis(0,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current );
}

void Gimbal_gyro_angle_current()
{
			  pid_init(&yaw_motor_pid, yaw_angle_pid, 10000, 10000);  
			
			  yaw_motor.target_speed=pid_pitch_calc(&yaw_motor_pid,yaw_motor.gyro_angle,yaw_motor.target_angle);
			
			  if((get_x_ch1()==0)&&(get_y_ch0()==0))
				{
					yaw_motor.target_speed=0;
				}
			
				pid_init(&yaw_motor_pid, yaw_speed_pid, 6000, 6000);
				
				gimbal_current=pid_calc(&yaw_motor_pid,yaw_motor.rotor_speed,yaw_motor.target_speed);
				
				if(gimbal_current>yaw_motor.target_speed)
				{
					gimbal_current=0;
				}
				
				yaw_motor.set_current = gimbal_current;
				
				set_motor_current_chassis(0,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current );
}



