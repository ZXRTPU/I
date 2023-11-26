#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#include "rc_map.h"
#include "gimbal_task.h"
#include "pid.h"

#define LENGTH_A 
#define LENGTH_B


extern gimbal_motor_info_t yaw_motor;

pid_struct_t motor_pid_chassis[4];
motor_info_t  motor_info_chassis[4];       //电机信息结构体
volatile int16_t motor_speed_target[4];
volatile uint16_t motor_angle_target[4];

//PID的参数数组
fp32 motor_speed_pid [3]={30,0.5,10};   //用的原来的pid
fp32 motor_angle_pid[3]={2,0,0.3};

int16_t ZERO_pos[4]={0};//要获取零位校准时底盘电机的编码值

pid_struct_t chassis_pid_direct;
chassis_direct_t chassis_direct;

/*
int16_t Temp_Vx;
int16_t Temp_Vy;

int fllowflag = 0;*/


extern RC_ctrl_t rc_ctrl;

extern float powerdata[4];
extern uint16_t shift_flag;
int8_t chassis_mode = 1;//判断底盘状态，用于UI编写

double a=0;
int16_t b=0;

void  Chassis_task(void const * argument)
{    
    /*  
	  osDelay(10);

    ZERO_pos[2]= motor_info_chassis[2].rotor_angle;
	 */
	
	 	for (uint8_t i = 0; i < 4; i++)
	  {
        pid_init(&motor_pid_chassis[i], motor_speed_pid, 6000, 6000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
	  } 

		pid_init(&chassis_pid_direct,motor_angle_pid,10000,10000);
    
    for(;;)				//底盘运动任务
    {     
        //情形一：控器传回的小车运动情况计算小车四个电机的目标转速
         chassis_motol_speed_calculate();
		
        //PID计算应该发送给4个电机分别的电流
         chassis_current_give();
            
         osDelay(1);

    }

}

//========================public-发送底盘控制电流================================================================
volatile int16_t Vx=0,Vy=0,Wz=0;

static void Chassis_loop_Init()
{
	Vx = 0;
	Vy = 0;
	Wz = 0;
	
	//不同控制模式下目标速度的设置
	
	if(rc_ctrl.rc.s[0]==1)//上拨
	{
	  Vx=rc_ctrl.rc.ch[0];
	  Vy=rc_ctrl.rc.ch[1];
	  Wz=rc_ctrl.rc.ch[2];
	}
	else if(rc_ctrl.rc.s[0]==2)//下拨
	{
		lock_chassis_direct();
	}
}

//运动解算
void chassis_motol_speed_calculate(void)
{
	
	  motor_speed_target[CHAS_LF] =  0;
    motor_speed_target[CHAS_RF] =  0;
    motor_speed_target[CHAS_RB] =  0; 
    motor_speed_target[CHAS_LB] =  0;
	
	  motor_speed_target[CHAS_LF] =Vx -Vy+Wz;
    motor_speed_target[CHAS_RF] =Vx +Vy +Wz;
    motor_speed_target[CHAS_RB] =-Vx+Vy+Wz; 
    motor_speed_target[CHAS_LB] =-Vx -Vy+Wz;
}

//速度限制函数
  void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }

}

//电机电流控制
void chassis_current_give() 
{
	
    uint8_t i=0;
        
    for(i=0 ; i<4; i++)
    {                                                                       
        motor_info_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
    }
    set_motor_current_chassis(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);

}
//=========================================================================================================================

//============================底盘任务模式及模式切换===============================================
//============================1.底盘朝向固定
//=================1.1底盘搭载陀螺仪的情况
fp32 ZERO_chassis=0;
void lock_chassis_direct()
{
	fp32 chassis_err=0;
	
	chassis_err=chassis_direct.gyro_angle-ZERO_chassis;
	
	detel_chassis(&chassis_err);
	
	//此处的目标速度是整车的目标旋转角速度Wz
	chassis_direct.target_speed=pid_calc(&chassis_pid_direct,-chassis_err,0);
	
	Wz=chassis_direct.target_speed;
}

//越界处理
void detel_chassis(fp32* angle)
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

//=================1.2只有云台搭载陀螺仪-底盘跟随（通过yaw轴电机编码器和云台陀螺仪控制）
//云台与底盘相联系的部分：yaw轴6020
//底盘跟随云台，即使编码器的编码值保持不变
fp32 yaw_err=0;

void Chassis_following()
{
	fp32 Wz_max=6000;
	fp32 angle_value=3;
	fp32 angle_weight=30;
	
	yaw_err=Get_err();
	
	if(yaw_err>3||yaw_err<-3)
	{
		Wz=yaw_err*angle_weight;//设Wz的转速为 角度差*权重，让底盘转回去
	}
	
	//速度限幅
	if(Wz>Wz_max)
	{
		Wz=Wz_max;
	}
	else if(Wz<-Wz_max)
	{
	  Wz=-Wz_max;
	}
}

//以yaw轴陀螺仪的值维目标值，底盘陀螺仪为当前值
float Get_err()
{
	 yaw_err=encoder_map_360(yaw_motor.ZERO_gyro,yaw_motor.gyro_angle)
	          -encoder_map_360(chassis_direct.ZERO_gyro,chassis_direct.gyro_angle);
	if(yaw_err>180)
	{
		yaw_err=yaw_err-360;
	}
	else if(yaw_err<-180)
	{
		yaw_err=yaw_err+360;
	}
	
	return yaw_err;
}



//============================2.小陀螺模式
//=================2.1底盘高速旋转时，云台朝向保持不变
//=================2.2底盘高速旋转的同时，云台朝向可以用遥控器调控
//不管底盘是不是在旋转，只要保持陀螺仪的朝向不变就可以了
void small_xiaoyuoluo()
{
	fp32 ZERO_gimbal_gyro=0;
	ZERO_gimbal_gyro= ZERO_gimbal_gyro+get_xy_angle_8191(ZERO_gimbal_gyro);
	
	fp32 err_gyro=0;
	gimbal_motor_info_t yaw_gyro;
	
	err_gyro=yaw_gyro.gyro_angle-ZERO_gimbal_gyro;
	
	if(err_gyro>180)
	{
		err_gyro-=360;
	}
	else if(err_gyro<-180)
	{
		err_gyro+=360;
	} 
	
	//yaw_gyro.target_speed=pid_calc(-err_gyro,0)
	//yaw_gyro.set_current=pid_calc(yaw_gyro.rotor_speed,yaw_gyro.target_speed)
		
}


//============================用遥控器控制6020转过固定角度=========================================
fp32 current=0;
//这个函数在步兵中应该用于云台gimbal的控制
void chassis_current_give_RC_6020() 
{
	
    uint8_t i=0;
        
    for(i=0 ; i<4; i++)
    {
			  pid_init(&motor_pid_chassis[i], motor_angle_pid, 10000, 10000);  
			
			  motor_speed_target[i]=pid_pitch_calc(&motor_pid_chassis[i],
			                                        encoder_map_8191(ZERO_pos[i],motor_info_chassis[i].rotor_angle),
			                                         get_xy_angle_8191(ZERO_pos[2]));
			
			  a=get_xy_angle_8191(ZERO_pos[2]);
			  b=encoder_map_8191(ZERO_pos[2],motor_info_chassis[2].rotor_angle);
			
			  if((get_x_ch1()==0)&&(get_y_ch0()==0))
				{
					motor_speed_target[i]=0;
				}
			
				pid_init(&motor_pid_chassis[i], motor_speed_pid, 6000, 6000);
				
				current=pid_calc(&motor_pid_chassis[i], motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
				
				if(current>motor_speed_target[i])
				{
					current=0;
				}
				
				motor_info_chassis[i].set_current = current;
    }
    	
		set_motor_current_chassis(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);

}



/*
			for(a=0;a<4096;a+=512)
			{
				while(encoder_map_8191(ZERO_pos[2],motor_info_chassis[2].rotor_angle)!=a)
				{
					chassis_current_give_1();
				}
				
				osDelay(100);
				
			}*/
