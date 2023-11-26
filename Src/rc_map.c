#include "rc_map.h"
#include "math.h"

extern  RC_ctrl_t rc_ctrl;

double  get_x_ch1()
{
	double x=0; 
  x=(double)rc_ctrl.rc.ch[1]-1024; //x处于-660~660之间
	
	return x;
}

double  get_y_ch0()
{
	int y=0; 
  y=(double)rc_ctrl.rc.ch[0]-1024; //y处于-660~660之间
	
	return y;
}

//将从遥控器获取的角度值整定到标准编码盘上,返回目标角度
double get_xy_angle_8191(uint16_t ZERO_POS)
{
	double angle=0.0f;
	
	if(get_x_ch1()>0)
	{
		angle=atan2(get_y_ch0(),get_x_ch1());
	}
	else if((get_x_ch1()==0)&&(get_y_ch0()>0))
	{
		angle=PI/2;
	}
	else if((get_x_ch1()==0)&&(get_y_ch0()<0))
	{
		angle=-PI/2;
	}
	else if((get_x_ch1()<0)&&(get_y_ch0()>=0))
	{
		angle=atan2(get_y_ch0(),get_x_ch1())+PI;
	}
	else if((get_x_ch1()<0)&&(get_y_ch0()<0))
	{
		angle=atan2(get_y_ch0(),get_x_ch1())-PI;
	}
	else
	{
		angle=0;
	}
	
	//以上得到的角度是弧度制的，处于-PI~PI之间，需要转换为电机对应的编码器范围-4096~4096
	
	angle=(angle/PI)*4096;
	
	return angle;
}







