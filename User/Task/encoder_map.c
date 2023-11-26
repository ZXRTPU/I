#include "encoder_map.h"

//对当前角度即rotor_angle整定到标准编码盘上，返回当前角度
int16_t encoder_map_8191(int16_t ZERO_POS,int16_t rotor_angle)
{
	int16_t k=0;
	 k=ZERO_POS;
	
	//n=uint16_t rotor_angle;
  int16_t n=0;
	n=rotor_angle;
	
	if(n>(k+4096))
	{
		n=n-8192;
	}
	else if((k<=n)&&(n<=(k+4096)))
	{
		n=n-k;
	}
	else if(n<(k-4096))
	{
		n=n+8192;
	}
	else if(((k-4096)<=n)&&(n<k))
	{
		n=n-k;
	}

	return n;
}

int16_t encoder_map_360(int16_t ZERO_gyro,int16_t gyro_angle)
{
	int16_t k=0;
	 k=ZERO_gyro;
	
	//n=uint16_t rotor_angle;
  int16_t n=0;
	n=gyro_angle;
	
	if(n>(k+180))
	{
		n=n-360;
	}
	else if((k<=n)&&(n<=(k+180)))
	{
		n=n-k;
	}
	else if(n<(k-180))
	{
		n=n+360;
	}
	else if(((k-180)<=n)&&(n<k))
	{
		n=n-k;
	}

	return n;
}
