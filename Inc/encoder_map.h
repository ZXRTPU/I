#ifndef ENCODER_MAP_H
#define ENCODER_MAP_H

#include "main.h"
#include "rc_potocal.h"
#include "math.h"

//对当前角度即rotor_angle整定到标准编码盘上
int16_t encoder_map_8191(int16_t ZERO_POS,int16_t rotor_angle);

//陀螺仪数据标准化
int16_t encoder_map_360(int16_t ZERO_gyro,int16_t gyro_angle);

#endif