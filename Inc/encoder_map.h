#ifndef ENCODER_MAP_H
#define ENCODER_MAP_H

#include "main.h"
#include "rc_potocal.h"
#include "math.h"

//�Ե�ǰ�Ƕȼ�rotor_angle��������׼��������
int16_t encoder_map_8191(int16_t ZERO_POS,int16_t rotor_angle);

//���������ݱ�׼��
int16_t encoder_map_360(int16_t ZERO_gyro,int16_t gyro_angle);

#endif