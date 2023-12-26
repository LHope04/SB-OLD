#ifndef _MOTOR_H
#define _MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
typedef struct
{
    uint16_t can_id;		//ID��
    int16_t  set_current;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
}motor_info_t;

extern motor_info_t  motor_info_chassis[8];//Can_1�������

#ifdef __cplusplus
}
#endif

#endif 