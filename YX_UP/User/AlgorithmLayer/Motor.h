#ifndef MOTOR_H
#define MOTOR_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "main.h"
//================================================����ṹ��================================================//
typedef struct
{
    uint16_t can_id;		//ID��
    int16_t  set_voltage;		//������Ϣ
    uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
}moto_info_t;

//================================================����ṹ������================================================//
extern float target_speed[7];	//����׷���ٶ�
extern float target_speed_can_2[7];
extern moto_info_t motor_info[7];		//��������7���ֽ�
extern moto_info_t motor_info_can_2[7];

	 
#ifdef __cplusplus
}
#endif

#endif