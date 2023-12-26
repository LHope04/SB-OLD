#ifndef YAW_TASK_H
#define YAW_TASK_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "Exchange_task.h"
#include "main.h"
#include "pid.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "INS_task.h"
#include "Motor.h"

//================================================�궨��================================================//
#define Rotate_gain 1.23f		//���ٶ�ǰ������ӳ��Ȩ��
#define Chassis_R	30.0f		//����ת�뾶
#define Chassis_r 7.5f		//���ְ뾶
#define valve 20		//ң������ֵ(����)
#define base 1024		//ң�����Ļ���ֵ
#define base_max 1684		//ң�������ֵ
#define base_min 364		//ң������Сֵ
#define angle_valve 1		//�Ƕ���ֵ���������Χ�ھͲ�ȥ������(����yaw��ʱʹ��)
#define mouse_x_valve 10.0f		//��������ֵ
#define mouse_x_weight 0.5f		//���ӳ��Ȩ��
#define Yaw_sita_weight 0.5f 		//ң��������λ�û�Ȩ��
#define Yaw_minipc_weight 1.75f		//�Ӿ�����ӳ��Ȩ��(�ٶȰ�)
#define Yaw_minipc_sita_weight 0.003f	//�Ӿ�����ӳ��Ȩ��(λ�ð�)

//================================================����================================================//
extern int16_t Rotate_w;
extern int16_t Rotate_W;
extern fp32 target_yaw;
extern int8_t Update_yaw_flag;
extern float target_yaw;

//���庯��
void Yaw_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif

#endif