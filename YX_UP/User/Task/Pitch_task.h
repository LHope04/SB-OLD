#ifndef PITCH_TASK_H
#define PITCH_TASK_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "INS_task.h"
#include "Exchange_task.h"
#include "Can_user.h"
#include "stm32f4xx_it.h"
#include "Motor.h"

//===============================================�궨��================================================//

#define Up_inf -12	//imu̧ͷ��λֵ
#define Down_inf 22	//imu��ͷ��λֵ
#define mouse_y_valve 10.f	//���������ֵ
#define mouse_y_weight 12.0f	//���ӳ��Ȩ��
#define Pitch_minipc_weight	0.5f	//�Ӿ���׼�ٶȻ�Ȩ��
#define Pitch_sita_minipc_weight 0.002f	//�Ӿ���׼λ�û�Ȩ��
#define Pitch_sita_weight 0.2f	//ң��������λ�û�Ȩ��
#define Pitch_up 4150	//������̧ͷ��λ
#define Pitch_down 3700	//��������ͷ��λ

void Pitch_task(void const * argument);
extern float target_pitch;

#ifdef __cplusplus
}
#endif

#endif