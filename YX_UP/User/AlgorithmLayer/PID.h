#ifndef __PID_H
#define __PID_H
#include "main.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))		//Խ���򸳱߽�ֵ


typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float ref;      // target value
  float fdb;      // feedback value  
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;


void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
							
float pid_calc(pid_struct_t *pid, float ref, float fdb);	//�ٶȻ�PID����,ref���ٶ�Ŀ��ֵ��fdb���ٶȷ���ֵ
float pid_calc_sita(pid_struct_t *pid,float ref, float fdb);	//λ�û�PID���㣬��λ�ǽǶȣ�ref�ǽǶ�Ŀ��ֵ��fdb�ǽǶȷ���ֵ

//================================================PID�ṹ������================================================//
extern pid_struct_t motor_pid[7];	
extern pid_struct_t motor_pid_sita[7];
extern pid_struct_t motor_pid_can_2[7];	
extern pid_struct_t motor_pid_sita_can_2[7];
							
#endif
							