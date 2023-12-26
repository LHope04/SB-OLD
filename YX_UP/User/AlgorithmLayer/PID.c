#include "PID.h"

//================================================PID�ṹ�嶨��================================================//
pid_struct_t motor_pid[7];	
pid_struct_t motor_pid_sita[7];
pid_struct_t motor_pid_can_2[7];	
pid_struct_t motor_pid_sita_can_2[7];

//================================================PID��ʼ��================================================//
//	Input parameter: 
//	struct pid 
//	p
//	i
//	d
//	�����޷����ֵi_max
//	pid���������ֵout_max

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

//================================================PID�ٶȻ�================================================//
//	Input parameter:
//	struct pid
//	target Ŀ���ٶ�ֵ---->��ӦPid�ṹ������е�ref
//	respend ��ǰ�ٶ�ֵ(�������ֵ)---->��ӦPid�ṹ������е�fdb

float pid_calc(pid_struct_t *pid, float target, float respond)
{
  pid->ref = target;
  pid->fdb = respond;
  pid->err[1] = pid->err[0];//err[1]����һ�μ�������Ĳ�ֵ
  pid->err[0] = pid->ref - pid->fdb;//err[0]����һ�ε�Ԥ���ٶȺ�ʵ���ٶȵĲ�ֵ,������ֵ�ǿ����Ǹ�����
  
  pid->p_out  = pid->kp * pid->err[0];//40 3 0�Ǳ�׼ֵ��������ӵ�watch1����
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//��ֹԽ��
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//��ֹԽ��
  return pid->output;
}

//================================================PID�ǶȻ�================================================//
//	Input parameter:
//	struct pid
//	target Ŀ��Ƕ�ֵ---->��ӦPid�ṹ������е�ref
//	respend ��ǰ�Ƕ�ֵ(�������ֵ)---->��ӦPid�ṹ������е�fdb
//	ע�⣺��λ�ǽǶ�(�ǻ���)

float pid_calc_sita(pid_struct_t *pid,float target, float respond) //λ�û�PID���㣬��λ�ǽǶȣ�ref�ǽǶ�Ŀ��ֵ��fdb�ǽǶȷ���ֵ
{
  pid->ref = target;
  pid->fdb = respond;
	pid->err[1] = pid->err[0];
	
	float err = 0;
	err = target - respond;
	//Խ�紦��
	if(err > 180)
	{
		err -= 360;
	}
	else if(err < -180)
	{
		err += 360;
	}
  pid->err[0] = err;
	
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
	
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//��ֹiֵ����Խ��
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//��ֹ����Խ��
  return pid->output;
}
