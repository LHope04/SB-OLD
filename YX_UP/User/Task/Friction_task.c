
#include "Friction_task.h"

//================================================���������������================================================//
//��һ�棺
//�������дĦ���ֺͲ��̣��²ࣩ
//Ħ���ֵ�ID�ֱ���1��2   ---   ��C��CAN_2
//���̵�ID��5   ---   CAN_1��������ͨ��

//�ڶ��棺
//�����������Ħ���ֺͲ���(�ϲ�)
//��Ħ����ID�ֱ���3��4   ---   ��C��CAN_2
//�������̣�����ID��6   ---   CAN_1��������ͨ��

//===============================================����================================================//
//PID��ʼ��
static void Friction_init();

//Ħ���ּ��ٸ�ֵ
static void Friction_calc();

//Ħ���ּ��ٸ�ֵ
static void Friction_down();

//Ħ���ֳ����ж�
static bool Friction_judge();

//Ħ����Pid���ֵ����
static void Friction_send();

//����Pid���ֵ�ͷ���
static void Bopan_send();

//���̶�ת���
static void Bopan_judge();

//����PId����
static void Bopan_calc(int16_t speed);

//===============================================ȫ�ֱ���================================================//
int16_t bopan_shoot_speed = 20*36;//90*36;	//���̷��䵯��ת��
int16_t bopan_reversal_speed = -35*36;	//���̷�תת��
uint8_t bopan_reversal_flag = 0;	//���̷�ת��־λ��0Ϊ��ת��1Ϊ��ת

void Friction_task(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();	//PID��ʼ��
	
  for(;;)
  {
		//===============================================Ħ����================================================//
		if(rc_ctrl.rc.s[0] == 2)	//����Ħ����
		{
			Friction_calc();	//ת��->����
		}
		else
		{
			Friction_down();	//Ħ���ּ��ٵ���
		}
		Friction_send();	//Ħ���ֵ�������
		
		//===============================================����================================================//
		//Bopan_judge();	//��������ת��⣬���Ժ�����������
		if(rc_ctrl.rc.s[1] == 1)	//��������(����ģʽ)
		{	
			if(!bopan_reversal_flag)	//������ת
			{
				Bopan_calc(bopan_shoot_speed);
			}
			else if(bopan_reversal_flag)	//���̷�ת
			{
				Bopan_calc(bopan_reversal_speed);
			}
			
		}
		else if(rc_ctrl.rc.s[1] == 2  && Sentry.Flag_shoot)//��⵽Ŀ��
		{
			if(!bopan_reversal_flag)	//������ת
			{
				Bopan_calc(bopan_shoot_speed);
			}
			else if(bopan_reversal_flag)	//���̷�ת
			{
				Bopan_calc(bopan_reversal_speed);
			}
		}
		else
		{			
			Bopan_calc(0);
		}
		Bopan_send();	//����PID����
		
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

//===============================================PID��ʼ��================================================//
static void Friction_init()
{
	pid_init(&motor_pid_can_2[0],40,0.8,1,16384,16384);//Ħ����
	pid_init(&motor_pid_can_2[1],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[2],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[3],40,0.8,1,16384,16384);
	
	pid_init(&motor_pid[4],20,0.03,0.5,16384,16384);//����(��צ)
	pid_init(&motor_pid[5],20,0.03,0.5,16384,16384);
}

//===============================================Ħ����ת��->����================================================//
static void Friction_calc()
{
	target_speed_can_2[0]=-19*350;
	target_speed_can_2[1]=19*350;
	target_speed_can_2[2]=-19*350;
	target_speed_can_2[3]=19*350;
	
	motor_info_can_2[0].set_voltage = pid_calc(&motor_pid_can_2[0], target_speed_can_2[0], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
	motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);
}

//===============================================Ħ���ּ��ٵ���================================================//
static void Friction_down()
{
	target_speed_can_2[0]=0;
	target_speed_can_2[1]=0;
	target_speed_can_2[2]=0;
	target_speed_can_2[3]=0;
	motor_info_can_2[0].set_voltage = pid_calc(&motor_pid_can_2[0], target_speed_can_2[0], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
	motor_info_can_2[3].set_voltage = pid_calc(&motor_pid_can_2[3], target_speed_can_2[3], motor_info_can_2[3].rotor_speed);
}

//===============================================Ħ����ת���ж�(δʹ��)================================================//
static bool Friction_judge()
{
	if(	(motor_info_can_2[1].rotor_speed>=8500) && (motor_info_can_2[2].rotor_speed<=-8500) )
	{
		return true;
	}
	return false;
}

//===============================================����PID����================================================//
static void Bopan_calc(int16_t speed)
{
	motor_info[4].set_voltage=pid_calc(&motor_pid[4],speed,motor_info[4].rotor_speed);
	motor_info[5].set_voltage=pid_calc(&motor_pid[5],-speed,motor_info[5].rotor_speed);	
}

//===============================================Ħ���ֵ������ͺ���================================================//
static void Friction_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x200;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (motor_info_can_2[0].set_voltage>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] =    (motor_info_can_2[0].set_voltage)&0xff;
  tx_data[2] = (motor_info_can_2[1].set_voltage>>8)&0xff;
  tx_data[3] =    (motor_info_can_2[1].set_voltage)&0xff;
  tx_data[4] = (motor_info_can_2[2].set_voltage>>8)&0xff;
  tx_data[5] =    (motor_info_can_2[2].set_voltage)&0xff;
  tx_data[6] = (motor_info_can_2[3].set_voltage>>8)&0xff;
  tx_data[7] =    (motor_info_can_2[3].set_voltage)&0xff;
	
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);
}

//===============================================���̵������ͺ���================================================//
static void Bopan_send()
{

	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = 0x1ff;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (motor_info[4].set_voltage>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] =    (motor_info[4].set_voltage)&0xff;
  tx_data[2] = (motor_info[5].set_voltage>>8)&0xff;
  tx_data[3] =    (motor_info[5].set_voltage)&0xff;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = 0;
  tx_data[7] = 0;
	
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);
}

//===============================================���̶�ת���================================================//
static void Bopan_judge()
{
	if(motor_info[4].torque_current > 6000)//�� monitor��һ�£���һ�����paramater������ֵ
	{
		bopan_reversal_flag = 1;	//��Ϊ��ת	
	}
	
	if(motor_info[4].torque_current < -6000)
	{
		bopan_reversal_flag = 0;	//��Ϊ��ת	
	}
}