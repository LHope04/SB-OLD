#include "Pitch_task.h"

//================================================Pitch������������================================================//

//��һ�棺
//��������������̨����ģʽѡ�񣬿��ƣ�У׼��
//��װһЩ�����������п��Ƶ���
//Pitch������C��CAN_2�����IDΪ5
//�������⣺��Ҫ����̨Pitch�������·���������Pitch�ܸ�Ƶ���Ӱ�죬�����ٶ�̫��

//�ڶ��棺
//����˫6020�ṹ���¼ӵ�Pitch����C��CAN_2�����IDΪ6
//Pitchֵ�Ǹ��ģ�̧ͷ����ֵ����(�����Ǹ���)


//===============================================ȫ�ֱ���================================================//

uint16_t gimbal_rotor_angle;		//IDΪ5��6020������ֵ
uint16_t gimbal_rotor_angle_2;		//IDΪ6��6020������ֵ
float Pitch_imu;		//pitch imu��������
float Pitch_imu_speed;		//pitch imu���ٶ� 
float target_pitch;		//Ŀ��pitch
float ins_pitch_speed;

//================================================����================================================//

//��ʼ��PID����
static void gimbal_init();	

//��������
static void gimbal_zero();

//��ȡimu����
static void gimbal_read_imu();

//��ȡ��������ֵ
static void gimbal_read_motor();

//ң��������ģʽ(�ٶȻ�)
static void gimbal_control_speed();

//Ѳ��ģʽ(�ٶȻ�)
static void gimbal_mode_search_speed();

//ң��������ģʽ(λ�û�)
static void gimbal_mode_control_sita();

//Ѳ��ģʽ(λ�û�)
static void gimbal_mode_search_sita();

//PID���������
static void gimbal_can_send();

//��λ������������δʹ�ã�
static void gimbal_limit();

//��λ�������ǣ�
static void gimbal_imu_limit();

//������Pitch(����)
static void gimbal_mouse();

//��������(�ٶȻ�)
static void gimbal_minipc_control();

//��������(λ�û�)
static void gimbal_minipc_control_sita();

// pitch

void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */

	gimbal_init();	//PID������ʼ��
	osDelay(3000);
  for(;;)
  {
		gimbal_zero();	//�ٶ�����
		gimbal_read_motor();	//��ȡ������ֵ
		gimbal_read_imu();	//��ȡImuֵ
		if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1	)	//����ģʽ
		{
			gimbal_minipc_control_sita();	//λ�û��Ӿ���׼
			gimbal_mode_control_sita();	//ң����λ�û�����ģʽ
		 }
		  
		else if(rc_ctrl.rc.s[1]==2)		//�ϳ�ģʽ
		{
			if(Sentry.foe_flag)	//�����⵽Ŀ��
			{
				gimbal_minipc_control_sita(); //�Ӿ���׼
			}			
			else
			{
				gimbal_mode_search_sita();	//�ڱ�Ѳ��ģʽ
			}
		}
		gimbal_imu_limit();	//�����λ
		target_speed_can_2[4] -= pid_calc_sita(&motor_pid_sita_can_2[4], target_pitch, Pitch_imu);	//imuλ�û�����
		//target_speed_can_2[5] += pid_calc_sita(&motor_pid_sita_can_2[5], target_pitch, Pitch_imu);
		
//		����������ģʽ
//		target_speed_can_2[4] -= pid_calc_sita(&motor_pid_sita_can_2[4], target_pitch*(-22.75)+3632, motor_info_can_2[4].rotor_angle);	//������λ�û�����(ʹ�����ʱע��ң����ӳ��Ȩ��)
//		target_speed_can_2[5] += pid_calc_sita(&motor_pid_sita_can_2[5], target_pitch*(22.75)+2474, motor_info_can_2[5].rotor_angle);


		ins_pitch_speed = Pitch_imu_speed * 9.55f;	//ͳһ�ٶȻ���λ���Ա�����Ϊ��׼��rad/s -> round/min	60/(2*pi)=9.55
		motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], ins_pitch_speed);
		motor_info_can_2[5].set_voltage = -motor_info_can_2[4].set_voltage;
		//motor_info_can_2[5].set_voltage = pid_calc(&motor_pid_can_2[5], target_speed_can_2[5], ins_pitch_speed );
		
//		����������ģʽ
//		motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], motor_info_can_2[4].rotor_speed);
//		motor_info_can_2[5].set_voltage = pid_calc(&motor_pid_can_2[5], target_speed_can_2[5], motor_info_can_2[5].rotor_speed);
			gimbal_can_send();
		
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//================================================��ʼ��PID����================================================//
static void gimbal_init()	
{
	
	pid_init(&motor_pid_can_2[4],120,0.001,5,30000,30000);// 120 0.01 0
	pid_init(&motor_pid_can_2[5],120,0.001,5,30000,30000);// 120 0.01 0
	pid_init(&motor_pid_sita_can_2[4],18,0,350,30000,30000);// 10 0 1300
	pid_init(&motor_pid_sita_can_2[5],18,0,350,30000,30000);// 10 0 1300
	target_pitch = Pitch_imu;
} 

//================================================��ȡ������ֵ================================================//
static void gimbal_read_motor()
{
	gimbal_rotor_angle = motor_info_can_2[4].rotor_angle;	//���·�:0E30   ���Ϸ�:10B0  ����:3632->4272  ,������640
	gimbal_rotor_angle_2 = motor_info_can_2[5].rotor_angle; //���·�:09AA   ���Ϸ�:0730   ����:2474->1840  ,������634
}

//================================================��ȡimuֵ================================================//
static void gimbal_read_imu()
{
	Pitch_imu = INS_angle[1];   //������Pitchֵ
	Pitch_imu_speed = INS_gyro[1];   //�����ǽ��ٶ�ֵ
}


//================================================�ٶȻ�����================================================//
static void gimbal_control_speed()
{
		if( (rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684 ) || (remote.key.ctrl))
		{
			target_speed_can_2[4]=15;
			target_speed_can_2[5]=-15;
		}
		else if( (rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974) || (remote.key.shift))
		{
			target_speed_can_2[4]=-15;
			target_speed_can_2[5]=15;
		}
		else
		{
			target_speed_can_2[4]=0;
			target_speed_can_2[5]=0;
		}
}	

//================================================Pitch���ݷ���================================================//
static void gimbal_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (motor_info_can_2[4].set_voltage>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] = (motor_info_can_2[4].set_voltage)&0xff;
  tx_data[2] = (motor_info_can_2[5].set_voltage>>8)&0xff;
  tx_data[3] = (motor_info_can_2[5].set_voltage)&0xff;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


//================================================���ڱ������ķ�ת��λ(δʹ��)================================================//
static void gimbal_limit()
{
	gimbal_read_imu();
	if( gimbal_rotor_angle >= Pitch_up)
	{
		
		target_speed_can_2[4]=-20;
		target_speed_can_2[5]=20;
	}
	
	else if( gimbal_rotor_angle <= Pitch_down)
	{
		target_speed_can_2[4]=20;
		target_speed_can_2[5]=-20;
	}
}

//================================================Pitch�����λ================================================//
static void gimbal_imu_limit()
{
	if(target_pitch < Up_inf && Pitch_imu)
	{
		target_pitch = Up_inf;
	}
	else if(target_pitch >= Down_inf && Pitch_imu)
	{
		target_pitch = Down_inf;
	}
}

//================================================������(δʹ��)================================================//
static void gimbal_mouse()
{
	if(remote.mouse.y > mouse_y_valve || remote.mouse.y < -mouse_y_valve)
	{
		target_speed_can_2[4] += (fp32)remote.mouse.y * mouse_y_weight;
		target_speed_can_2[5] -= (fp32)remote.mouse.y * mouse_y_weight;
	}
}

//================================================�Ӿ���׼(�ٶȻ�ģʽ)================================================//
static void gimbal_minipc_control()
{
		target_speed_can_2[4] -= chase.pitch * Pitch_minipc_weight;
		target_speed_can_2[5] += chase.pitch * Pitch_minipc_weight;
}

//================================================Ѳ��ģʽ(�ٶȻ�ģʽ)================================================//
static void gimbal_mode_search_speed()
{
	switch(TIM1_Mode)
	{
		case 1: target_speed_can_2[4]=-4 , target_speed_can_2[5]=4;break;
		case 2: target_speed_can_2[4]=4 , target_speed_can_2[5]=-4;break;
	}
}

//================================================�ٶ�����================================================//
static void gimbal_zero()
{
	target_speed_can_2[4] = 0;
	target_speed_can_2[5] = 0;
}

//================================================ң����λ�û�����ģʽ(����������Ȩ��)================================================//
static void gimbal_mode_control_sita()
{
		if( (rc_ctrl.rc.ch[1]>=324 && rc_ctrl.rc.ch[1] <=1684 ) )
		{
			target_pitch -= (rc_ctrl.rc.ch[1]- 1024)/660.0 * Pitch_sita_weight; 			
		}
}

//================================================�Ӿ���׼(λ�û�ģʽ)================================================//
static void gimbal_minipc_control_sita()
{
	//target_pitch -= (chase.pitch - target_pitch) * Pitch_sita_minipc_weight;
	target_pitch =chase.pitch;
}

//================================================Ѳ��ģʽ(λ�û�ģʽ)================================================//
static void gimbal_mode_search_sita()
{
	switch(TIM1_Mode)
	{
		case 1: target_pitch -= 0.03f;break;
		case 2: target_pitch += 0.03f;break;
	}
}