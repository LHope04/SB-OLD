#include "Yaw_task.h"

//================================================YAW������������================================================//

//	˼·�������ٶȻ�������PID��������̨������̨�����ǵ�yawֵ�������ǶȻ����в���
//  ע�⣺��̨����ת�Ǹ�������˳ʱ���������ֵ����
//  ��һ�汾�����ң�������Ƶİ汾
//  YAW����6020�����������C��CAN_1�����IDΪ6�����߽ӵ�����can1�ֵ��ģ�

//================================================ȫ�ֱ���================================================//
int yaw_fix_flag = 1;		//yaw������ʶ����Ϊ1ʱ��������Ϊ0ʱ����yaw
int8_t Update_yaw_flag;		//����Ÿ���yaw���־λ(δʹ��)

fp32 ins_yaw;		//Imuʵ�ʽǶ�ֵ
fp32 target_yaw;	//Ŀ��Ƕ�ֵ
fp32 err_yaw;		//�ǶȲ�ֵ
fp32 angle_weight = 1;	//�ǶȻ�->�ٶȻ���ӳ���Ȩ��

//ǰ�����Ʊ���
int16_t Rotate_w;
int16_t Rotate_W;

//================================================����================================================//

//��ʼ��PID����
static void Yaw_init();	

//ÿ��ѭ����ʼ��
static void Yaw_loop_init();

//��ȡimu����
static void Yaw_read_imu();

//����������̨(���У��ٶȻ�)
static void Yaw_fix();

//����������̨(���У�λ�û�)
static void Yaw_fix_sita();

//Mode_1�µĿ����㷨,ֱ�Ӷ���Ѳ���ƶ����ٶȻ����ƣ���������
static void Yaw_mode_search();

//Mode_2�µĿ����㷨���ٶȻ����ƣ���������
static void Yaw_mode_remote_speed();

//Mode_3�µĿ����㷨��λ�û����ƣ������ǣ�
static void Yaw_mode_remote_site();

//������YawԽ�紦��
static void detel_calc();

//���ٶȲ���ǰ������
static void Yaw_Rotate();

//�����Ƶ���
static void Yaw_mouse();

//PID����ͷ���
static void Yaw_can_send();

//�����Ӿ�����(�ٶȻ�)
static void Yaw_minipc_control();

//�����Ӿ�����(λ�û�)
static void Yaw_minipc_control_sita();


//================================================YAW�����������================================================//
void Yaw_task(void const *pvParameters)
{
  //������ʼ������
	Yaw_init();
	osDelay(3000);
	
	
	//ѭ����������
  for(;;)
  {
		Yaw_loop_init();//ѭ����ʼ��
		Yaw_read_imu();//��ȡImu�Ƕ�

		//�ϳ�ģʽ�����Ͻǿ��ؿ������·�
		if(rc_ctrl.rc.s[1] == 2 && ins_yaw)
		{
			if(Sentry.foe_flag)	//����Ӿ���⵽Ŀ��
			{
					Yaw_minipc_control_sita();	//�Ӿ�����
					Yaw_Rotate();		//ǰ�����Ʋ������̴�������ת���ٶ�
					Yaw_fix_sita();		//�Ƕȿ���
			}			
			else//û��⵽��Ѳ��ģʽ
			{					
					Yaw_Rotate();			//ǰ�����Ʋ������̴�������ת���ٶ�	
					Yaw_mode_search();			//�ڱ�Ѳ��ģʽ
					yaw_fix_flag = 1;		//���費�����ı�־λ
			}
		}
		else if(rc_ctrl.rc.s[1] == 1 || rc_ctrl.rc.s[1] == 3)	//����ģʽ
		{
			Yaw_minipc_control_sita();	//�Ӿ�����
			Yaw_Rotate();		//ǰ�����Ʋ������̴�������ת���ٶ�
			Yaw_mode_remote_site();		//ң��������ģʽ(λ�ÿ���)
			yaw_fix_flag = 1;		//���費�����ı�־λ
		}
		detel_calc();	//Խ�紦��
		target_speed[6] +=  pid_calc_sita(&motor_pid_sita[6], target_yaw, INS_angle[0]);//�Ƕ�->�ٶȣ��ں�Խ�紦��
		motor_info[6].set_voltage = pid_calc(&motor_pid[6], target_speed[6], 9.55f * INS_gyro[2]);//�������ǵĽ��ٶȣ�rad/s -> r/min�����ٶ�->����
		Yaw_can_send();//����������ݷ���
    osDelay(1);
  }

}


//��ʼ��PID����
static void Yaw_init()	
{
	//idΪcan1��5��
	pid_init(&motor_pid[6],300,0.001,0,30000,30000);
	pid_init(&motor_pid_sita[6],18,0,30,30000,30000);
	target_yaw = ins_yaw;
}


//================================================ѭ����ʼ��================================================//
static void Yaw_loop_init()
{
	target_speed[6]=0;
}


//================================================��ȡimu�Ƕ�================================================//
static void Yaw_read_imu()
{
		//�����Ƕ�ֵ��ȡ
		ins_yaw = INS_angle[0];
		//ins_pitch = INS_angle[1];
		//ins_row = INS_angle[2];
}

//================================================Yaw�����������================================================//
static void Yaw_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = 0x00;	//�ȷ��߰�λ		
  tx_data[1] = 0x00;
  tx_data[2] = (motor_info[6].set_voltage>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[3] = (motor_info[6].set_voltage)&0xff;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1);

}

//================================================YAWǰ���������ٶ�================================================//
//����㷨����д��һ�����ֽ����о����ܻ��е����⣬����Ҫ�ɷ���������ǰ������Ӱ�첻��
//����ֱ�ӵ��ӵ��ٶȻ���
static void Yaw_Rotate()
{
	Rotate_W = (Rotate_gain * Rotate_w * Chassis_r) / Chassis_R;
	target_speed[6] += Rotate_W;
}

//================================================��������yaw��(δʹ�ã����ο�)================================================//
static void Yaw_fix()
{
	//ҡ�˻�������̨(һ�������)
					if(yaw_fix_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						target_yaw = ins_yaw;
						yaw_fix_flag = 0;
					}
						err_yaw = ins_yaw - target_yaw;		//��ʵʱ���ݼ���ʼ����
					
					//Խ�紦��,��֤ת�����򲻱�
					if(err_yaw < -180)	//	Խ��ʱ��180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	Խ��ʱ��-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//��ֵ�ж�
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_speed[6] -= err_yaw * angle_weight;
					}
					
					else
					{
						target_speed[6] = 0;
					}
				
}

//================================================�Ƕ�����================================================//
static void Yaw_fix_sita()
{
		if(yaw_fix_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ��ѭ��ִ�����ģʽ��ʱ��ֻ��ִ��һ��
		{
			target_yaw = ins_yaw;
			yaw_fix_flag = 0;
		}
}

//================================================������================================================//
static void Yaw_mouse()
{
	if(remote.mouse.x > mouse_x_valve || remote.mouse.x < -mouse_x_valve)
	{
		yaw_fix_flag = 1;
		target_speed[6] -= (fp32)remote.mouse.x * mouse_x_weight;
	}
}

//================================================����Ѳ������ģʽ================================================//
static void Yaw_mode_search()
{
	target_yaw -= 0.05f;
}

//================================================�ٶȿ���ģʽ================================================//
static void Yaw_mode_remote_speed()
{
	
		if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve && (!remote.key.q) && (!remote.key.e) && (remote.mouse.x < mouse_x_valve) && (remote.mouse.x > -mouse_x_valve) && (!remote.mouse.press_right))
	{
		Yaw_fix();
	}
	else if( (rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max) || (remote.key.e) )
	{
		target_speed[6] -= 60;
		yaw_fix_flag = 1;
	}
	
	else if( (rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve ) || (remote.key.q) )
	{
		target_speed[6] += 60;
		yaw_fix_flag = 1;
	}
}

//================================================�ٶ��Ӿ�����================================================//
static void Yaw_minipc_control()
{
		target_speed[6] -= chase.yaw * Yaw_minipc_weight;
}

//================================================λ�ÿ���ģʽ================================================//
static void Yaw_mode_remote_site()
{
		if(rc_ctrl.rc.ch[0] >= 324 && rc_ctrl.rc.ch[0]<= 1684)
		{			
			target_yaw -= (rc_ctrl.rc.ch[0]-base)/660.0 * Yaw_sita_weight; 			
		}
}

//================================================Խ�紦��================================================//
static void detel_calc()	//������д���Ľ��Ӧ����һ���ģ����PID���Խ�紦��һ��
{
	if(target_yaw >360)
	{
		target_yaw -=360;
	}
	
	else if(target_yaw<0)
	{
		target_yaw += 360;
	}
//	if(target_yaw >180)
//	{
//		target_yaw -=360;
//	}
//	
//	else if(target_yaw<-180)
//	{
//		target_yaw += 360;
//	}
}

//================================================λ���Ӿ�����================================================//
//ע��ʹ�ø��������ܱ������ȣ�������Ӿ��ԽӺ�
//ֱ�ӵ��ӽǶȻ�����
static void Yaw_minipc_control_sita()
{
	//	target_yaw -= chase.yaw * Yaw_minipc_sita_weight;
	target_yaw = chase.yaw;
}