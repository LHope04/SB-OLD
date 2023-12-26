#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "MX_FREERTOS_Init.h"
#include "Yaw_task.h"
#include "StartDefaultTask.h"
#include "Pitch_task.h"
#include "Exchange_task.h"
#include "Friction_task.h"
#include "INS_task.h"

//================================================���񴴽�================================================//
osThreadId insTaskHandle;
osThreadId yawTaskHandle;
osThreadId defaultTaskHandle;
osThreadId pitchtaskHandle;
osThreadId exchangeHandle;
osThreadId frictionHandle;

void MX_FREERTOS_Init(void) {
  
	//��������
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	
	//YAW��������
	osThreadDef(yawtask, Yaw_task, osPriorityRealtime, 0, 256);		
  yawTaskHandle = osThreadCreate(osThread(yawtask), NULL);
	
	//Pitch��������
	osThreadDef(pitchtask, Pitch_task, osPriorityRealtime, 0, 256);
  pitchtaskHandle = osThreadCreate(osThread(pitchtask), NULL);
	
	//����C��ͨ������
	osThreadDef(exchangetask, Exchange_task,  osPriorityHigh, 0, 512);
  exchangeHandle = osThreadCreate(osThread(exchangetask), NULL);

	//Ħ���ֺͲ��̿�������
  osThreadDef(frictiontask, Friction_task, osPriorityIdle, 0, 128);
  frictionHandle = osThreadCreate(osThread(frictiontask), NULL);
	
	//����IMU����
	osThreadDef(imutask, INS_Task,  osPriorityRealtime, 0, 512);
  insTaskHandle = osThreadCreate(osThread(imutask), NULL);
	
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}