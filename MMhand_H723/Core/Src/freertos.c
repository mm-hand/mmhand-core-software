/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CanCommTask.h"
#include "ServoCommTask.h"
#include "UpperCommTask.h"
#include "UsbCommTask.h"
#include "queue.h"

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for canCommunicationTask */
osThreadId_t canCommTaskHandle;
const osThreadAttr_t canCommTask_attributes = {
  .name = "canCommunicationTask",
  /* CMSIS-RTOS stack_size is in bytes. */
  .stack_size = CAN_COMM_TASK_STACK_SIZE,
  .priority = (osPriority_t)(osPriorityNormal + TASK_CAN_COMM_PRIORITY),
};

/* Definitions for usbCommunicationTask */
osThreadId_t usbCommTaskHandle;
const osThreadAttr_t usbCommTask_attributes = {
  .name = "usbCommunicationTask",
  /* CMSIS-RTOS stack_size is in bytes. */
  .stack_size = USB_COMM_TASK_STACK_SIZE,
  .priority = (osPriority_t)(osPriorityNormal + TASK_USB_COMM_PRIORITY),
};

/* Definitions for servoCommunicationTask */
osThreadId_t servoCommTaskHandle;
const osThreadAttr_t servoCommTask_attributes = {
  .name = "servoCommunicationTask",
  /* CMSIS-RTOS stack_size is in bytes. */
  .stack_size = SERVO_COMM_TASK_STACK_SIZE,
  .priority = (osPriority_t)(osPriorityNormal + TASK_SERVO_COMM_PRIORITY),
};

/* Definitions for upperCommunicationTask */
osThreadId_t upperCommTaskHandle;
const osThreadAttr_t upperCommTask_attributes = {
  .name = "upperCommunicationTask",
  /* CMSIS-RTOS stack_size is in bytes. */
  .stack_size = UPPER_COMM_TASK_STACK_SIZE,
  .priority = (osPriority_t)(osPriorityNormal + TASK_UPPER_COMM_PRIORITY),
};

/* USER CODE BEGIN Variables */
TaskSharedData_t sharedData;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void *argument);
void canCommunicationTask(void *parameter);
void servoCommunicationTask(void *parameter);
void usbCommunicationTask(void *parameter);
void upperCommunicationTask(void *parameter);

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/**
  * @brief  FreeRTOS initialization.
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  memset(&sharedData, 0, sizeof(sharedData));
  sharedData.system_state = SYSTEM_STATE_READY;
  sharedData.control_mode = CONTROL_MODE_JOINT;
  sharedData.servo_target_owner = SERVO_TARGET_OWNER_NONE;
  sharedData.active_motor_mask = validMotorChannelMask();
  sharedData.active_joint_mask = JOINT_CHANNEL_MASK_ALL;

  sharedData.targetAnglesMutex = xSemaphoreCreateMutex();
  sharedData.commandStateMutex = xSemaphoreCreateMutex();

  sharedData.stateEventQueue = xQueueCreate(5, sizeof(SystemEvent_t));
  sharedData.canRxQueue = xQueueCreate(1, sizeof(RemoteSensorData_t));
  sharedData.canTxQueue = xQueueCreate(5, sizeof(RemoteCommand_t));
  sharedData.mappedAngleQueue = xQueueCreate(1, sizeof(MappedAngleData_t));
  sharedData.jointDebugQueue = xQueueCreate(8, sizeof(JointDebugData_t));
  sharedData.servoTargetQueue = xQueueCreate(1, sizeof(ServoTargetBatch_t));
  sharedData.servoFeedbackQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
  sharedData.servoAngleQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
  sharedData.servoRawQueue = xQueueCreate(1, sizeof(ServoAngleData_t));
  sharedData.servoTelemetryQueue = xQueueCreate(1, sizeof(ServoTelemetryData_t));
  sharedData.servoTelemetrySnapshotQueue = xQueueCreate(1, sizeof(ServoTelemetryData_t));
  sharedData.tactileQueue = xQueueCreate(1, sizeof(RemoteTactileData_t));
  sharedData.usbRxQueue = xQueueCreate(USB_COMM_RX_QUEUE_LENGTH, sizeof(UsbPacket_t));
  sharedData.usbTxQueue = xQueueCreate(USB_COMM_TX_QUEUE_LENGTH, sizeof(UsbPacket_t));

  if ((sharedData.targetAnglesMutex == NULL) ||
      (sharedData.commandStateMutex == NULL) ||
      (sharedData.stateEventQueue == NULL) ||
      (sharedData.canRxQueue == NULL) ||
      (sharedData.canTxQueue == NULL) ||
      (sharedData.mappedAngleQueue == NULL) ||
      (sharedData.jointDebugQueue == NULL) ||
      (sharedData.servoTargetQueue == NULL) ||
      (sharedData.servoFeedbackQueue == NULL) ||
      (sharedData.servoAngleQueue == NULL) ||
      (sharedData.servoRawQueue == NULL) ||
      (sharedData.servoTelemetryQueue == NULL) ||
      (sharedData.servoTelemetrySnapshotQueue == NULL) ||
      (sharedData.tactileQueue == NULL) ||
      (sharedData.usbRxQueue == NULL) ||
      (sharedData.usbTxQueue == NULL))
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  if (defaultTaskHandle == NULL)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN RTOS_THREADS */
  canCommTaskHandle = osThreadNew(canCommunicationTask, &sharedData, &canCommTask_attributes);
  if (canCommTaskHandle == NULL)
  {
    Error_Handler();
  }

  usbCommTaskHandle = osThreadNew(usbCommunicationTask, &sharedData, &usbCommTask_attributes);
  if (usbCommTaskHandle == NULL)
  {
    Error_Handler();
  }

  servoCommTaskHandle = osThreadNew(servoCommunicationTask, &sharedData, &servoCommTask_attributes);
  if (servoCommTaskHandle == NULL)
  {
    Error_Handler();
  }

  upperCommTaskHandle = osThreadNew(upperCommunicationTask, &sharedData, &upperCommTask_attributes);
  if (upperCommTaskHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_canCommunicationTask */
/**
  * @brief  Function implementing the canCommunicationTask thread.
  * @param  parameter: TaskSharedData_t pointer.
  * @retval None
  */
/* USER CODE END Header_canCommunicationTask */
void canCommunicationTask(void *parameter)
{
  /* USER CODE BEGIN canCommunicationTask */
  CanCommTask_Run(parameter);
  /* USER CODE END canCommunicationTask */
}

/* USER CODE BEGIN Header_servoCommunicationTask */
/**
  * @brief  Function implementing the servoCommunicationTask thread.
  * @param  parameter: TaskSharedData_t pointer.
  * @retval None
  */
/* USER CODE END Header_servoCommunicationTask */
void servoCommunicationTask(void *parameter)
{
  /* USER CODE BEGIN servoCommunicationTask */
  ServoCommTask_Run(parameter);
  /* USER CODE END servoCommunicationTask */
}

/* USER CODE BEGIN Header_usbCommunicationTask */
/**
  * @brief  Function implementing the usbCommunicationTask thread.
  * @param  parameter: TaskSharedData_t pointer.
  * @retval None
  */
/* USER CODE END Header_usbCommunicationTask */
void usbCommunicationTask(void *parameter)
{
  /* USER CODE BEGIN usbCommunicationTask */
  UsbCommTask_Run(parameter);
  /* USER CODE END usbCommunicationTask */
}

/* USER CODE BEGIN Header_upperCommunicationTask */
/**
  * @brief  Function implementing the upperCommunicationTask thread.
  * @param  parameter: TaskSharedData_t pointer.
  * @retval None
  */
/* USER CODE END Header_upperCommunicationTask */
void upperCommunicationTask(void *parameter)
{
  /* USER CODE BEGIN upperCommunicationTask */
  UpperCommTask_Run(parameter);
  /* USER CODE END upperCommunicationTask */
}
