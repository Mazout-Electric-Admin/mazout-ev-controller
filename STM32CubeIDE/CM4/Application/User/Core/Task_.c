/*
 * Task_.c
 *
 *  Created on: Jan 31, 2025
 *      Author: pirda
 */


#include <_Task_.h>
#include <string.h>
#include "cmsis_os.h"



/* USER CODE BEGIN Header_StartSendTask */
/**
* @brief Function implementing the sendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendTask */
void StartSendTask(void *argument)
{
  /* USER CODE BEGIN StartSendTask */
  /* Infinite loop */
  for(;;)
  {
	  SocketSendData();
	  osDelay(4000);
  }
  /* USER CODE END StartSendTask */
}


/* USER CODE BEGIN Header_StartReceiveTask */
/**
* @brief Function implementing the receiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveTask */
void StartReceiveTask(void *argument)
{
  /* USER CODE BEGIN StartReceiveTask */
  /* Infinite loop */
  for(;;)
  {
	  SocketReceiveData();
	  //osDelay(100);
  }
  /* USER CODE END StartReceiveTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */
  /* Infinite loop */
  for(;;)
  {
	gps();
    osDelay(8000);
  }
  /* USER CODE END StartGpsTask */
}
