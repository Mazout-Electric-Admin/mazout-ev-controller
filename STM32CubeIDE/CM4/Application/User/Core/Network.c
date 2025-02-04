/*
 * Network.c
 *
 *  Created on: Jan 31, 2025
 *      Author: pirda
 */


#include "Network.h"

#include "cmsis_os.h"

extern osMutexId uart_lockHandle;

extern UART_HandleTypeDef huart1;

extern serverProperties serverAttributes;

char txBuffer[RX_BUFFER_SIZE];        // Buffer for sending AT commands
char rxBuffer[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
char checkBuffer[RX_BUFFER_SIZE];

uint8_t propertyIndex = 3;
uint8_t c_type = 0;


void NetworkInit() {

	uint16_t rxLength = 0;
	//while(rxBuffer[0] == ){
	strcpy(txBuffer, CMD_ECHO_OFF);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));
	//}
	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);

    // Check SIM is ready
    strcpy(txBuffer, CMD_CHECK_SIM);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, NULL_ , sizeof(txBuffer));

    memset(rxBuffer, NULL_ , sizeof(rxBuffer));
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
    //HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);

    //osDelay(100);

    // Check network registration
    strcpy(txBuffer, CMD_NETWORK_REG);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, NULL_ , sizeof(txBuffer));

    memset(rxBuffer, NULL_ , sizeof(rxBuffer));
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
    //HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
    //osDelay(100);

    // Check network registration
	strcpy(txBuffer, CMD_NETWORK_PDP_CFG);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

    // Set APN
    strcpy(txBuffer, CMD_SET_APN);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, NULL_ , sizeof(txBuffer));

    memset(rxBuffer, NULL_ , sizeof(rxBuffer));
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

    // PDP Contextn
    strcpy(txBuffer, CMD_ACTIVATE_PDP);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);


	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_PORT);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_GNSS_CFG);//3
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_GPS_A);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_GPS);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_GLONASS);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_GALILEO);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_BID_T);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_BID_F);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Set GPS Mode
	strcpy(txBuffer, CMD_GPS_CFG_GNSS);//2
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	memset(rxBuffer, NULL_ , sizeof(rxBuffer));
	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

	// Start GPS session
	strcpy(txBuffer, CMD_GPS_MODE_ON);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_, sizeof(txBuffer));

    // Start TCP/IP service
    strcpy(txBuffer, CMD_NETWORK_ACTIVATE);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, NULL_ , sizeof(txBuffer));

    memset(rxBuffer, NULL_ , sizeof(rxBuffer));
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	HAL_UART_Abort(&huart1);
	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	//osDelay(100);

    // Check response
    if (strstr(rxBuffer, RESPONSE_NETOPEN_OK) == NULL) {
        //Error_Handler();
    }

    return;
}

void OpenSocket() {
	uint16_t rxLength = 0;

    sprintf(txBuffer, CMD_OPEN_SOCKET_FORMAT, SOCKET_INDEX, SERVER_IP, SERVER_PORT);
    HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    memset(txBuffer, NULL_ , sizeof(txBuffer));

    memset(rxBuffer, NULL_ , sizeof(rxBuffer));
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
   	HAL_UART_Abort(&huart1);
   	HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), &rxLength, UART_TIMEOUT);
   	//HAL_UART_Receive(&huart1, (uint8_t *)rxBuffer, sizeof(rxBuffer), UART_TIMEOUT);
   	//HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxBuffer, 256);
	memset(rxBuffer, NULL_ , sizeof(rxBuffer));


    // Check response
    if (strstr(rxBuffer, RESPONSE_SOCKET_OPEN_OK) == NULL) {
        //Error_Handler();
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rxBuffer, RX_BUFFER_SIZE);   /////////Need to change as this will go on
    																		// to all the other files also
    return;
}

void device_config(int type)
{
	if(type == 1)
	{
	  serverAttributes.currentData[0] = 0xDD;
	  serverAttributes.currentData[1] = 0x4D;
	  serverAttributes.voltageData[0] = 0x63;
	  serverAttributes.voltageData[1] = 0x63;
	  serverAttributes.rpm[0] = 0x01;
	  serverAttributes.rpm[1] = 0xF3;
	  serverAttributes.motor_temp[0] = 0x20;
	  serverAttributes.device_temp[0] = 0x20;
	  serverAttributes.networkStrength[0] = 0x10;
	  serverAttributes.soc[0] = 0x63;
	  serverAttributes.soc[1] = 0x63;
	  serverAttributes.torque[0] = 0x04;
	  serverAttributes.throttle[0] = 0x01;
	  serverAttributes.throttle[1] = 0x02;
	}
	else if(type == 2)
	{
	  serverAttributes.currentData[0] = 0x01;
	  serverAttributes.currentData[1] = 0x16;
	  serverAttributes.voltageData[0] = 0x63;
	  serverAttributes.voltageData[1] = 0x63;
	  serverAttributes.rpm[0] = 0x05;
	  serverAttributes.rpm[1] = 0xDB;
	  serverAttributes.motor_temp[0] = 0x21;
	  serverAttributes.device_temp[0] = 0x21;
	  serverAttributes.networkStrength[0] = 0x16;
	  serverAttributes.soc[0] = 0x63;
	  serverAttributes.soc[1] = 0x63;
	  serverAttributes.torque[0] = 0x0A;
	  serverAttributes.throttle[0] = 0x01;
	  serverAttributes.throttle[1] = 0x02;
	}
	else if(type == 3)
	{
	  serverAttributes.currentData[0] = 0x01;
	  serverAttributes.currentData[1] = 0x55;
	  serverAttributes.voltageData[0] = 0x63;
	  serverAttributes.voltageData[1] = 0x63;
	  serverAttributes.rpm[0] = 0x09;
	  serverAttributes.rpm[1] = 0xC3;
	  serverAttributes.motor_temp[0] = 0x21;
	  serverAttributes.device_temp[0] = 0x21;
	  serverAttributes.networkStrength[0] = 0x16;
	  serverAttributes.soc[0] = 0x63;
	  serverAttributes.soc[1] = 0x63;
	  serverAttributes.torque[0] = 0x09;
	  serverAttributes.throttle[0] = 0x01;
	  serverAttributes.throttle[1] = 0x02;
	}
	return;
}

void SocketSendData(void) {
	uint8_t data[21];
	uint8_t data_len = 0;
	TickType_t xLastWakeTime;

	memset(data, NULL_, sizeof(data));

	data_len = encodeServerData(propertyIndex, data);

	//osMutexAcquire(uart_lockHandle, osWaitForever);
	if (osMutexAcquire(uart_lockHandle, 1000) != osOK) {  // Add timeout
	    //printf("Mutex acquisition failed! Possible deadlock.\n");
	    return;
	}

	sprintf(txBuffer, CMD_SEND_DATA_FORMAT, SOCKET_INDEX, data_len);

	/*
	// Wait for `>` prompt
	while (!strstr((char *)checkBuffer, SERVER_ACK)) {

		HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
		//osDelay(500);  // Wait for the response
		//TickType_t xLastWakeTime = osKernelGetTickCount();
		osDelayUntil(500);
	}
	*/
	 // Wait for `>` prompt
	uint32_t timeout = osKernelGetTickCount() + 1000;  // 5s timeout
	while (!strstr((char *)checkBuffer, SERVER_ACK)) {
		HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);

		if (osKernelGetTickCount() > timeout) {
			osMutexRelease(uart_lockHandle);
			return;  // Exit if server never responds
		}

		xLastWakeTime = osKernelGetTickCount();
		osDelayUntil(xLastWakeTime + 500);  // Fixed osDelayUntil usage
	}

	memset(txBuffer, NULL_ , sizeof(txBuffer));

	// Send data
	//memset(txBuffer, (uint8_t *)data, sizeof(data));
	memcpy(txBuffer, (uint8_t *)data, sizeof(data));

	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	memset(txBuffer, NULL_ , sizeof(txBuffer));

	osMutexRelease(uart_lockHandle);

	osDelay(10);
	//TickType_t xLastWakeTime = osKernelGetTickCount();
	//osDelayUntil(10);

	return;
}

void SocketReceiveData(void) {
	int length = sizeof(rxBuffer);
	uint8_t ctr = 0;

	//osMutexAcquire(uart_lockHandle, osWaitForever);
	if (osMutexAcquire(uart_lockHandle, 1000) != osOK) {  // Add timeout
		//printf("Mutex acquisition failed! Possible deadlock.\n");
		return;
	}

    sprintf(txBuffer, CMD_RECEIVE_DATA_FORMAT, SOCKET_INDEX, length);
    //while(++ctr < 10) {
    	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    	//osDelay(50);
    //}
    memset(txBuffer, NULL_ , sizeof(txBuffer));

    osMutexRelease(uart_lockHandle);

    osDelay(1);

    //return;

}

