/*
 * GPS.c
 *
 *  Created on: Jan 31, 2025
 *      Author: pirda
 */


#include "GPS.h"
#include "Network.h"
#include "ServiceLayer.h"

#include "cmsis_os.h"

extern osMutexId uart_lockHandle;

uint8_t gps_start = 0;

extern serverProperties serverAttributes;
extern uint8_t propertyIndex;
extern uint8_t c_type;

extern char txBuffer[RX_BUFFER_SIZE];        // Buffer for sending AT commands
extern char rxBuffer[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
extern char checkBuffer[RX_BUFFER_SIZE];

extern UART_HandleTypeDef huart1;

void gps(void) {
	//SocketSendData();
	char *gpsString;
	char extractedData[32] = {0};
	uint16_t ctr = 0;
	uint8_t set = 0;

    //osMutexAcquire(uart_lockHandle, osWaitForever);
    if (osMutexAcquire(uart_lockHandle, 1000) != osOK) {  // Add timeout
		//printf("Mutex acquisition failed! Possible deadlock.\n");
		return;
	}

    device_config(c_type);

    memset(txBuffer, NULL_, sizeof(txBuffer));
	strcpy(txBuffer, CMD_GPS_CHECK);
	//osDelay(2000);
	HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
	osDelay(100);
	gpsString = strstr((char *)checkBuffer, CMD_GPS_CHECK_R_T);
	if (gpsString != NULL){
		set = 1;
	}

	while (1) {
		if (set == 0)
		{
			memset(txBuffer, NULL_, sizeof(txBuffer));
			strcpy(txBuffer, CMD_GPS_CHECK);
			HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
			osDelay(100);
			gpsString = strstr((char *)checkBuffer, CMD_GPS_CHECK_R_T);
			if (gpsString != NULL){
				set = 1;
				//osDelay(6000);
				memset(txBuffer, NULL_, sizeof(txBuffer));
				strcpy(txBuffer, CMD_GPS_DATA);
				HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
				osDelay(100);
				gpsString = strstr((char *)rxBuffer, GPS_ACK);
				if (gpsString != NULL){
				}
			}
			osDelay(100);
			gpsString = strstr((char *)checkBuffer, ERROR_GPS_ACTIVE);
			if (gpsString != NULL){
				set = 1;
			}
		}
		else if (set == 1) {

			memset(txBuffer, NULL_, sizeof(txBuffer));
			strcpy(txBuffer, CMD_GPS_DATA);
			HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
			osDelay(100);
			gpsString = strstr((char *)rxBuffer, GPS_ACK);
			if (gpsString != NULL){
				break;
			}

		}
		if (ctr == 100 && gps_start == 0) {
			goto cleanup;
		}
		else if (ctr == 2 && gps_start == 1){
			goto cleanup;
		}
		osDelay(100);
		gpsString = strstr((char *)checkBuffer, ERROR_GPS_FIX);
		if (gpsString != NULL){
			memset(txBuffer, NULL_, sizeof(txBuffer));
			strcpy(txBuffer, CMD_GPS_CHECK);
			HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
			osDelay(100);
			gpsString = strstr((char *)checkBuffer, CMD_GPS_CHECK_R_T);
			if (gpsString != NULL && gps_start == 0){
				++ctr;
				osDelay(6000);
			}
			else if (gpsString != NULL && gps_start == 1){
				++ctr;
			}
			gpsString = strstr((char *)checkBuffer, CMD_GPS_CHECK_R_F);
			if (gpsString != NULL)	goto cleanup;
		}
	}

    memset(txBuffer, NULL_, sizeof(txBuffer));

    // Check for "GNRMC" in the response
    gpsString = strstr(gpsString, ".");//CHECK_DOT);
    if (gpsString == NULL) {
        goto cleanup;
    }

    // Check for the first comma
    gpsString = strstr(gpsString, ",");//CHECK_COMMA);
    if (gpsString == NULL) {
        goto cleanup;
    }

    // Extract data
    strncpy(extractedData, gpsString, 18);
    extractedData[19] = NULL_; // Ensure null termination

    // Remove the first comma
    char *comma = strchr(extractedData, ',');
    if (comma != NULL) {
        memmove(comma, comma + 1, strlen(comma)); // Shift the characters left
    }

    // Copy data to server attributes
    memcpy(serverAttributes.gpsData, extractedData, strlen(extractedData));

    // End GPS session
    //strcpy(txBuffer, "AT+QGPSEND\r\n");
    //HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    //memset(txBuffer, NULL_, sizeof(txBuffer));

    // Release mutex and exit
    osMutexRelease(uart_lockHandle);
    ctr = 0;
    if(++propertyIndex > 11){
  		propertyIndex  = 3;
  	}
    gps_start = 1;
    return;

cleanup:
    // End GPS session in case of invalid data
    //strcpy(txBuffer, "AT+QGPSEND\r\n");
    //HAL_UART_Transmit(&huart1, (uint8_t *)txBuffer, strlen(txBuffer), UART_TIMEOUT);
    //memset(txBuffer, NULL_, sizeof(txBuffer));

    // Release mutex before exiting
    osMutexRelease(uart_lockHandle);
    ctr = 0;
    if(++propertyIndex > 11){
  		propertyIndex  = 3;
  	}
    return;
}

