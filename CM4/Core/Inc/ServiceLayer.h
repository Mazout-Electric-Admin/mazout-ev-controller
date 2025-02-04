/*
 * ServiceLayer.h
 *
 *  Created on: Jan 31, 2025
 *      Author: pirda
 */

#ifndef APPLICATION_USER_CORE_INC_SERVICELAYER_H_
#define APPLICATION_USER_CORE_INC_SERVICELAYER_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"

#include "Packet.h"
#include "AT_command.h"


typedef enum {
    IMMOBILIZE_STATUS = 0x01,
    RPM_PRESET        = 0x02,
    GPS               = 0x03,
    BUS_CURRENT       = 0x04,
    BUS_VOLTAGE       = 0x05,
    RPM               = 0x06,
    DEVICE_TEMP       = 0x07,
    NETWORK_STRENGTH  = 0x08,
	TORQUE			  = 0x09,
	SOC				  = 0x0A,
	THROTTLE		  = 0x0B,
	MOTOR_TEMP 		  = 0x0C,
} ServerPropertyType;


typedef struct {
    uint8_t immobilizeStatus[1]; // 1 byte
    uint8_t rpmPreset[2];        // 1 byte
    uint8_t gpsData[32];          // 6 bytes
    uint8_t currentData[2];      // 2 bytes
    uint8_t voltageData[2];      // 2 bytes
    uint8_t rpm[2];              // 1 byte
    uint8_t device_temp[1];      // 1 byte
    uint8_t motor_temp[1];
    uint8_t torque[1];
    uint8_t soc[2];
    uint8_t throttle[2];
    uint8_t networkStrength[1];  // 1 byte
} serverProperties;

extern serverProperties serverAttributes;

//extern char txBuffer[RX_BUFFER_SIZE];        // Buffer for sending AT commands
//extern char rxBuffer[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
//extern char checkBuffer[RX_BUFFER_SIZE];

void HandleReceivedData(uint8_t writeIndexS);
uint8_t encodeServerData(ServerPropertyType type, uint8_t *packet);
void decodeServerData(uint8_t *packet, uint8_t length);

#endif /* APPLICATION_USER_CORE_INC_SERVICELAYER_H_ */
