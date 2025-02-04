/*
 * Network.h
 *
 *  Created on: Jan 31, 2025
 *      Author: pirda
 */

#ifndef APPLICATION_USER_CORE_INC_NETWORK_H_
#define APPLICATION_USER_CORE_INC_NETWORK_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"

#include "AT_command.h"
#include "Packet.h"
#include "ServiceLayer.h"

extern char txBuffer[RX_BUFFER_SIZE];        // Buffer for sending AT commands
extern char rxBuffer[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
extern char checkBuffer[RX_BUFFER_SIZE];

//serverProperties serverAttributesN;

void NetworkInit(void);
void OpenSocket(void);
void device_config(int type);
void SocketSendData(void);
void SocketReceiveData(void);

#endif /* APPLICATION_USER_CORE_INC_NETWORK_H_ */
