/*
 * ServiceLayer.c
 *
 *  Created on: Jan 31, 2025
 *      Author: pirda
 */



#include "ServiceLayer.h"
#include "Network.h"



uint16_t writeIndex = 0;  // Updated by DMA
uint16_t readIndex = 0;   // Updated by application

extern char txBuffer[RX_BUFFER_SIZE];        // Buffer for sending AT commands
extern char rxBuffer[RX_BUFFER_SIZE];        // Buffer for receiving AT responses
extern char checkBuffer[RX_BUFFER_SIZE];

serverProperties serverAttributes;

extern UART_HandleTypeDef huart1;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    //if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
      //  __HAL_UART_CLEAR_IDLEFLAG(&huart1);  // Clear the idle flag

	// Process received data
	writeIndex = Size;//(RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx));// % RX_BUFFER_SIZE;
	HandleReceivedData(writeIndex);
}

void HandleReceivedData(uint8_t writeIndex) {

	uint16_t newDataCount = (writeIndex >= readIndex)
	                            ? (writeIndex - readIndex)
	                            : (RX_BUFFER_SIZE - readIndex + writeIndex);
	memset(checkBuffer, '\0', RX_BUFFER_SIZE);
	for (uint16_t i = 0; i < newDataCount; i++) {
		// Copy new data to the process buffer
		uint8_t newByte = rxBuffer[readIndex];
		checkBuffer[i] = newByte;

		// Increment read index circularly
		readIndex = (readIndex + 1) % RX_BUFFER_SIZE;
	}
	// Check if we have a complete packet
	if (readIndex >= PACKET_MIN_LENGTH) { // Assume minimum length is 2 bytes (Type + Length)
		for(int i = 0; i < 20; i++){
			if (checkBuffer[i] == 0xAA) {
				decodeServerData((uint8_t *)&checkBuffer[i], readIndex);
				break;
			}
		}
	}
}


uint8_t encodeServerData(ServerPropertyType type, uint8_t *packet) {
    uint8_t payloadLength = 0;
    uint8_t *payload;

    switch (type) {
        case IMMOBILIZE_STATUS:
            //payload = serverAttributes.immobilizeStatus;
            //payloadLength = sizeof(serverAttributes.immobilizeStatus);
            break;
        case RPM_PRESET:
            //payload = serverAttributes.rpmPreset;
           // payloadLength = sizeof(serverAttributes.rpmPreset);
            break;
        case GPS:
            payload = serverAttributes.gpsData;
            payloadLength = sizeof(serverAttributes.gpsData);
            break;
        case BUS_CURRENT:
            payload = serverAttributes.currentData;
            payloadLength = sizeof(serverAttributes.currentData);
            break;
        case BUS_VOLTAGE:
            payload = serverAttributes.voltageData;
            payloadLength = sizeof(serverAttributes.voltageData);
            break;
        case RPM:
            payload = serverAttributes.rpm;
            payloadLength = sizeof(serverAttributes.rpm);
            break;
        case DEVICE_TEMP:
            payload = serverAttributes.device_temp;
            payloadLength = sizeof(serverAttributes.device_temp);
            break;
        case NETWORK_STRENGTH:
            payload = serverAttributes.networkStrength;
            payloadLength = sizeof(serverAttributes.networkStrength);
            break;
        case TORQUE:
			payload = serverAttributes.torque;
			payloadLength = sizeof(serverAttributes.torque);
			break;
        case SOC:
			payload = serverAttributes.soc;
			payloadLength = sizeof(serverAttributes.soc);
			break;
        case MOTOR_TEMP:
			payload = serverAttributes.motor_temp;
			payloadLength = sizeof(serverAttributes.motor_temp);
			break;
        case THROTTLE:
			payload = serverAttributes.throttle;
			payloadLength = sizeof(serverAttributes.throttle);
			break;
        default:
            return 0; // Unknown type
    }

    // Create the packet
    uint8_t index = 0;
    packet[index++] = 0xAA;  // Header byte 1
    packet[index++] = 0xBB;  // Header byte 2
    packet[index++] = type;  // Property type
    packet[index++] = payloadLength; // Payload length

    // Copy payload
    memcpy(&packet[index], payload, payloadLength);
    index += payloadLength;

    // Add checksum
    uint8_t checksum = 0;
    for (uint8_t i = 2; i < index; i++) {
        checksum ^= packet[i];
    }
    packet[index++] = checksum;
    if (packet[index-1] == 0) packet[index-1] = 0xDD;
    packet[index++] = 0xCC;

    return index; // Total packet length
}

void decodeServerData(uint8_t *packet, uint8_t length) {
    if (length < 5) return; // Invalid packet length

    // Validate header
    if (packet[0] != 0xAA || packet[1] != 0xBB) return;

    // Extract type and payload length
    ServerPropertyType type = packet[2];
    uint8_t payloadLength = packet[3];

    // Validate checksum
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 4 + payloadLength; i++) {
        checksum ^= packet[i];
    }
    if (checksum != packet[4 + payloadLength]) return;

    // Extract payload
    uint8_t *payload = &packet[4];

    // Update serverAttributes
    switch (type) {
        case IMMOBILIZE_STATUS:
            memcpy(serverAttributes.immobilizeStatus, payload, payloadLength);
            break;
        case RPM_PRESET:
            memcpy(serverAttributes.rpmPreset, payload, payloadLength);
            break;
        case GPS:
            //memcpy(serverAttributes.gpsData, payload, payloadLength);
            break;
        case BUS_CURRENT:
            memcpy(serverAttributes.currentData, payload, payloadLength);
            break;
        case BUS_VOLTAGE:
            memcpy(serverAttributes.voltageData, payload, payloadLength);
            break;
        case RPM:
            memcpy(serverAttributes.rpm, payload, payloadLength);
            break;
        case DEVICE_TEMP:
            memcpy(serverAttributes.device_temp, payload, payloadLength);
            break;
        case MOTOR_TEMP:
			memcpy(serverAttributes.motor_temp, payload, payloadLength);
			break;
        case NETWORK_STRENGTH:
            memcpy(serverAttributes.networkStrength, payload, payloadLength);
            break;
        case TORQUE:
			memcpy(serverAttributes.torque, payload, payloadLength);
			break;
        case SOC:
			memcpy(serverAttributes.soc, payload, payloadLength);
			break;
        case THROTTLE:
			memcpy(serverAttributes.throttle, payload, payloadLength);
			break;
        default:
            // Unknown type
            break;
    }
    return;
}
