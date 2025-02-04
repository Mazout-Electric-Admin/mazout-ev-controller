/*
 * Commutation.h
 *
 *  Created on: Jan 31, 2025
 *      Author: pirda
 */

#ifndef APPLICATION_USER_CORE_INC_COMMUTATION_H_
#define APPLICATION_USER_CORE_INC_COMMUTATION_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32h7xx_hal.h"
//#include "stm32h7xx_nucleo.h"
//#include "stm32h7xx_hal.h"

#include "ServiceLayer.h"
#include "Network.h"

void SetCommutationStep(uint8_t step, uint16_t duty);
void threeSine(uint16_t degree);

#endif /* APPLICATION_USER_CORE_INC_COMMUTATION_H_ */
