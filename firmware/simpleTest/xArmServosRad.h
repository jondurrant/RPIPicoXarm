/*
 * xArmServosRad.h
 *
 *  Created on: 27 Mar 2024
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_XARMSERVOSRAD_H_
#define FIRMWARE_SRC_XARMSERVOSRAD_H_

#include "xArmServoController.h"

#include <stdarg.h>
#include <pico/stdlib.h>
#include "hardware/uart.h"

class xArmServosRad : public  xArmServoController{
public:
	xArmServosRad(xArmMode mode, uart_inst_t * uart);
	virtual ~xArmServosRad();

	 float getRadPos(int servo_id);


};

#endif /* FIRMWARE_SRC_XARMSERVOSRAD_H_ */
