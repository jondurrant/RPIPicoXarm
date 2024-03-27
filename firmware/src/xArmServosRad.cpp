/*
 * xArmServosRad.cpp
 *
 *  Created on: 27 Mar 2024
 *      Author: jondurrant
 */

#include "xArmServosRad.h"

xArmServosRad::xArmServosRad(xArmMode mode, uart_inst_t * uart): xArmServoController(mode, uart) {
	// NOP

}

xArmServosRad::~xArmServosRad() {
	// TODO Auto-generated destructor stub
}


float xArmServosRad::getRadPos(int servo_id){
	// 0 			= 		0Deg  					0.0 Rad
	// 1000 	=  	240Deg				4.18879Rad
	int p =  getPosition(servo_id);
	if (( p > 1000) || (p < 0)){
		return -1.0;
	}

	float rad = p * (4.18879 / 1000);
	return rad;
}
