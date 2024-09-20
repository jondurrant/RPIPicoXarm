/*
 * xArmServosRad.cpp
 *
 *  Created on: 27 Mar 2024
 *      Author: jondurrant
 */

#include "xArmServosRad.h"
#include <cstdio>

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


void xArmServosRad::setRadPos(
		int servo_id,
		float rad,
		unsigned duration){
	if ((rad < 0.0) || (rad > 4.18879)) {
		return;
	}

	int p = (int)((rad / 4.18879 ) * 1000.0);
	printf("Setting %d (%f) to %d\n", servo_id, rad, p);
	setPosition(servo_id, p, duration, false);
}
