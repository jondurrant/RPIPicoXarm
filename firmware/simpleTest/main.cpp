/*
 * main.cpp
 *
 *  Created on: 9 Jun 2022
 *      Author: jondurrant
 */

#include <cstdio>

#include "hardware/uart.h"
#include "xArmServoController.h"
#include "xArmServosRad.h"


int main(){


	stdio_init_all();
	sleep_ms(2000);

	printf("Go\n");
	uart_init(uart1, 9600);
    uart_set_format( uart1,  8,  1, UART_PARITY_NONE);
	gpio_set_function(20, GPIO_FUNC_UART);
    gpio_set_function(21, GPIO_FUNC_UART);

    //xArmServoController xarm( xArm,  uart1);
    xArmServosRad xarm( xArm,  uart1);
    printf("Voltage %d\n", xarm.getBatteryVoltage());

   // xarm.setPosition(3,  50);
   // xarm.servoOff(3);
    xarm.servoOff(4);
    xarm.servoOff(5);
    //xarm.servoOff(6);


    xArmServo servos[6];
    for (int i=0; i < 6; i++){
    	servos[i].servo_id = 6 - i ;
    	servos[i].position = 0;
    }


    int pos = 100;
    bool dir = true;
	for (;;){
			printf("Set %u\n", pos);
			//xarm.setPosition(6, i, 300, true);
			for (int j=0; j < 6; j++){
				servos[j].position = pos;
			}
			xarm.setPosition( servos, 4,  1000,   true);
			sleep_ms(1000);
			//printf("Get (%d) %d\n", i, xarm.getPosition(6));

			printf("%d, %d, %d, %d \n",
					xarm.getPosition(3),
					xarm.getPosition(4),
					xarm.getPosition(5),
					xarm.getPosition(6)
					);

			printf("%.2f, %.2f, %.2f, %.2f \n",
					xarm.getRadPos(3),
					xarm.getRadPos(4),
					xarm.getRadPos(5),
					xarm.getRadPos(6)
					);

			if (dir){
				pos += 100;
				if (pos > 900){
					pos = 800;
					dir = false;
				}
			} else {
				pos -= 100;
				if (pos < 100){
					pos = 200;
					dir = true;
				}
			}
			sleep_ms(2000);
	}




}


