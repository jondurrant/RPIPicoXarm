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
#include "uRosBridge.h"

#include "xArmAgent.h"
#include "BlinkAgent.h"

extern"C"{
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdio_uart.h"
}

//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

#define GP_STATUS 5
#define GP_BLINK 2
#define GP_U1TX 20
#define GP_U1RX 21


/***
 * Debug function to look at Task Stats
 */
void runTimeStats(   ){
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	unsigned long ulTotalRunTime;


   // Get number of takss
   uxArraySize = uxTaskGetNumberOfTasks();
   printf("Number of tasks %d\n", uxArraySize);

   //Allocate a TaskStatus_t structure for each task.
   pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

   if( pxTaskStatusArray != NULL ){
      // Generate raw status information about each task.
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 &ulTotalRunTime );

	 // Print stats
	 for( x = 0; x < uxArraySize; x++ )
	 {
		 printf("Task: %d \t cPri:%d \t bPri:%d \t hw:%d \t%s\n",
				pxTaskStatusArray[ x ].xTaskNumber ,
				pxTaskStatusArray[ x ].uxCurrentPriority ,
				pxTaskStatusArray[ x ].uxBasePriority ,
				pxTaskStatusArray[ x ].usStackHighWaterMark ,
				pxTaskStatusArray[ x ].pcTaskName
				);
	 }


      // Free array
      vPortFree( pxTaskStatusArray );
   } else {
	   printf("Failed to allocate space for stats\n");
   }

   //Get heap allocation information
   HeapStats_t heapStats;
   vPortGetHeapStats(&heapStats);
   printf("HEAP avl: %d, blocks %d, alloc: %d, free: %d\n",
		   heapStats.xAvailableHeapSpaceInBytes,
		   heapStats.xNumberOfFreeBlocks,
		   heapStats.xNumberOfSuccessfulAllocations,
		   heapStats.xNumberOfSuccessfulFrees
		   );
}


/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void *params){
	BlinkAgent blink(GP_BLINK);

	printf("Boot task started\n");
	//blink.start("Blink", TASK_PRIORITY);

	uart_init(uart1, 9600);
	uart_set_format( uart1,  8,  1, UART_PARITY_NONE);
	//gpio_set_function(20, GPIO_FUNC_UART);
	//gpio_set_function(21, GPIO_FUNC_UART);
	//gpio_set_function(8, GPIO_FUNC_UART);
	//gpio_set_function(9, GPIO_FUNC_UART);
	gpio_set_function(GP_U1TX, GPIO_FUNC_UART);
	gpio_set_function(GP_U1RX, GPIO_FUNC_UART);


	//xArmServoController xarm( xArm,  uart1);
	xArmServosRad xarm( xArm,  uart1);
	printf("Voltage %d\n", xarm.getBatteryVoltage());

	xArmAgent armAgent;
	armAgent.setArm(&xarm);
	uRosBridge *bridge = uRosBridge::getInstance();

	bridge->setuRosEntities(&armAgent);
	bridge->setLed(GP_STATUS);
	bridge->start("Bridge",  TASK_PRIORITY+2);
	armAgent.start("xArm Agent", TASK_PRIORITY);


	for (int i=1; i <= 6; i++){
		xarm.setPosition(i,  500);
		vTaskDelay(100);
	}

	vTaskDelay(2000);

	for (int i=1; i <= 6; i++){
		xarm.servoOff(i);
		vTaskDelay(100);
	}

	for (;;){
		vTaskDelay(1000);
	}

	for (;;){
		printf("POS: ");
		for (int i=5; i >=0; i--){
			printf("%d=%.2f, ",
				i + 1,
				xarm.getRadPos(i)
				);
		}
		printf("\n");
		vTaskDelay(1000);
	}


   // xarm.setPosition(3,  50);
   // xarm.servoOff(3);
	//xarm.servoOff(4);
	//xarm.servoOff(5);
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
			//xarm.setPosition( servos, 4,  1000,   true);
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


/***
 * Launch the tasks and scheduler
 */
void vLaunch( void) {

	//Start blink task
    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 500, NULL, TASK_PRIORITY, &task);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}




int main(){


	stdio_init_all();
	sleep_ms(1000);
	stdio_filter_driver(&stdio_uart);
	sleep_ms(1000);

	printf("Go\n");

    //Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();

	for (;;){
		sleep_ms(3000);
	}



}


