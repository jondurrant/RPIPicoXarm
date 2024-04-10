/*
 * xArmAgent.cpp
 *
 *  Created on: 31 Mar 2024
 *      Author: jondurrant
 */

#include "xArmAgent.h"
#include "uRosBridge.h"

#define _USE_MATH_DEFINES
#include <math.h>

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rmw_microros/time_sync.h"
}



xArmAgent::xArmAgent() {


	xArmSemaphore = xSemaphoreCreateBinary();
	if (xArmSemaphore == NULL){
		printf("Failed to allocate semaphore\n");
	} else {
		xSemaphoreGive(xArmSemaphore);
	}
}

xArmAgent::~xArmAgent() {
	if (xArmSemaphore != NULL){
		vSemaphoreDelete(xArmSemaphore);
	}
}

/***
 * Create the entities (Publishers)
 * @param node
 * @param support
 */
void xArmAgent::createEntities(rcl_node_t *node, rclc_support_t *support){
	rclc_publisher_init_default(
			&xPubJointJog,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
			"joint_jog");

	xCount = 1;
}

/***
 * Destroy the entities
 * @param node
 * @param support
 */
void xArmAgent::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	xCount = 0;
	rcl_publisher_fini(&xPubJointJog, 		node);
}

/***
 * Return the number of entities
 * @return
 */
uint xArmAgent::getCount(){
	return xCount;
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint xArmAgent::getHandles(){
	return 1;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void xArmAgent::addToExecutor(rclc_executor_t *executor){
}

/***
 * Call back on a publish to show msg has been completed.
 * Can be used to free up allocated memory
 * @param msg - ROS Msg
 * @param args - Arguments passed into the publish step
 * @param status -
 */
void xArmAgent::pubComplete(void *msg, void *args, uRosPubStatus status){

}


void xArmAgent::setArm(xArmServosRad *arm){
	pArm = arm;
}


/***
 * Run loop for the agent.
 */
void xArmAgent::run(){
	setupRosMsgs();

	uint32_t lastPub = 0;

	for (;;){
		if (pArm != NULL){

			uint32_t now = to_ms_since_boot (get_absolute_time ());

			if (( now - lastPub) > 500){
				pubRosPos();
				lastPub = now;
			}
		}
		vTaskDelay(10);
	}
}


/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE xArmAgent::getMaxStackSize(){
	return 250;
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void xArmAgent::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){

}

/***
 * Setup the Ros Msg Structure;
 */
void xArmAgent::setupRosMsgs(){

	control_msgs__msg__JointJog__init(&xJointJogMsg);
	rosidl_runtime_c__String__Sequence__init(&xJointJogMsg.joint_names, 4);
	xJointJogMsg.joint_names.size=4;
	xJointJogMsg.joint_names.capacity=4;
	if (!rosidl_runtime_c__String__assign(&xJointJogMsg.joint_names.data[0], pJoint6Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointJogMsg.joint_names.data[1], pJoint5Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointJogMsg.joint_names.data[2], pJoint4Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointJogMsg.joint_names.data[3], pJoint3Name)){
		printf("ERROR: Joined assignment failed\n");
	}


	rosidl_runtime_c__double__Sequence__init(&xJointJogMsg.displacements, 4);
	xJointJogMsg.displacements.size=4;
	xJointJogMsg.displacements.capacity=4;


	xJointJogMsg.duration = 0.5;
}

/***
 * Pub Position to ROS
 */
void xArmAgent::pubRosPos(){


	if (xSemaphoreTake(xArmSemaphore, 1000 ) == pdTRUE){
		//Populate the Joint possition message
		int64_t time = rmw_uros_epoch_nanos();

		xJointJogMsg.header.stamp.sec = time / 1000000000;
		xJointJogMsg.header.stamp.nanosec = time % 1000000000;


		double a = pArm->getRadPos(6);
		xJointJogMsg.displacements.data[0] = 2.07 - a;
		vTaskDelay(200);

		a = pArm->getRadPos(5);
		xJointJogMsg.displacements.data[1] = 2.07 - a;
		vTaskDelay(200);

		a = pArm->getRadPos(4);
		xJointJogMsg.displacements.data[2] = 2.07 - a;
		vTaskDelay(200);

		a = pArm->getRadPos(3);
		xJointJogMsg.displacements.data[3] = 2.07 - a;

		xJointJogMsg.displacements.size = 4;
		xJointJogMsg.joint_names.size = 4;



		if (!uRosBridge::getInstance()->publish(&xPubJointJog,
				&xJointJogMsg,
				this,
				NULL)){
			//printf("Joint Pub failed\n");
		}
		xSemaphoreGive(xArmSemaphore );
	}
}
