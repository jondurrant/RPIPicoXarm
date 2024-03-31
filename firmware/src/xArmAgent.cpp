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
	// TODO Auto-generated constructor stub

}

xArmAgent::~xArmAgent() {
	// TODO Auto-generated destructor stub
}

/***
 * Create the entities (Publishers)
 * @param node
 * @param support
 */
void xArmAgent::createEntities(rcl_node_t *node, rclc_support_t *support){
	rclc_publisher_init_default(
			&xPubJoint,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
			"joint_states");
	xCount = 1;
}

/***
 * Destroy the entities
 * @param node
 * @param support
 */
void xArmAgent::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	xCount = 0;
	rcl_publisher_fini(&xPubJoint, 		node);
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
	return 0;
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


	for (;;){
		if (pArm != NULL){
			pubRosPos();
		}
		vTaskDelay(250);
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
void xArmAgent::handleSubscriptionMsg(const void* msg, uRosSubContext_t* Context){

}

/***
 * Setup the Ros Msg Structure;
 */
void xArmAgent::setupRosMsgs(){
	//Setup the joint state msg
	sensor_msgs__msg__JointState__init(&xJointStateMsg);
	rosidl_runtime_c__double__Sequence__init(&xJointStateMsg.position, 1);
	xJointStateMsg.position.data[0] = 0.0;
	xJointStateMsg.position.size = 1;
	xJointStateMsg.position.capacity = 1;
	rosidl_runtime_c__double__Sequence__init(&xJointStateMsg.velocity, 1);
	xJointStateMsg.velocity.data[0] = 0.0;
	xJointStateMsg.velocity.size = 1;
	xJointStateMsg.velocity.capacity = 1;
	rosidl_runtime_c__String__Sequence__init(&xJointStateMsg.name, 1);
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[0], pJoint6Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	xJointStateMsg.name.size=1;
	xJointStateMsg.name.capacity=1;
}

/***
 * Pub Position to ROS
 */
void xArmAgent::pubRosPos(){
	//Populate the Joint possition message
	int64_t time = rmw_uros_epoch_nanos();

	xJointStateMsg.header.stamp.sec = time / 1000000000;
	xJointStateMsg.header.stamp.nanosec = time % 1000000000;
	double a = pArm->getRadPos(6);
	xJointStateMsg.position.data[0] = a;
	xJointStateMsg.velocity.data[0] = 0.0;
	if (!uRosBridge::getInstance()->publish(&xPubJoint,
			&xJointStateMsg,
			this,
			NULL)){
		//printf("Joint Pub failed\n");
	}
}
