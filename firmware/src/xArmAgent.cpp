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

#define QUEUE_LENGTH 12

struct ServoCmd {
	uint8_t 	servo;
	float 		rad;
	uint			ms;
};
typedef struct ServoCmd ServoCmd_t;


xArmAgent::xArmAgent() {
	xCmdQueue = xQueueCreate(
			QUEUE_LENGTH,
             sizeof(ServoCmd_t)
			 );
	if (xCmdQueue == NULL){
		printf("Failed to allocate queue\n");
	}

	xArmSemaphore = xSemaphoreCreateBinary();
	if (xArmSemaphore == NULL){
		printf("Failed to allocate semaphore\n");
	} else {
		xSemaphoreGive(xArmSemaphore);
	}
}

xArmAgent::~xArmAgent() {
	if (xCmdQueue != NULL){
		vQueueDelete(xCmdQueue);
	}
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
			&xPubJoint,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
			"joint_states");

	rclc_subscription_init_default(
			  &xSubJointJog,
			  node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
			  "joint_jog");

	xCount = 2;
}

/***
 * Destroy the entities
 * @param node
 * @param support
 */
void xArmAgent::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	xCount = 0;
	rcl_publisher_fini(&xPubJoint, 		node);
	rcl_subscription_fini(&xSubJointJog, 	node);
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
	buildContext(&xSubJointJogContext, NULL);

	rclc_executor_add_subscription_with_context(
			executor,
			&xSubJointJog,
			&xJointJogMsg,
			uRosEntities::subscriptionCallback,
			&xSubJointJogContext,
			ON_NEW_DATA);
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
	ServoCmd_t cmd;
	setupRosMsgs();

	uint32_t lastPub = 0;

	for (;;){
		if (pArm != NULL){

			if (pdTRUE == xQueueReceive(
				   xCmdQueue,
                   &cmd,
                   0)){
				pArm->setRadPos(cmd.servo, cmd.rad, cmd.ms);
			}

			uint32_t now = to_ms_since_boot (get_absolute_time ());

			if (( now - lastPub) > 500){
				pubRosPos();
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
	ServoCmd_t cmd;

	if (context == &xSubJointJogContext){
		control_msgs__msg__JointJog * pJointJogMsg = (control_msgs__msg__JointJog *) msg;


		if (xSemaphoreTake(xArmSemaphore, 1000 ) == pdTRUE){
			for (int i=0; i < pJointJogMsg->joint_names.size; i++){
				int servo = -1;

				for (int j=0; j < 6; j++){
	/*DEBUG
					printf("\"%s\" (%d, %d) = %s (%d, %d)\n",
							pJointJogMsg->joint_names.data[i].data,
							pJointJogMsg->joint_names.data[i].size,
							pJointJogMsg->joint_names.data[i].capacity,
							xJointStateMsg.name.data[j].data,
							xJointStateMsg.name.data[j].size,
							xJointStateMsg.name.data[j].capacity
							);
	*/
					if (rosidl_runtime_c__String__are_equal(
							&xJointStateMsg.name.data[j],
							&pJointJogMsg->joint_names.data[i])
						){
							servo  = 6 - j;
							break;
						}
				}
				printf("JointJog %d = %f\n",
						servo,
						pJointJogMsg->displacements.data[i]
						);

				float a = pJointJogMsg->displacements.data[i];
				if (servo == 1){
					a = 2.8 - a;
				} else {
					a  = 2.07 - a;
				}

				pArm->setRadPos(servo, a,  (uint) (pJointJogMsg->duration * 1000.0));

				/*
				cmd.servo = servo;
				cmd.rad = a;
				cmd.ms =  (uint) (pJointJogMsg->duration * 1000.0);

				if (servo > 0){
					xQueueSendToBackFromISR(
							xCmdQueue,
							&cmd,
							NULL
							);
				}
				*/
			}
			xSemaphoreGive(xArmSemaphore );
		}// Semaphore
	}
}

/***
 * Setup the Ros Msg Structure;
 */
void xArmAgent::setupRosMsgs(){
	//Setup the joint state msg
	sensor_msgs__msg__JointState__init(&xJointStateMsg);
	rosidl_runtime_c__double__Sequence__init(&xJointStateMsg.position, 6);
	for (int i=0; i < 6; i++){
		xJointStateMsg.position.data[i] = 0.0;
	}
	xJointStateMsg.position.size = 6;
	xJointStateMsg.position.capacity = 6;
	/*
	rosidl_runtime_c__double__Sequence__init(&xJointStateMsg.velocity, 1);
	xJointStateMsg.velocity.data[0] = 0.0;
	xJointStateMsg.velocity.size = 1;
	xJointStateMsg.velocity.capacity = 1;
	*/
	rosidl_runtime_c__String__Sequence__init(&xJointStateMsg.name, 6);
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[0], pJoint6Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[1], pJoint5Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[2], pJoint4Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[3], pJoint3Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[4], pJoint2Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[5], pJoint1Name)){
		printf("ERROR: Joined assignment failed\n");
	}
	xJointStateMsg.name.size=6;
	xJointStateMsg.name.capacity=6;

	control_msgs__msg__JointJog__init(&xJointJogMsg);
	rosidl_runtime_c__String__Sequence__init(&xJointJogMsg.joint_names, 6);
	xJointJogMsg.joint_names.size=6;
	xJointJogMsg.joint_names.capacity=6;
	for (int i=0; i < 6; i++){
		rosidl_runtime_c__String__assign(&xJointJogMsg.joint_names.data[i], "armX");
	}
	rosidl_runtime_c__double__Sequence__init(&xJointJogMsg.displacements, 6);
	xJointJogMsg.displacements.size=6;
	xJointJogMsg.displacements.capacity=6;
}

/***
 * Pub Position to ROS
 */
void xArmAgent::pubRosPos(){


	if (xSemaphoreTake(xArmSemaphore, 1000 ) == pdTRUE){
		//Populate the Joint possition message
		int64_t time = rmw_uros_epoch_nanos();

		xJointStateMsg.header.stamp.sec = time / 1000000000;
		xJointStateMsg.header.stamp.nanosec = time % 1000000000;
		for (int i =0; i < 5; i++){
			double a = pArm->getRadPos(6 -i);
			if (a >= 0.0){
					xJointStateMsg.position.data[i] = 2.07 - a;
			} else {
				printf("Read failure on %d\n", 6-i);
			}
			vTaskDelay(100);
			//xJointStateMsg.position.data[i] =  0.0;
		}

		// Gripper
		double a = pArm->getRadPos(1);
		if (a >= 0.0){
			xJointStateMsg.position.data[5] =  2.8 - a;
		}
		if (!uRosBridge::getInstance()->publish(&xPubJoint,
				&xJointStateMsg,
				this,
				NULL)){
			//printf("Joint Pub failed\n");
		}
		xSemaphoreGive(xArmSemaphore );
	}
}
