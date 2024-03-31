/*
 * xArmAgent.h
 *
 *  Created on: 31 Mar 2024
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_XARMAGENT_H_
#define FIRMWARE_SRC_XARMAGENT_H_

#include "Agent.h"
#include "uRosEntities.h"

#include "xArmServosRad.h"

extern "C"{
#include <sensor_msgs/msg/joint_state.h>
}

class xArmAgent  : public Agent, public uRosEntities {
public:
	xArmAgent();
	virtual ~xArmAgent();

	/***
	 * Create the entities (Publishers)
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Destroy the entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Return the number of entities
	 * @return
	 */
	virtual uint getCount();

	/***
	 * Return the number of handles needed by the executor
	 * @return
	 */
	virtual uint getHandles();


	/***
	 * Add subscribers, guards and timers to the executor
	 * @param executor
	 */
	virtual void addToExecutor(rclc_executor_t *executor);

	/***
	 * Call back on a publish to show msg has been completed.
	 * Can be used to free up allocated memory
	 * @param msg - ROS Msg
	 * @param args - Arguments passed into the publish step
	 * @param status -
	 */
	virtual void pubComplete(void *msg, void *args, uRosPubStatus status);


	void setArm(xArmServosRad *arm);

protected:

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

	/***
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* Context);

	/***
	 * Setup the Ros Msg Structure;
	 */
	void setupRosMsgs();

	/***
	 * Pub Position to ROS
	 */
	void pubRosPos();

private:
	rcl_publisher_t xPubJoint;
	uint xCount = 0;

	sensor_msgs__msg__JointState xJointStateMsg;
	const char *pJoint6Name = "arm6";

	xArmServosRad * pArm = NULL;

};

#endif /* FIRMWARE_SRC_XARMAGENT_H_ */
