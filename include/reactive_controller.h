/*
 * reactivecontroller.h
 *
 *  Created on: Aug 11, 2019
 *      Author: captain
 */

#ifndef SIMSTAGE_GROUP_FOUR_INCLUDE_REACTIVE_CONTROLLER_H_
#define SIMSTAGE_GROUP_FOUR_INCLUDE_REACTIVE_CONTROLLER_H_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>

#include "obstacle_distance.h"

class ReactiveController {
private:
	const double DISTANCE_TO_THE_WALL_ = 0.4;
	ros::NodeHandle n_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber laser_sub_;
	ObstacleDistance robot_obstacle_distances_;
	geometry_msgs::Twist robot_current_twist;
	//semantic enum for state machine used to controll the parameters;
	enum State {
		INITIAL,
		FOUND_A_WALL,
		FOLLOWING,
		LOST_A_WALL,
		WALL_IN_FRONT,
		TURNING_LEFT,
		TURNING_RIGHT
	};
	//iterator used to get back from recursive state
	int get_back_in = 0;
	//states for current state of the robot and the one to get back after iterator is 0
	ReactiveController::State robot_state = INITIAL, get_back_state = INITIAL;
	//this function changes the speed accordingly to the states of the robot
	geometry_msgs::Twist calculateCommand();
	//this function calculates the state of the robot accordingly to the minimal distances in differen angles
	void calculateRobotState();
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
public:
	ReactiveController();
	void run();

};

#endif /* SIMSTAGE_GROUP_FOUR_INCLUDE_REACTIVE_CONTROLLER_H_ */
