/*
 * reactivecontroller.cpp
 *
 *  Created on: Aug 11, 2019
 *      Author: captain
 */

#include "reactive_controller.h"

ReactiveController::ReactiveController() {
	// Initialize ROS
	this->n_ = ros::NodeHandle();

	// Create a publisher object, able to push messages
	this->cmd_vel_pub_ = this->n_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

	// Create a subscriber for laser scans
	this->laser_sub_ = this->n_.subscribe("base_scan", 10,
			&ReactiveController::laserCallback, this);

}

geometry_msgs::Twist ReactiveController::calculateCommand() {
	switch (this->robot_state) {
	case this->State::INITIAL: {
		this->robot_current_twist.linear.x = 0.5;
		this->robot_current_twist.linear.y = 0.0;
		this->robot_current_twist.linear.z = 0.0;
		this->robot_current_twist.angular.x = 0.0;
		this->robot_current_twist.angular.y = 0.0;
		this->robot_current_twist.angular.z = 0.0;
		break;
	}
	case this->State::FOUND_A_WALL: {
		this->robot_current_twist.linear.x = 0.0;
		this->robot_current_twist.linear.y = 0.0;
		this->robot_current_twist.linear.z = 0.0;
		this->robot_current_twist.angular.x = 0.0;
		this->robot_current_twist.angular.y = 0.0;
		this->robot_current_twist.angular.z = 0.0;
		break;
	}
	case this->State::FOLLOWING: {
		this->robot_current_twist.linear.x = 0.4;
		this->robot_current_twist.linear.y = 0.0;
		this->robot_current_twist.linear.z = 0.0;
		this->robot_current_twist.angular.x = 0.0;
		this->robot_current_twist.angular.y = 0.0;
		this->robot_current_twist.angular.z = 0.0;
		break;
	}
	case this->State::LOST_A_WALL: {
		this->robot_current_twist.linear.x = 0.25;
		this->robot_current_twist.linear.y = 0.0;
		this->robot_current_twist.linear.z = 0.0;
		this->robot_current_twist.angular.x = 0.0;
		this->robot_current_twist.angular.y = 0.0;
		this->robot_current_twist.angular.z = 15;
		break;
	}
	case this->State::TURNING_RIGHT: {
		this->robot_current_twist.linear.x = 0.0;
		this->robot_current_twist.linear.y = 0.0;
		this->robot_current_twist.linear.z = 0.0;
		this->robot_current_twist.angular.x = 0.0;
		this->robot_current_twist.angular.y = 0.0;
		this->robot_current_twist.angular.z = -0.5;
		break;
	}
	case this->State::TURNING_LEFT: {
		this->robot_current_twist.linear.x = 0.0;
		this->robot_current_twist.linear.y = 0.0;
		this->robot_current_twist.linear.z = 0.0;
		this->robot_current_twist.angular.x = 0.0;
		this->robot_current_twist.angular.y = 0.0;
		this->robot_current_twist.angular.z = 0.5;
		break;
	}
	}
	geometry_msgs::Twist msg(this->robot_current_twist);
	ROS_INFO("The state of robot is %d\n Distances: %f %f %f %f %f %f %f",
			this->robot_state, this->robot_obstacle_distances_.getBackLeft(),
			this->robot_obstacle_distances_.getLeft(),
			this->robot_obstacle_distances_.getFrontLeft(),
			this->robot_obstacle_distances_.getFront(),
			this->robot_obstacle_distances_.getFrontRight(),
			this->robot_obstacle_distances_.getRight(),
			this->robot_obstacle_distances_.getBackRight());
	return msg;
}

void ReactiveController::calculateRobotState() {
	switch (this->robot_state) {
	case this->State::INITIAL: {
		if ((this->robot_obstacle_distances_.getFront()
				< (this->DISTANCE_TO_THE_WALL_))) {
			this->robot_state = this->State::FOUND_A_WALL;

		} else if ((this->robot_obstacle_distances_.getLeft()
				< (this->DISTANCE_TO_THE_WALL_))) {
			this->robot_state = this->State::FOLLOWING;
		}
		break;
	}
	case this->State::FOUND_A_WALL: {
		if ((this->robot_obstacle_distances_.getLeft()
				<= (this->DISTANCE_TO_THE_WALL_))
				&& (this->robot_obstacle_distances_.getFrontLeft()
						>= (this->DISTANCE_TO_THE_WALL_ * 0.5))
				&& (this->robot_obstacle_distances_.getFront()
						> (this->DISTANCE_TO_THE_WALL_))) {
			this->robot_state = this->State::FOLLOWING;
		} else {
			this->get_back_in = 1;
			this->get_back_state = this->State::FOUND_A_WALL;
			this->robot_state = this->State::TURNING_RIGHT;
		}
		break;
	}
	case this->State::FOLLOWING: {
		if ((this->robot_obstacle_distances_.getFrontLeft()
				>= (this->DISTANCE_TO_THE_WALL_ * 1.1))) {
			this->robot_state = this->State::LOST_A_WALL;
		}
		if ((this->robot_obstacle_distances_.getLeft()
				>= (this->DISTANCE_TO_THE_WALL_ * 1.1))) {
			this->robot_state = this->State::LOST_A_WALL;
		}
		if ((this->robot_obstacle_distances_.getFront()
				< (this->DISTANCE_TO_THE_WALL_ * 0.8))
				|| (this->robot_obstacle_distances_.getFrontLeft()
						<= (this->DISTANCE_TO_THE_WALL_ * 0.8))) {
			this->robot_state = this->State::FOUND_A_WALL;
		}

		break;
	}
	case this->State::LOST_A_WALL: {
		if ((this->robot_obstacle_distances_.getLeft()
				< (this->DISTANCE_TO_THE_WALL_))) {
			this->robot_state = this->State::FOLLOWING;
		}
		if ((this->robot_obstacle_distances_.getFront()
				< (this->DISTANCE_TO_THE_WALL_ * 0.5))
				|| (this->robot_obstacle_distances_.getFrontLeft()
						<= (this->DISTANCE_TO_THE_WALL_ * 0.8))) {
			this->robot_state = this->State::FOUND_A_WALL;
		}
		break;
	}
	case this->State::TURNING_RIGHT: {
		if (this->get_back_in < 0) {
			this->robot_state = this->get_back_state;

		} else {
			this->get_back_in--;
		}
		break;
	}
	case this->State::TURNING_LEFT: {

		break;
	}
	}

}

void ReactiveController::laserCallback(
		const sensor_msgs::LaserScan::ConstPtr& msg) {
	this->robot_obstacle_distances_.updateDistances(msg);
	this->calculateRobotState();
}

void ReactiveController::run() {
	// Send messages in a loop
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		// Calculate the command to apply
		auto msg = calculateCommand();

		// Publish the new command
		this->cmd_vel_pub_.publish(msg);

		ros::spinOnce();

		// And throttle the loop
		loop_rate.sleep();
	}
}
