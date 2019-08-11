#include <ros/init.h>

#include "../include/reactive_controller.h"

int main(int argc, char **argv) {
// Initialize ROS
	ros::init(argc, argv, "reactive_controller");

// Create our controller object and run it
	auto controller = ReactiveController();
	controller.run();

// And make good on our promise
	return 0;
}

