/*
 * obstacle_distance.h
 *
 *  Created on: Aug 8, 2019
 *      Author: captain
 */

#ifndef SIMSTAGE_GROUP_FOUR_SRC_OBSTACLE_DISTANCE_H_
#define SIMSTAGE_GROUP_FOUR_SRC_OBSTACLE_DISTANCE_H_

#include <sensor_msgs/LaserScan.h>

class ObstacleDistance {
	static const int BACK_LEFT_ANGLE_ =15 ,
					 LEFT_ANGLE_ = 45,
					 FRONT_LEFT_ANGLE_ = 20,
					 FRONT_ANGLE_ = 60,
					 FRONT_RIGHT_ANGLE_ = 20,
					 RIGHT_ANGLE_ = 45,
					 BACK_RIGHT_ANGLE_ = 15;
	double back_left_, left_, front_left_, front_, front_right_, right_,
			back_right_, average_;
	void setBackLeft(double backLeft);
	void setBackRight(double backRight);
	void setFront(double front);
	void setFrontLeft(double frontLeft);
	void setFrontRight(double frontRight);
	void setLeft(double left);
	void setRight(double right);
	void setAverage(double average);
public:
	ObstacleDistance();
	virtual ~ObstacleDistance();
	double getBackLeft() const;
	double getBackRight() const;
	double getFront() const;
	double getFrontLeft() const;
	double getFrontRight() const;
	double getLeft() const;
	double getRight() const;
	double getAverage() const;
	void updateDistances(const sensor_msgs::LaserScan::ConstPtr& msg);

};

#endif /* SIMSTAGE_GROUP_FOUR_SRC_OBSTACLE_DISTANCE_H_ */
