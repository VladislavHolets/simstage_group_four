/*
 * obstacle_distance.cpp
 *
 *  Created on: Aug 8, 2019
 *      Author: captain
 */

#include "obstacle_distance.h"

#include <numeric>

ObstacleDistance::ObstacleDistance() {
	// TODO Auto-generated constructor stub
	this->back_left_ = 0;
	this->back_right_ = 0;
	this->front_ = 0;
	this->front_left_ = 0;
	this->front_right_ = 0;
	this->left_ = 0;
	this->right_ = 0;
	this->average_ = 0;
}

double ObstacleDistance::getBackLeft() const {
	return this->back_left_;
}

void ObstacleDistance::setBackLeft(double backLeft) {
	this->back_left_ = backLeft;
}

double ObstacleDistance::getBackRight() const {
	return this->back_right_;
}

void ObstacleDistance::setBackRight(double backRight) {
	this->back_right_ = backRight;
}

double ObstacleDistance::getFront() const {
	return this->front_;
}

void ObstacleDistance::setFront(double front) {
	this->front_ = front;
}

double ObstacleDistance::getFrontLeft() const {
	return this->front_left_;
}

void ObstacleDistance::setFrontLeft(double frontLeft) {
	this->front_left_ = frontLeft;
}

double ObstacleDistance::getFrontRight() const {
	return this->front_right_;
}

void ObstacleDistance::setFrontRight(double frontRight) {
	this->front_right_ = frontRight;
}

double ObstacleDistance::getLeft() const {
	return this->left_;
}

void ObstacleDistance::setLeft(double left) {
	this->left_ = left;
}

double ObstacleDistance::getRight() const {
	return this->right_;
}

double ObstacleDistance::getAverage() const {
	return average_;
}

void ObstacleDistance::setAverage(double average) {
	average_ = average;
}

void ObstacleDistance::setRight(double right) {
	this->right_ = right;
}

ObstacleDistance::~ObstacleDistance() {
	// TODO Auto-generated destructor stub
}


void ObstacleDistance::updateDistances(
		const sensor_msgs::LaserScan::ConstPtr& msg) {
	this->setAverage(
			std::accumulate(msg->ranges.begin(), msg->ranges.end(), 0.0)
					/ msg->ranges.size());
	this->setBackRight(
			*std::min_element(msg->ranges.begin(),
					msg->ranges.begin() + this->BACK_LEFT_ANGLE_));
	this->setRight(
			*std::min_element(msg->ranges.begin() + this->BACK_LEFT_ANGLE_,
					msg->ranges.begin() + this->BACK_LEFT_ANGLE_
							+ this->LEFT_ANGLE_));
	this->setFrontRight(
			*std::min_element(
					msg->ranges.begin() + this->BACK_LEFT_ANGLE_
							+ this->LEFT_ANGLE_,
					msg->ranges.begin() + this->BACK_LEFT_ANGLE_
							+ this->LEFT_ANGLE_ + this->FRONT_LEFT_ANGLE_));
	this->setFront(
			*std::min_element(
					msg->ranges.begin() + this->BACK_LEFT_ANGLE_
							+ this->LEFT_ANGLE_ + this->FRONT_LEFT_ANGLE_,
					msg->ranges.begin() + this->BACK_LEFT_ANGLE_
							+ this->LEFT_ANGLE_ + this->FRONT_LEFT_ANGLE_
							+ this->FRONT_ANGLE_));
	this->setFrontLeft(
			*std::min_element(
					msg->ranges.end() - this->BACK_RIGHT_ANGLE_
							- this->RIGHT_ANGLE_ - this->FRONT_RIGHT_ANGLE_,
					msg->ranges.end() - this->BACK_RIGHT_ANGLE_
							- this->RIGHT_ANGLE_));
	this->setLeft(
			*std::min_element(
					msg->ranges.end() - this->BACK_RIGHT_ANGLE_
							- this->RIGHT_ANGLE_,
					msg->ranges.end() - this->BACK_RIGHT_ANGLE_));
	this->setBackLeft(
			*std::min_element(msg->ranges.end() - this->BACK_RIGHT_ANGLE_,
					msg->ranges.end()));
}
