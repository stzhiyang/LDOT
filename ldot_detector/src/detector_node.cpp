/*
	FILE: detector_node.cpp
	--------------------------
	Run detector node
*/
#include <ros/ros.h>
#include <ldot_detector/dynamicDetector.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "ldot_detector_node");
	ros::NodeHandle nh;

	onboardDetector::dynamicDetector d (nh);

	ros::spin();

	return 0;
}