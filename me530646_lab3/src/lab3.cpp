#include <Eigen/Core>
#include <ros/ros.h>
#include <ur5/utilities.h>
#include <ur5/ur5_joints.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>
#include <iostream>

#define PI 3.1415926
using namespace Eigen;
using namespace std;

//data got from the UR5 manuel
//An array of the six alpha D-H parameters for the UR5
double UR5::alpha[] = {PI/2,0,0,PI/2,-PI/2,0};
//An array of the six a D-H parameters for the UR5
double UR5::a[] = {0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000};
//An array of the six d D-H parameters for the UR5
double UR5::d[] = {0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823};


/**
 * Creates an instance of the class to allow plotting 
 * of the UR5 in rviz.
 */
UR5::UR5(ros::NodeHandle &n):rvizPlotter(n){
 	fwd_pb = n.advertise<ur5::ur5_joints>("forwardKinematics", 10);
	link_names.push_back("shoulder");
	link_names.push_back("upper_arm");
	link_names.push_back("forearm");
	link_names.push_back("wrist_1");
	link_names.push_back("wrist_2");
	link_names.push_back("wrist_3");
	link_names.push_back("end-effector");
}

/**
 * Returns the 4x4 transformation given the a 4x1 vector of
 * Denavit-Hartenberg parameters.
 */
Eigen::Matrix4f UR5::dhf(double alpha, double a, double d, double theta){
	//TODO
	Matrix4f r = Matrix4f::Zero();
	r(0,0) = cos(theta);
	r(0,1) = -sin(theta) * cos(alpha);
	r(0,2) = sin(theta) * sin(alpha);
	r(0,3) = a * cos(theta);
	r(1,0) = sin(theta);
	r(1,1) = cos(theta) * cos(alpha);
	r(1,2) = -cos(theta) * sin(alpha);
	r(1,3) = a * sin(theta);
	r(2,1) = sin(alpha);
	r(2,2) = cos(alpha);
	r(2,3) = d;
	r(3,3) = 1;
	return r;         
}

/**
 * Returns the 4x4 transformation from the base of the
 * UR5 to its gripper given a 6x1 vector of the joint
 * angles.
 */
Eigen::Matrix4f UR5::fwd(double q[]){
	//TODO
	Matrix4f tr = MatrixXf::Identity(4,4);
	for(int i=0;i<6;i++){
		tr = tr * dhf(alpha[i],a[i],d[i],q[i]);
	}
	return tr;
}

/**
 * Plots the robot in rviz by defining its joint angles.
 */
 void UR5::plotfwd(double q[]){
	ur5::ur5_joints j;

	j.q.push_back(q[0]-PI);
	for(int i = 1; i < 6; i++)
	{
	  j.q.push_back(q[i]);
	}
	while(fwd_pb.getNumSubscribers() < 1){}
	fwd_pb.publish(j);
}

/**
 * Plots the coordinate frames (according to D-H convention) for the
 * UR5 given a set of joint angles. Each frame is plotted relative to
 * fixed frame base_link.
 */	
 void UR5::plotframes(double q[]){
 	//TODO
 	//For plotting use: this->rvizPlotter.plotf(...)
 	
 	char buffer [10];
 	Matrix4f tr = MatrixXf::Identity(4,4);
	for(int i=0;i<6;i++){
		sprintf (buffer, "Frame%d", i);   //Frame0 - Frame5 where Frame5 is the end-effector frame.
		tr = tr*dhf(alpha[i],a[i],d[i],q[i]);
		this->rvizPlotter.plotf(tr,"base_link",buffer);
	}
	
}


