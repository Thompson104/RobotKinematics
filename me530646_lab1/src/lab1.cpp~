#include <Eigen/Core>
#include <me530646_lab1/lab1.h>
#include <ur5/utilities.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>

#define PI 3.1415926
using namespace Eigen;
using namespace std;
/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the x axis (roll) of phi radians.
 */
Eigen::Matrix3f rollr(double phi){
	//TODO
	Eigen::Matrix3f r = Eigen::Matrix3f::Zero();
	r(0,0) = 1;
	r(1,1) = cos(phi);
	r(1,2) = -sin(phi);
	r(2,1) = sin(phi);
	r(2,2) = cos(phi);
	return r;
}

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the y axis (pitch) of theta radians.
 */
Eigen::Matrix3f pitchr(double theta){
	//TODO		 
	Eigen::Matrix3f r = Eigen::Matrix3f::Zero();
	r(0,0) = cos(theta);
	r(0,2) = sin(theta);
	r(1,1) = 1;
	r(2,0) = -sin(theta);
	r(2,2) = cos(theta);
	return r;
}

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the z axis (yaw) of psi radians.
 */
Eigen::Matrix3f yawr(double psi){
	//TODO
	Eigen::Matrix3f r = Eigen::Matrix3f::Zero();
	r(0,0) = cos(psi);
	r(0,1) = -sin(psi);
	r(1,0) = sin(psi);
	r(1,1) = cos(psi);
	r(2,2) = 1;
	return r;
}

/**
 * Returns the 3x3 rotation matrix that represents successive 
 * roll, pitch, and yaw rotations.
 */
Eigen::Matrix3f rpyr(double roll, double pitch, double yaw){
	//TODO
	Eigen::Matrix3f r = Eigen::Matrix3f::Zero();
	r = yawr(yaw)*pitchr(pitch)*rollr(roll); //computer the rotation matrix
	return r;
}

/**
 * Returns the roll, pitch, and yaw from the given 3x3 rotation matrix.
 */
Eigen::Vector3f rpyrinv(Eigen::Matrix3f r){
	//TODO
	
	//There might be more than ome solution, we just return 1 possible
	//solution for this function.
	//The algorithm is from Spong and MLS book
	//define the vector
	Eigen::Vector3f vec = Eigen::Vector3f::Zero();
	
	if(r(2,0) != 1 && r(2,0)!= -1){
		vec(1) = -asin(r(2,0));
		vec(0) = atan2(r(2,1)/cos(vec(1)),r(2,2)/cos(vec(1)));
		vec(2) = atan2(r(1,0)/cos(vec(1)),r(0,0)/cos(vec(1)));
	}else{
		vec(2) = 0;
		if(r(2,0) == -1){
			vec(1) = PI/2;
			vec(0) = atan2(r(0,1),r(0,2));
		}else{
			vec(1) = -PI/2;
			vec(0) = atan2(-r(0,1),-r(0,2));
		}
	}

	return vec;
}

/**
 * Returns the 4x4 homogeneous transformation that represents a 
 * translation of (x,y,z) and a rotation of (roll,pitch,yaw).
 */
Eigen::Matrix4f xf(double x, double y, double z, double roll, double pitch, double yaw){
	//TODO
	Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
	t.block<3,3>(0,0) = rpyr(roll,pitch,yaw);
	t.block<3,1>(0,3) = Eigen::Vector3f(x,y,z);
	return t;
}

/**
 * Returns the matrix inverse to the given homogeneous transformation.
 */
Eigen::Matrix4f finv(Eigen::Matrix4f f){
	//TODO
	//We don't use the inbuild inverse() function, we use a shortcut. The property of homogeneous transformation.
	//Details can be found from MLS book.
	Eigen::Matrix4f t = f;
	Eigen::Matrix3f temp = t.block<3,3>(0,0); 
	t.block<3,3>(0,0) = temp.transpose();
	t.block<3,1>(0,3) = -temp.transpose()*t.block<3,1>(0,3);

	return t;
}

/**
 * Animates a roll-pitch-yaw rotation about the world
 * coordinate frame
 */
void XYZFixedPlot(RvizPlotter &p,double roll, double pitch, double yaw){
	//TODO
	double d = PI/400; //divided the circle into 400 parts. 
	
	double r = 0;
	double pi = 0;
	double y = 0;
	
	//roll rotation
	while(r < roll){
		p.plotf(roll4(r),"Fixed");
		r+= d;
	}
	//pitch rotaion
	while(pi < pitch){
		p.plotf(pitch4(pi)*roll4(r),"Fixed");
		pi+= d;
	}
	//yaw rotation
	while(y < yaw){
		p.plotf(yaw4(y)*pitch4(pi)*roll4(r),"Fixed");
		y+= d;
	}
	
	//Hint:First apply the roll rotation in increments,
	//using plotf to draw the frame in rviz
}

/**
 * Animates a roll-pitch-yaw rotation about the body-fixed
 * coordinate frame
 */
void ZYXRelativePlot(RvizPlotter &p,double roll, double pitch, double yaw){
	//TODO
	double d = PI/400; //divided the circle into 800 parts. 
	
	double r = 0;
	double pi = 0;
	double y = 0;
	//yaw rotation
	while(y < yaw){
		p.plotf(yaw4(y),"Relative");
		y+= d;
	}
	//pitch rotaion
	while(pi < pitch){
		p.plotf(yaw4(y)*pitch4(pi),"Relative");
		pi+= d;
	}
	//roll rotation
	while(r < roll){
		p.plotf(yaw4(y)*pitch4(pi)*roll4(r),"Relative");
		r+= d;
	}
	
	
}


////////////////////Helpful Functions///////////////////
/**
 * Returns the homogenous transformation for rotation
 * about the x axis.
 */
Eigen::Matrix4f roll4(double roll)
{
	Eigen::Matrix4f r = Eigen::MatrixXf::Identity(4,4);
	r.block<3,3>(0,0) = rollr(roll);
	return r;
}

/**
 * Returns the homogenous transformation for rotation
 * about the y axis.
 */
Eigen::Matrix4f pitch4(double pitch)
{
	Eigen::Matrix4f r = Eigen::MatrixXf::Identity(4,4);
	r.block<3,3>(0,0) = pitchr(pitch);
	return r;
}

/**
 * Returns the homogenous transformation for rotation
 * about the z axis.
 */
Eigen::Matrix4f yaw4(double yaw)
{
	Eigen::Matrix4f r = Eigen::MatrixXf::Identity(4,4);
	r.block<3,3>(0,0) = yawr(yaw);
	return r;
}
