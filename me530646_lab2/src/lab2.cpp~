#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <me530646_lab1/lab1.h>
#include <Eigen/Eigenvalues>
#define PI 3.1415926
using namespace Eigen;
using namespace std;
/**
 * Returns the 3x3 skew-symmetric matrix that corresponds to vector e.
 */
Eigen::Matrix3f skew3(Eigen::Vector3f e){
	//TODO
	Matrix3f r = Matrix3f::Zero();
	r(0,1) = -e(2);
	r(0,2) = e(1);
	r(1,0) = e(2);
	r(1,2) = -e(0);
	r(2,0) = -e(1);
	r(2,1) = e(0);
	return r;
}

/**
 * Returns the 3x3 rotation matrix that corresponds to a rotation of 
 * |e| radians about vector e.
 */
Eigen::Matrix3f expr(Eigen::Vector3f e){
	//TODO
	Matrix3f r = Matrix3f::Zero();
	r = skew3(e).exp();
	return r;
}

/**
 * Returns the (x,y,z) translation and (roll,pitch,yaw) rotations
 * of the given homogeneous transformation.
 */
Eigen::VectorXf xfinv(Eigen::Matrix4f xf){
	Eigen::VectorXf xyzrpy(6);
	//TODO
	xyzrpy.block<3,1>(0,0) = xf.block<3,1>(0,3);
	xyzrpy.block<3,1>(3,0) = rpyrinv(xf.block<3,3>(0,0));

	return xyzrpy;
}


