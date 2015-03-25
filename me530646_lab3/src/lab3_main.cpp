#include <Eigen/Core>
#include <ur5/utilities.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>
#include <iostream>
using namespace std;
using namespace Eigen;
int main(int argc, char **argv){
	ros::init(argc,argv,"lab3");
	ros::NodeHandle n;
	UR5 robot = UR5(n);
	
	//frist joint position
	double q1[] = {1,1,2,3,1,0};
	MatrixXf v(6,1);
	v << 1,1,2,3,1,0;
	cout<<"Joint Position 1:"<<endl;
	printEigen(v);
	robot.plotfwd(q1);
	robot.plotframes(q1);
	cout<<"Transformation from robot_base to end_effector:"<<endl;
	printEigen(getTransformation("base_link","Frame5"));
	//stopping point
	char a;
	cin>>a;
	//second joint position
	double q2[] = {2,1,1,1,1,1};
	v << 2,1,1,1,1,1;
	cout<<"Joint Position 2:"<<endl;
	printEigen(v);
	robot.plotfwd(q2);
	robot.plotframes(q2);
	cout<<"Transformation from robot_base to end_effector:"<<endl;
	printEigen(getTransformation("base_link","Frame5"));
	
}
