#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <ur5/utilities.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>
#include <me530646_lab4/lab4.h>
#include "project_han.cpp"
#include <ur_kinematics/ur_kin.h>
// #include "project.cpp"

#define PI M_PI


int main(int argc, char **argv){
	
	ros::init(argc,argv,"lab3");
	ros::NodeHandle n;
	UR5 robot = UR5(n);
/******************** Lab Exercises *****************************/
	double q1[6] = {1.7,0.1,2.1,3.0,0.87,1.9};
	double q2[6] = {2.2,2.5,1.2,2.9,2.3,1.2};
	double** re;
	re = min_dis(q1,UR5::fwd(q2));
	vector<double*> result;
	linear_trajectory(re,result);
// 4b - Linear interpolation between initial and final joint vectors
	for (int i = 0; i < result.size(); i ++) {
		robot.plotfwd(result[i]);
		ros::Duration(0.005).sleep();
	}

	double cur1[6]; // min_i
	double cur2[6]; // min_i
	double end[6];  // min_f
	for (int i = 0; i < 6; i++) {
		cur1[i] = re[0][i];
		cur2[i] = re[0][i];
		end[i] = re[1][i];
	}

	Eigen::MatrixXf temp;
// 5b - Trajectory calculated using transpose of the Jacobian
	while(!converge(cur1,end)) {
		robot.plotfwd(cur1);
		temp = traj2(cur1,end)*epsilon;
		for (int i =0; i < 6; i++) {
			cur1[i] += temp(i,0);
			cur1[i] = cur1[i];
		}
	}
// 5c - Trajectory calculated using inverse of the Jacobian
	while(!converge(cur2,end)) {
		robot.plotfwd(cur2);
		temp = traj3(cur2,end)*epsilon;
		for (int i =0; i < 6; i++) {
			cur2[i] += temp(i,0);
			cur2[i] = cur2[i];
		}
	}

/************* RViz Demo (Jacobian Transpose) ****************************
// This is the Demo in RViz of the "creative" implementation of our 
// trajectory calculations. The gripper grabs a can, pours it into a cup,
// and places the can down in a new location.
	double q_demo1[6] = {0,0,0,0,0,0};
	double q_demo2[6] = {2*PI/3,-PI/4,PI/2,-5*PI/8,0,0};
	double q_demo3[6] = {5*PI/4,-PI/3,PI/2,-5*PI/8,0,0};
	double q_demo4[6] = {5*PI/4,-PI/3,PI/2,-5*PI/8,-7*PI/12,0};
	double q_demo5[6] = {PI,-PI/6,PI/3,-5*PI/8,0,0};

	std::cout << "At initial position" << std::endl;
	ros::Duration(1).sleep();

	// robot.openHand();
	Eigen::MatrixXf temp;
	while(!converge(q_demo1,q_demo2)) {
		robot.plotfwd(q_demo1);
		temp = traj2(q_demo1,q_demo2)*epsilon;
		for (int i =0; i < 6; i++) {
			q_demo1[i] += temp(i,0);
		}
	}
	// robot.closeHand();
	std::cout << "Grasped Can" << std::endl;
	ros::Duration(1).sleep();

	while(!converge(q_demo2,q_demo3)) {
		robot.plotfwd(q_demo2);
		temp = traj2(q_demo2,q_demo3)*epsilon;
		for (int i =0; i < 6; i++) {
			q_demo2[i] += temp(i,0);
		}
	}
	std::cout << "Reached Glass" << std::endl;
	ros::Duration(1).sleep();

	while(!converge(q_demo3,q_demo4)) {
		robot.plotfwd(q_demo3);
		temp = traj2(q_demo3,q_demo4)*epsilon;
		for (int i =0; i < 6; i++) {
			q_demo3[i] += temp(i,0);
		}
	}
	std::cout << "Poured Can into Glass" << std::endl;
	ros::Duration(1).sleep();

	while(!converge(q_demo4,q_demo5)) {
		robot.plotfwd(q_demo4);
		temp = traj2(q_demo4,q_demo5)*epsilon;
		for (int i =0; i < 6; i++) {
			q_demo4[i] += temp(i,0);
		}
	}

	// robot.openHand();
	std::cout << "Placed can down" << std::endl;
/*************** UR5 LIVE DEMO *************************************
	double q_demo1[6] = {1.7,0.1,2.1,3.0,0.87,1.9};
	double q_demo2[6] = {2.2,2.5,1.2,2.9,2.3,1.2};
	double q_demo3[6] = {1.1,0.7,3.1,2.3,1.5,.97};
	double q_demo4[6] = {3.1,0.4,2.6,2.9,1.0,3.1};
	double** re_demo;
	re_demo = min_dis(q_demo1,UR5::fwd(q_demo2));
	vector<double*> result1;
	linear_trajectory(re_demo,result1);

	robot.openHand();

	for (int i = 0; i < result1.size(); i ++) {
		robot.movefwd(result1[i]);
		ros::Duration(0.005).sleep();
	}

	robot.closeHand();

	re_demo = min_dis(q_demo2,UR5::fwd(q_demo3));
	vector<double*> result2;
	linear_trajectory(re_demo,result2);
	for (int i = 0; i < result2.size(); i ++) {
		robot.movefwd(result2[i]);
		ros::Duration(0.005).sleep();
	}

	re_demo = min_dis(q_demo3,UR5::fwd(q_demo4));
	vector<double*> result3;
	linear_trajectory(re_demo,result3);
	for (int i = 0; i < result3.size(); i ++) {
		robot.movefwd(result3[i]);
		ros::Duration(0.005).sleep();
	}

	robot.openHand();
********************************************************************/
}
