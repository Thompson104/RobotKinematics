#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <ur5/utilities.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>
#include <me530646_lab4/lab4.h>
#include <climits>
#include <vector>
#include "project.cpp"
#define DELTA_P 0.001;
using namespace std;
using namespace Eigen;
#define PI M_PI

void linear_trajectory(double** poses,vector<double*>& trajectory);
double** min_dis(Eigen::Matrix4f Ti, Eigen::Matrix4f Tf);
double** min_dis(double* initial, Eigen::Matrix4f Tf);
void straight_line(double **pose,vector<double*>& trajectory);

double** min_dis(Eigen::Matrix4f Ti, Eigen::Matrix4f Tf){
	if (singular(Ti,4)) {
		std::cout << "T1 is singular\nAborting..." << std::endl;
		std::exit(0);
	}
	if (singular(Tf,4)) {
		std::cout << "T1 is singular\nAborting..." << std::endl;
		std::exit(0);
	}
	//init jontposition for initial pose and final pose
	double *q_sol_i[8];
	double *q_sol_f[8];

	for(int i = 0; i < 8; i++)
	{
		q_sol_i[i] = new double[6];
		q_sol_f[i] = new double[6];
	}
	//solve inverse kinematics
	int num_sol_i = inverse(Ti,q_sol_i);
	int num_sol_f = inverse(Tf,q_sol_f);
	if (num_sol_i == 0) {
		std::cout << "Ti not in workspace of UR5\nAborting..." << std::endl;
		std::exit(0);
	}
	if (num_sol_f == 0) {
		std::cout << "Tf not in workspace of UR5\nAborting..." << std::endl;
		std::exit(0);
	}
	double dis_min = INT_MAX;
	double dis = 0;
	int min_i = 0;
	int min_f = 0;
	//searching for minimum distance
	for(int i = 0;i<num_sol_i;i++){
		for(int j = 0; j<num_sol_f;j++){
			dis = 0;
			for(int k = 0;k<6;k++){
				dis += pow((q_sol_i[i][k] - q_sol_f[j][k]),2);
			}
			 
			if(dis < dis_min){
				dis_min = dis;
				min_i = i;
				min_f = j;
			}
		}
	}
	//return smallest poses
	static double* re[2];
	re[0] = new double[6];
	re[1] = new double[6];

	for(int k = 0;k<6;k++){
		re[0][k] = q_sol_i[min_i][k];
		re[1][k] = q_sol_f[min_f][k];
	}


	return re;
}

//input the initial pose only, not solving for inverse kinematics for the initial pose
double** min_dis(double* q_i, Eigen::Matrix4f Tf){
	if (singular(UR5::fwd(q_i),4)) {
		std::cout << "T1 is singular\nAborting..." << std::endl;
		std::exit(0);
	}
	if (singular(Tf,4)) {
		std::cout << "T1 is singular\nAborting..." << std::endl;
		std::exit(0);
	}
	//init jointposition for initial pose and final pose
	double *q_sol_f[8];

	for(int i = 0; i < 8; i++)
	{
		q_sol_f[i] = new double[6];
	}
	//solve inverse kinematics
	int num_sol_f = inverse(Tf,q_sol_f);
	if (num_sol_f == 0) {
		std::cout << "Tf not in workspace of UR5\nAborting..." << std::endl;
		std::exit(0);
	}
	double dis_min = INT_MAX;
	double dis = 0;
	int min_f = 0;
	//searching for minimum distance
	
	for(int j = 0; j<num_sol_f;j++){
		dis = 0;
		
		for(int k = 0;k<6;k++){
			dis += pow((q_i[k] - q_sol_f[j][k]),2);
		}
			 
		if(dis < dis_min){
			dis_min = dis;
			min_f = j;
		}
	}
	
	//return smallest poses
	static double* re[2];
	re[0] = new double[6];
	re[1] = new double[6];

	for(int k = 0;k<6;k++){
		re[0][k] = q_i[k];
		re[1][k] = q_sol_f[min_f][k];
	}


	return re;
}

//linearly interpolate the joint trajectory
void linear_trajectory(double** poses,vector<double*>& trajectory){
	double joint[6] = {poses[0][0],poses[0][1],poses[0][2],poses[0][3],poses[0][4],poses[0][5]};
	bool joint_max[6] = {false,false,false,false,false,false};

	//loop until all joint reach the final position
	while(!(joint_max[0] && joint_max[1] && joint_max[2] && joint_max[3] && joint_max[4] && joint_max[5])){
	
		double* step = new double[6];
		
		for(int i=0;i<6;i++){
			step[i] = joint[i];
		}
		
		trajectory.push_back(step);
		
		for(int i=0;i<6;i++){
			//DELTA is a small joint movement
			double err = DELTA_P;
			if(abs(joint[i] - poses[1][i])<=err){
				joint_max[i] = true;
			}else if(joint[i] > poses[1][i]){
				joint[i] -= DELTA_P;
			}else if(joint[i] <= poses[1][i]){
				joint[i] += DELTA_P;
			}
		}

	}

}

void straight_line(double **pose,vector<double*>& trajectory){
	MatrixXf Ti = UR5::fwd(pose[0]);
	MatrixXf Tf = UR5::fwd(pose[1]);
	int k_v = 4;
	int k_w = 7;
	double delta = 0.03;
	
	MatrixXf R = Ti.block<3,3>(0,0);
	MatrixXf Rf = Tf.block<3,3>(0,0);
	Vector3f p = Ti.block<3,1>(0,3);
	Vector3f pf = Tf.block<3,1>(0,3);
	
	Vector3f w;
	Vector3f v;
	
	//initialize first pose
	double *newpose =new double[6];
	double currentpose[6];
	for(int k = 0;k<6;k++){
		currentpose[k] = pose[0][k];
		newpose[k] = currentpose[k];
	}
	trajectory.push_back(newpose);
	char a;
	cin>>a;
	while(fabs(sqrt(pow(v(0,0),2)+pow(v(1,0),2)+pow(v(2,0),2))) > 0.01 || fabs(sqrt(pow(w(0,0),2)+pow(w(1,0),2)+pow(w(2,0),2)))>0.1){
		cout<<"v"<<fabs(sqrt(pow(v(0,0),2)+pow(v(1,0),2)+pow(v(2,0),2)))<<endl;
		cout<<"w"<<fabs(sqrt(pow(w(0,0),2)+pow(w(1,0),2)+pow(w(2,0),2)))<<endl;
		MatrixXf skew = (R*Rf.transpose()).log();
		Vector3f W = Vector3f(skew(2,1),skew(0,2),skew(1,0));

		w = -k_w * W;
		v = -k_v * (p - pf);
		
		MatrixXf jacobian = J(currentpose);
		
		VectorXf combine = VectorXf(6,1);
		combine.block<3,1>(0,0) = v;
		combine.block<3,1>(3,0) = w;
		VectorXf q = jacobian.transpose() * combine;
		
		newpose =new double[6];
		
		for(int k=0;k<6;k++){
			currentpose[k] += q[k]*delta;
			newpose[k] = currentpose[k];
		}
		
		trajectory.push_back(newpose);
		printf("pose = [%f,%f,%f,%f,%f,%f]\n",newpose[0],newpose[1],newpose[2],newpose[3],newpose[4],newpose[5]);
		
		MatrixXf Tn = UR5::fwd(currentpose);
		R = Tn.block<3,3>(0,0);
		p = Tn.block<3,1>(0,3);
	}

}





