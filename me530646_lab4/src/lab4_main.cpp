#include <Eigen/Core>
#include <ur5/utilities.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>
#include <me530646_lab4/lab4.h>
#include <ur_kinematics/ur_kin.h>
#include <iostream>
using namespace std;
using namespace Eigen;
int main(int argc, char **argv){
	ros::init(argc,argv,"lab4");
	ros::NodeHandle n;
	UR5 robot = UR5(n);

	//How to call inverse
	double q[6] = {2.2,3.5,4.2,2.9,3.3,1.2};
	double *q_sol[8];
	for(int i = 0; i < 8; i++)
	{
		q_sol[i] = new double[6];
	}
	int num_sol = inverse(UR5::fwd(q),q_sol);
	cout<<"For q[6] = {2.2,3.5,4.2,2.9,3.3,1.2}, the solutions of inverse kinematics are:"<<endl;
	for(int i=0;i<num_sol;i++){
		printf("q_sol = [%f,%f,%f,%f,%f,%f]\n",q_sol[i][0],q_sol[i][1],q_sol[i][2],q_sol[i][3],q_sol[i][4],q_sol[i][5]);
	}
	robot.plotfwd(q_sol[7]);
	//problem3
	//declare 3 different vectors
	double joint_pos[3][6] = {0.2,3.8,1.5,1.5,2.1,5.8,
							  2.2,3.5,4.2,2.9,3.3,1.2,
							  2.0,3.2,4.1,2.8,3.2,1.3};
	
	for(int k=0;k<3;k++){
		
		printf("*Joint Position%d=[%f,%f,%f,%f,%f,%f]\n",k,joint_pos[k][0],joint_pos[k][1],joint_pos[k][2],joint_pos[k][3],joint_pos[k][4],joint_pos[k][5]);
		
		for(int m=0;m<2;m++){
			//small angle 
			double small_angle[6];
			for(int i=0;i<6;i++){
				small_angle[i] = (double)(rand() % 10) / 10000;
			}
			cout<<"**********************************"<<endl;
			printf("*Small angle%d=[%f,%f,%f,%f,%f,%f]\n",m,small_angle[0],small_angle[1],small_angle[2],small_angle[3],small_angle[4],small_angle[5]);
		
			//vector from small angle
			VectorXf delta(6,1);
			for(int i=0;i<6;i++){
				delta(i) = small_angle[i];
			}
			//joint angle with small angle
			double joint_with_small_angle[6];
			for(int i=0;i<6;i++){
				joint_with_small_angle[i] = joint_pos[k][i] +  small_angle[i];
			}
			//compute fwd and fwd with small angle
			Matrix4f H0_6 = fwd(joint_pos[k]);
			Matrix4f H0_6_delta = fwd(joint_with_small_angle);
			MatrixXf Jacobian = J(joint_pos[k]); //compute Jacobian
	
			//compare translation difference
			Vector3f diff = H0_6_delta.block<3,1>(0,3) - H0_6.block<3,1>(0,3) ;
			Vector3f diff_J = Jacobian.block<3,6>(0,0) * delta;
			cout<<"*P0_6(theta) - P0_6(theta+delta):"<<endl<<diff<<endl<<"*J * delta:"<<endl<<diff_J<<endl;
			//compute rotation difference
			Matrix3f A = (H0_6_delta.block<3,3>(0,0) - H0_6.block<3,3>(0,0)) * H0_6.block<3,3>(0,0).transpose();
			Matrix3f B = skew3(Jacobian.block<3,6>(3,0) * delta); 
			float frobenius = ((A-B).transpose() * (A -B)).trace();
			cout<<"*The Frobenius difference between A and B is:"<< frobenius<<endl;
	
		}
	}
}	
	


