#include <Eigen/Core>
#include <ur5/utilities.h>
#include <math.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>
#include <me530646_lab4/lab4.h>

#define PI M_PI
#define approxZero 0.00001
using namespace Eigen;
using namespace std;

//data got from the UR5 manuel
//An array of the six alpha D-H parameters for the UR5
double alpha[] = {PI/2,0,0,PI/2,-PI/2,0};
//An array of the six a D-H parameters for the UR5
double a[] = {0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000};
//An array of the six d D-H parameters for the UR5
double d[] = {0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823};

Eigen::Matrix4f dhf(double alpha, double a, double d, double theta); //helper function for computing dhf
void constrain(double& input); //constrain for angles;
Eigen::Matrix4f fwd(double q[]); //helper function compute foward kinematics

Eigen::MatrixXf J(double q[6])
{
  //TODO
  Matrix4f T0_1 = dhf(alpha[0],a[0],d[0],q[0]);
  Matrix4f T1_2 = dhf(alpha[1],a[1],d[1],q[1]);
  Matrix4f T2_3 = dhf(alpha[2],a[2],d[2],q[2]);
  Matrix4f T3_4 = dhf(alpha[3],a[3],d[3],q[3]);
  Matrix4f T4_5 = dhf(alpha[4],a[4],d[4],q[4]);
  Matrix4f T5_6 = dhf(alpha[5],a[5],d[5],q[5]);
  
  Matrix4f T0_2 = T0_1 * T1_2;
  Matrix4f T0_3 = T0_2 * T2_3;
  Matrix4f T0_4 = T0_3 * T3_4;
  Matrix4f T0_5 = T0_4 * T4_5;
  Matrix4f T0_6 = T0_5 * T5_6;
  MatrixXf Jacobian = MatrixXf::Zero(6,6);
  Vector3f z[6];
  
  z[0] << 0,0,1;
  z[1] = T0_1.block<3,3>(0,0) * Vector3f(0,0,1);
  z[2] = T0_2.block<3,3>(0,0) * Vector3f(0,0,1);
  z[3] = T0_3.block<3,3>(0,0) * Vector3f(0,0,1);
  z[4] = T0_4.block<3,3>(0,0) * Vector3f(0,0,1);
  z[5] = T0_5.block<3,3>(0,0) * Vector3f(0,0,1);
  //z[5] = T0_6.block<3,3>(0,0) * Vector3f(0,0,1);
  
  
  Jacobian.block<3,1>(0,0) = z[0].cross(T0_6.block<3,1>(0,3));
  Jacobian.block<3,1>(0,1) = z[1].cross(T0_6.block<3,1>(0,3)- T0_1.block<3,1>(0,3));
  Jacobian.block<3,1>(0,2) = z[2].cross(T0_6.block<3,1>(0,3)- T0_2.block<3,1>(0,3));
  Jacobian.block<3,1>(0,3) = z[3].cross(T0_6.block<3,1>(0,3)- T0_3.block<3,1>(0,3));
  Jacobian.block<3,1>(0,4) = z[4].cross(T0_6.block<3,1>(0,3)- T0_4.block<3,1>(0,3));
  Jacobian.block<3,1>(0,5) = z[5].cross(T0_6.block<3,1>(0,3)- T0_5.block<3,1>(0,3));
  
  Jacobian.block<3,1>(3,0) = z[0];
  Jacobian.block<3,1>(3,1) = z[1];
  Jacobian.block<3,1>(3,2) = z[2];
  Jacobian.block<3,1>(3,3) = z[3];
  Jacobian.block<3,1>(3,4) = z[4];
  Jacobian.block<3,1>(3,5) = z[5];
  
  return Jacobian;
}

int inverse(Eigen::Matrix4f H0_6, double **q, double q6Des){
  int numSols = 0;
  /*==========================================
  /       Solving for q1
  /=========================================*/
  double q1[2];
  Vector4f P0_5 = H0_6 * Vector4f(0,0,-d[5],1) - Vector4f(0,0,0,1);
  double psi = atan2(P0_5(1),P0_5(0));
  double phi_1 = acos(d[3]/sqrt(pow(P0_5(0),2)+pow(P0_5(1),2)));
  double phi_2 = -phi_1;
  q1[0] = PI/2 + phi_1 + psi;
  q1[1] = PI/2 + phi_2 + psi;
  constrain(q1[0]);
  constrain(q1[1]); //round angle to positive
  /*==========================================
  /       Solving for q5
  /=========================================*/
  double q5[2][2];
  double P1_6_z = H0_6(0,3) * sin(q1[0]) - H0_6(1,3) * cos(q1[0]);
  //two conditions with different theta 1 angle
  double c_5_1 = (P1_6_z - d[3])/d[5];
  P1_6_z = H0_6(0,3) * sin(q1[1]) - H0_6(1,3) * cos(q1[1]);
  double c_5_2 = (P1_6_z - d[3])/d[5];
  //number of solution for theta 5
  int solution_5 = 0;
  if(c_5_1 > 1 && c_5_2 > 1)  //check for well defined angle
    return numSols;
  if(c_5_1 <= 1 && c_5_2 > 1){
    q5[0][0] = acos(c_5_1);
    q5[1][0] = -acos(c_5_1);
    constrain(q5[0][0]);
    constrain(q5[0][1]);
    solution_5 = 1;
  }else if (c_5_1 > 1 && c_5_2 <= 1){
    q5[0][1] = acos(c_5_2);
    q5[1][1] = -acos(c_5_2);
    constrain(q5[0][0]);
    constrain(q5[0][1]);
    solution_5 = 1;
  }else{
    q5[0][0] = acos(c_5_1);
    q5[0][1] = acos(c_5_2);
    q5[1][0] = -acos(c_5_1);
    q5[1][1] = -acos(c_5_2);
    constrain(q5[0][0]);
    constrain(q5[0][1]);
    constrain(q5[1][0]);
    constrain(q5[1][1]);
    solution_5 = 2;
  }
  /*==========================================
  /       Solving for q6,q2-q4
  /=========================================*/
  double q6[2][2];
  double q3[2][2][2];
  double q2[2][2][2];
  double q4[2][2][2];
  int solution_3 = 0;
  
  //For each solution to q1
  for(int i = 0; i < 2; i++)
  {
    //For each solution to q5
    for(int j = 0; j < solution_5; j++)
    {
      //solve for q6
      Matrix4f T0_1 = dhf(alpha[0],a[0],d[0],q1[i]);
      Matrix4f T1_6 = T0_1.inverse() * H0_6;
      Matrix4f T6_1 = T1_6.inverse();

      if(T6_1(1,2) == 0 || T6_1(0,2) == 0) //check for well defined angles
        return 0;
      if(sin(q5[j][i]) == 0)
        continue;
        
      q6[j][i] = atan2(-T6_1(1,2)/sin(q5[j][i]),T6_1(0,2)/sin(q5[j][i]));
      constrain(q6[j][i]);

      //solve for q3
      Matrix4f T4_5 = dhf(alpha[4],a[4],d[4],q5[j][i]);
      Matrix4f T5_6 = dhf(alpha[5],a[5],d[5],q6[j][i]);
      
      Matrix4f T1_4 = T1_6 * (T4_5 * T5_6).inverse();
      Vector4f P1_3 = T1_4 * Vector4f(0,-d[3],0,1) - Vector4f(0,0,0,1);
      q3[0][j][i] = +acos((P1_3.transpose() * P1_3 - a[1]*a[1] - a[2]*a[2])/(2*a[1]*a[2]));
      q3[1][j][i] = -acos((P1_3.transpose() * P1_3 - a[1]*a[1] - a[2]*a[2])/(2*a[1]*a[2]));
      constrain(q3[0][j][i]);
      constrain(q3[1][j][i]);
      solution_3 = 2;
      
      //For each solution to q2/q3
      for(int k = 0; k < solution_3; k++)
      {   
        //solve fo q2
        q2[k][j][i] = -atan2(P1_3(1),-P1_3(0)) + asin(a[2] * sin(q3[k][j][i]) / sqrt(P1_3(0)*P1_3(0)+P1_3(1)*P1_3(1)));
        constrain(q2[k][j][i]);
        //sovle for q4
        Matrix4f T1_2 = dhf(alpha[1],a[1],d[1],q2[k][j][i]);
        Matrix4f T2_3 = dhf(alpha[2],a[2],d[2],q3[k][j][i]);
        Matrix4f T3_4 = (T1_2 * T2_3).inverse() * T1_4;
        q4[k][j][i] = atan2(T3_4(1,0),T3_4(0,0));
		constrain(q4[k][j][i]);
		
        q[numSols][0] = q1[i];
        q[numSols][1] = q2[k][j][i];
        q[numSols][2] = q3[k][j][i];
        q[numSols][3] = q4[k][j][i];
        q[numSols][4] = q5[j][i];
        q[numSols][5] = q6[j][i];
        numSols++;
      }
    }
  }
  return numSols;
}

Eigen::Matrix4f dhf(double alpha, double a, double d, double theta){
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

void constrain(double& input){
	if(input - 0 < -0.01)
  		input +=2*PI;
  	else if(input - 0 < 0)
  		input = 0;
}

Eigen::Matrix4f fwd(double q[]){
	//TODO
	Matrix4f tr = MatrixXf::Identity(4,4);
	for(int i=0;i<6;i++){
		tr = tr * dhf(alpha[i],a[i],d[i],q[i]);
	}
	return tr;
}
