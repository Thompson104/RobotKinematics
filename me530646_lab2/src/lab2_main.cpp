#include <Eigen/Core>
#include <ur5/utilities.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <Eigen/Eigenvalues>
#include <iostream>
#define PI 3.1415926
using namespace Eigen;
using namespace std;

int main(int argc, char **argv){
	//Initializing ROS
    ros::init(argc, argv, "lab1_main");
    ros::NodeHandle node;
    //Creating an RvizPlotter
	RvizPlotter p = RvizPlotter(node);
	//Problem 2
	Eigen::Vector3f v = Eigen::MatrixXf::Random(3,1);
	cout<<"Generate a random vector v:"<<endl;;
	printEigen(v);
	double normV = v.norm();
	cout<<"Norm of v is:"<<normV<<endl;;
	Eigen::Vector3f v_unit = v/normV;
	cout<<"The unit vector of v is"<<endl;
	printEigen(v_unit);
	Eigen::Matrix3f v_skew = skew3(v);
	cout<<"The skew of v is"<<endl;
	printEigen(v_skew);
	cout<<"Compute v_skew * v"<<endl;
	printEigen(v_skew * v);

	Eigen::EigenSolver<Eigen::MatrixXf> es(v_skew);
	//VectorXcf MatrixXcf are used because there may 
	//be complex values for eigenvalues and eigenvectors
	Eigen::VectorXcf evals = es.eigenvalues();
	cout<<"The eigenvalues of v_skew are"<<endl;
	printEigen(evals);
	Eigen::MatrixXcf evecs = es.eigenvectors();
	cout<<"The eigenvectors of v_skew are"<<endl;
	printEigen(evecs);
	//Calculating the matrix expontential of matrix m
	//Eigen::Matrix4f m = Eigen::MatrixXf::Random(4,4);
	Eigen::Matrix3f expV = v_skew.exp();
	cout<<"The exponential of v_skew is"<<endl;
	printEigen(expV);
	cout<<"The value of exp(v_skew) * v is"<<endl;
	printEigen(expV * v);
	//Calculating the eigenvalues and eigenvectors of m
	Eigen::EigenSolver<Eigen::MatrixXf> es2(expV);
	//VectorXcf MatrixXcf are used because there may 
	//be complex values for eigenvalues and eigenvectors
	Eigen::VectorXcf evals2 = es2.eigenvalues();
	cout<<"The eigenvalues of v_skew exponential are"<<endl;
	printEigen(evals2);
	Eigen::MatrixXcf evecs2 = es2.eigenvectors();
	cout<<"The eigenvectors of v_skew exponential are"<<endl;
	printEigen(evecs2);

	//Problem 3
	//generate a original frame
	Matrix4f r = xf(0,0,0,0,0,0);
	p.plotf(r,"Original");
	//generate a random vector
	Eigen::Vector3f rv = Vector3f::Ones();
	//plot the vector
	p.plotv("map",Vector3f(0,0,0),rv);
	//perform rotation along the vector
	Matrix4f rr = Matrix4f::Identity();
	rr.block<3,3>(0,0) = expr(rv);
	p.plotf(rr,"Rotated");

	//Problem 4
	cout<<"xf(1,1,1,1,1,1) is"<<endl;
	printEigen(xf(1,1,1,1,1,1));
	cout<<"xf(1,1,1,1+2*PI,1,1) is"<<endl;
	printEigen(xf(1,1,1,1+2*PI,1,1));
	
	MatrixXf r2(6,1);
	r2(0,0) = 0;
	r2(1,0) = 0;
	r2(2,0) = 0;
	r2(3,0) = 4*PI;
	r2(4,0) = 0;
	r2(5,0) = 0;
	cout<<"vector r2 is"<<endl;
	printEigen(r2);
	cout<<"xfinv(xf(0,0,0,4PI,0,0))is"<<endl;
	printEigen(xfinv(xf(r2(0),r2(1),r2(2),r2(3),r2(4),r2(5))));
	
	Matrix4f H = Matrix4f::Identity();
	H(0,3)=1;
	H(1,3)=1;
	H(2,3)=1;
	cout<<"H is"<<endl;
	printEigen(H);
	VectorXf wr2(6); 
	wr2 = xfinv(H);
	cout<<"xf(xfinv(H)) is"<<endl;
	printEigen(xf(wr2(0),wr2(1),wr2(2),wr2(3),wr2(4),wr2(5)));
	
	//Problem 5
	double d = PI/600; //divided the circle into 800 parts. 
	Matrix4f H01 = Matrix4f::Identity();
	Matrix4f H12 = Matrix4f::Identity();
	Matrix4f H23 = Matrix4f::Identity();
	Vector3f e = Vector3f::Zero();
	Vector3f e2 = Vector3f::Zero();
	double q1 = 0;
	
	while(q1 < 5*PI/4){
		e(0) = q1;
		e2(2) = q1;
		H01.block<3,1>(0,3) = e;	
		H12.block<3,3>(0,0) = expr(e);
		H23.block<3,1>(0,3) = e2;
		
		p.plotf(H01,"Frame1");
		p.plotf(H12,"Frame1","Frame2");
		p.plotf(H23,"Frame2","Frame3");
		q1 += d;
	}
	cout<<"The calculated value is:"<<endl;
	printEigen(H01*H12*H23);
	cout<<"The Transformation H03 is"<<endl;
	printEigen(getTransformation("map","Frame3"));
}
