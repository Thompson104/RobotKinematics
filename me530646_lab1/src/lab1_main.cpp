#include <Eigen/Core>
#include <Eigen/LU>
#include <me530646_lab1/lab1.h>
#include <ur5/utilities.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>

#define PI 3.1415926
using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
	//Initializing ROS
    ros::init(argc, argv, "lab1_main");
    ros::NodeHandle node;
    //Creating an RvizPlotter
	RvizPlotter p = RvizPlotter(node);
	
	//Problem 5 (a)
	cout<<"Problem 5"<<endl;
	cout<<"************************"<<endl;
	cout<<"Matrix of rpyr(1,1,1):"<<endl;
	printEigen(rpyr(1,1,1));
	cout<<"Matrix of rpyr(1+2*PI,1,1):"<<endl;
	printEigen(rpyr(1+2*PI,1,1));

	//Problem 5 (c)
	cout<<"rpyrinv(rpyr(4*PI,0,0)):"<<endl;
	printEigen(rpyrinv(rpyr(4*PI,0,0)));
	cout<<"Vector3f(4*PI,0,0):"<<endl;
	printEigen(Eigen::Vector3f(4*PI,0,0));
	
	//Problem 5 (d)
	Eigen::Matrix3f m;
	m<<0,-1,0,
	   1,0,0,
	   0,0,1;
	cout<<"R:"<<endl;
	printEigen(m);
	cout<<"rpyr(rpyrinv(R)):"<<endl;
	printEigen(rpyr(rpyrinv(m)(0),rpyrinv(m)(1),rpyrinv(m)(2)));

	//problem 7
	cout<<"Problem 7"<<endl;
	cout<<"************************"<<endl;
	Eigen::Matrix4f t;
	t=xf(0,0,0,PI/4,PI/4,PI/4);
	cout<<"Original transformation Matrix"<<endl;
	printEigen(t);
	cout<<"Eigen inverse()"<<endl;
	printEigen(t.inverse());
	cout<<"Our implementation of inverse"<<endl;
	printEigen(finv(t));
	
	//Frame and vector demo
	Matrix4f r = xf(0,0,0,0,0,0);
	p.plotf(r*xf(2,-1,3,PI/3,PI/5,0),"frame1");
	p.plotf(xf(-1,2,0,PI,PI/3,-PI/2),"frame1","frame2");
	printEigen(getTransformation("map","frame2"));
	p.plotv("frame2",Vector3f(1,-1,0),Vector3f(1,2,-1));
	
	for(int i=1;i<300;i++){
		Matrix4f r = xf(0,0,0,PI*i/600,0,0);
		p.plotf(r*xf(2,-1,3,PI/3,PI/5,0),"frame1");
		p.plotf(xf(-1,2,0,PI+PI*i/100,PI/3,-PI/2),"frame1","frame2");
		printEigen(getTransformation("map","frame2"));
	}
	
	// Problem 8&9&10
	XYZFixedPlot(p,PI/4,PI/4,PI/4);
	ZYXRelativePlot(p,PI/4,PI/4,PI/4);

	/****************************************************/
}
