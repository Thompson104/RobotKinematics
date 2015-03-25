#define PI M_PI
#define kw 1
#define kv 1
#define epsilon 0.001
#define approxZero 0.0001

/*
Returns false if matrix R is full rank (rank(R) = n)
Returns true otherwise.
*/
bool singular(Eigen::MatrixXf R,int n){
  double rank = 0;
  Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(R);
  rank = lu_decomp.rank();
  return rank != n;
}

/*
Returns the desired end effector velocity vector (6x1) defines as 
[v;w]. This is calculated based on the current configuration of the
UR5 and the desired end configuration of the manipulator.
*/
Eigen::MatrixXf vel(double cur[6], double f[6]) {
	Eigen::MatrixXf T = UR5::fwd(cur);
	Eigen::MatrixXf Tf = UR5::fwd(f);
	Eigen::MatrixXf R = T.block<3,3>(0,0);
	Eigen::MatrixXf Rf = Tf.block<3,3>(0,0);

	Eigen::MatrixXf w_skew = (T*Tf.transpose()).log();
	Eigen::MatrixXf w(3,1);
	w << w_skew(2,1),w_skew(0,2),w_skew(1,0);
	
	Eigen::MatrixXf v (6,1); 
	v.block<3,1>(0,0) = -kv*(T.block<3,1>(0,3)-Tf.block<3,1>(0,3)); //v
	v.block<3,1>(3,0) = -kw*w; //omega

	return v;
}

/*
Returns the desired joint velocity vector, as calculated by the transpose
of the Jacobian of the current configuration of the UR5 multiplied by the
desired end effector veloicty vector. 
*/
Eigen::MatrixXf traj2(double cur[6], double f[6]) {
	return J(cur).transpose()*vel(cur,f);
}

/*
Returns the desired joint velocity vector, as calculated by multiplying the
matrix inverse of the Jacobian by the desired end effector velocity vector.
If the Jacobian matrix is singular, an error message is printed to the 
console and the program is aborted.
*/
Eigen::MatrixXf traj3(double cur[6], double f[6]) {
	Eigen::MatrixXf Jac = J(cur);
	if (!singular(Jac,6)) {
		// std::cout << "v= " << vel(cur,f).block<3,1>(0,0).norm() << " w= " << vel(cur,f).block<3,1>(3,0).norm() << std::endl;
		return Jac.inverse()*vel(cur,f);
	}
		std::cout << "Jacobian Matrix is Singular\nAborting...";
		std::exit(0);
}
/*
Returns true if the angular and linear velocities have converged.
Returns false otherwise.
*/
bool converge(double q1[6], double q2[6]) {
	Eigen::MatrixXf v = vel(q1,q2);
	return (v.block<3,1>(3,0).norm() < .125 && v.block<3,1>(0,0).norm() < .125);
}
