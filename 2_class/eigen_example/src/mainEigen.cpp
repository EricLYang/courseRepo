#include <iostream>
#include <ctime>
// This is the main module of Eigen providing dense matrix and vector support (both fixed and dynamic size) with all the features corresponding to a BLAS library and much more
#include <Eigen/Core>
// #include "Core"  "LU"  "Cholesky" QR" "SVD" "Geometry" "Eigenvalues"
#include <Eigen/Dense>


/****************************
* Usage example of eigen for pose
****************************/

int main( int argc, char** argv )
{
    //Eigen vector usage
    Eigen::Vector3d pos;
		pos << 0.0,-2.0,1.0;
		Eigen::Vector3d pos2 = Eigen::Vector3d(1,-3,0.5);
		pos = pos+pos2;

		Eigen::Vector3d pos3 = 	Eigen::Vector3d::Zero();

		pos.normalize();
		pos.inverse();

		//Dot and Cross Product
		Eigen::Vector3d v(1, 2, 3);
		Eigen::Vector3d w(0, 1, 2);

		double vDotw = v.dot(w); // dot product of two vectors
		Eigen::Vector3d vCrossw = v.cross(w); // cross product of two vectors

    // Eigen: Affine3d for pose message: including quarternion
    Eigen::Quaterniond rot;

		rot.setFromTwoVectors(Eigen::Vector3d(0,1,0), pos);

		Eigen::Matrix<double,3,3> rotationMatrix;

		rotationMatrix = rot.toRotationMatrix();

		Eigen::Quaterniond q(2, 0, 1, -3); 
		std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " 
    << std::endl << q.vec() << std::endl;

		q.normalize();

		std::cout << "To represent rotation, we need to normalize it such that its length is " 
    << q.norm() << std::endl;

		Eigen::Vector3d vec(1, 2, -1);
		Eigen::Quaterniond p;
		p.w() = 0;
		p.vec() = vec;
		Eigen::Quaterniond rotatedP = q * p * q.inverse(); 
		Eigen::Vector3d rotatedV = rotatedP.vec();
		std::cout << "We can now use it to rotate a vector " << std::endl 
    << vec << " to " << std::endl << rotatedV << std::endl;

		// convert a quaternion to a 3x3 rotation matrix:
		Eigen::Matrix3d R = q.toRotationMatrix(); 

		std::cout << "Compare with the result using an rotation matrix " << std::endl << R * vec << std::endl;

		Eigen::Quaterniond a = Eigen::Quaterniond::Identity();
		Eigen::Quaterniond b = Eigen::Quaterniond::Identity();
		Eigen::Quaterniond c; 
		// Adding two quaternion as two 4x1 vectors is not supported by the Eigen API.
		//That is, c = a + b is not allowed. The solution is to add each element:

		c.w() = a.w() + b.w();
		c.x() = a.x() + b.x();
		c.y() = a.y() + b.y();
		c.z() = a.z() + b.z();

   // Eigen Matrix usage
   //Transpose and inverse:
		Eigen::MatrixXd A(3, 2);
		A << 1, 2,
				 2, 3,
				 3, 4;

		Eigen::MatrixXd B = A.transpose();// the transpose of A is a 2x3 matrix
		// computer the inverse of BA, which is a 2x2 matrix:
		Eigen::MatrixXd C = (B * A).inverse();
		C.determinant(); //compute determinant
		Eigen::Matrix3d D = Eigen::Matrix3d::Identity();

		Eigen::Matrix3d m = Eigen::Matrix3d::Random();
		m = (m + Eigen::Matrix3d::Constant(1.2)) * 50;
		Eigen::Vector3d v2(1,2,3);

		std::cout << "m =" << std::endl << m << std::endl;
		std::cout << "m * v2 =" << std::endl << m * v2 << std::endl;

		//Accessing matrices:
		Eigen::MatrixXd A2 = Eigen::MatrixXd::Random(7, 9);
		std::cout << "The fourth row and 7th column element is " << A2(3, 6) << std::endl;

		Eigen::MatrixXd B2 = A2.block(1, 2, 3, 3);
		std::cout << "Take sub-matrix whose upper left corner is A(1, 2)" 
		<< std::endl << B2 << std::endl;

		Eigen::VectorXd a2 = A2.col(1); // take the second column of A
		Eigen::VectorXd b2 = B2.row(0); // take the first row of B2

		Eigen::VectorXd c2 = a2.head(3);// take the first three elements of a2
		Eigen::VectorXd d2 = b2.tail(2);// take the last two elements of b2

    return 0;
}
