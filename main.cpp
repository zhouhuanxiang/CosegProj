#include "tests/tests.h"

int main()
{
	test_sens();
//	test_plane();
	return 0;
}

//#include <iostream>  
//#include <Eigen/Dense>  
//
////using Eigen::MatrixXd;  
//using namespace Eigen;
//using namespace Eigen::internal;
//using namespace Eigen::Architecture;
//
//using namespace std;
//
//
//int main()
//{
//
//#pragma region one_d_object  
//
//	cout << "*******************1D-object****************" << endl;
//
//	Vector4d v1;
//	v1 << 1, 2, 3, 4;
//	cout << "v1=\n" << v1 << endl;
//
//	v1(0) = 22;
//	cout << "v1=\n" << v1 << endl;
//
//	VectorXd v2(3);
//	v2 << 1, 2, 3;
//	cout << "v2=\n" << v2 << endl;
//
//	Array4i v3;
//	v3 << 1, 2, 3, 4;
//	cout << "v3=\n" << v3 << endl;
//
//	ArrayXf v4(3);
//	v4 << 1, 2, 3;
//	cout << "v4=\n" << v4 << endl;
//
//
//	MatrixXf m = MatrixXf::Random(3, 2);
//	cout << "Here is the matrix m:" << endl << m << endl;
//	JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);
//	cout << "Its singular values are:" << endl << svd.singularValues() << endl;
//	cout << svd.singularValues()[0];
//	cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
//	cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
//	Vector3f rhs(1, 0, 0);
//	cout << "Now consider this rhs vector:" << endl << rhs << endl;
//	cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(rhs) << endl;
//
//
//	int a;
//	cin >> a;
//
//	return 0;
//
//}
