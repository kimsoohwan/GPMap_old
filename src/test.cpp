#define BOOST_TEST_MODULE Simple testcases
// STL
#include <iostream>
using namespace std;

// boost unit test
#include <boost/test/unit_test.hpp>

#include "common\common.hpp"
using namespace GPMap;

const float EPSILON = 1E-6f;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_SUITE(common_test)

// TEST1: invert 3x3 matrix
BOOST_AUTO_TEST_CASE(invert_3x3_matrix) {

	// Random Matrix
	Eigen::Matrix3f A = Eigen::Matrix3f::Random();
	Eigen::Matrix3f invA = A.inverse();
	Eigen::Matrix3f A_copy = A;
	//cout << "A = " << endl << A << endl << endl;
	//cout << "A*A^{-1} = " << endl << A*invA << endl << endl;
	//cout << A.determinant() << std::endl;

	// check
	if(invert3x3Matrix(A))
	{
		BOOST_CHECK_EQUAL(((A_copy*A - Eigen::Matrix3f::Identity()).array().abs() < EPSILON).all(), true);
		//cout << "A*A^{-1} = " << endl << A_copy*A << endl << endl;
		//cout << "invA (eigen) = " << endl << invA << endl << endl;
		//cout << "invA (mine) = " << endl << A << endl << endl;
	}
	else
		cout << "singular!" << endl;
}

BOOST_AUTO_TEST_SUITE_END()