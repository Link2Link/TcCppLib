
#include <iostream>
#include <random>
#include <future>
#include <fstream>

#include "PPX/ppx.h"
#include "ppxlog.h"



using namespace std;
using namespace ppx;


template <typename T>
inline void PRINT_SINGLE_ELEMENTS(const T &coll, const std::string &optcsrt = "")
{
	std::cout << optcsrt << coll << std::endl;
}

template <typename T>
inline void PRINT_LISTED_ELEMENTS(const T &coll, const std::string &optcsrt = "")
{
	std::cout << optcsrt;
	for (const auto ele : coll)
	{
		std::cout << ele << ' ';
	}
	std::cout << std::endl;
}

void test_expr()
{
	MatrixS<3, 4> m;
	ones(m);
	m = m.eye();


	PRINT_SINGLE_ELEMENTS(m, "m = ");
	// elem 0
	m.sub<2, 2, false>(0, 0) = {2, 3, 4};
	m.sub<3, 1, false>(0, 3) = MatrixS<3, 1>::zeros() - MatrixS<3, 1>::eye();
	PRINT_SINGLE_ELEMENTS(m, "m = ");

	MatrixS<3, 4> m2 = {2, 3, 0, 4, 0, 0, 0, 0, 1, -1, 0, 0};
	PRINT_SINGLE_ELEMENTS(m2, "m2 = ");
	cout << (m == m2) << endl;


}

void test_matrix()
{

}

void test_robotic()
{
	using namespace ppx;
	kinematics<6> UR5;
	SE3 F6{-1.0, 0.0, 0.0, 0.0,
		   0.0, 0.0, 1.0, 0.0,
		   0.0, 1.0, 0.0, 0.0,
		   0.817, 0.191, -0.006, 1.0};
	UR5.setJoint<0>({"R1", se3{0, 0, 1, 0, 0, 0, 0}, SE3()});
	UR5.setJoint<1>({"R2", se3{0.0, 1.0, 0.0, -0.089, 0.0, 0.0}, SE3{}});
	UR5.setJoint<2>({"R3", se3{0.0, 1.0, 0.0, -0.089, 0.0, 0.425}, SE3{}});
	UR5.setJoint<3>({"R4", se3{0.0, 1.0, 0.0, -0.089, 0.0, 0.817}, SE3{}});
	UR5.setJoint<4>({"R5", se3{0.0, 0.0, -1.0, -0.109, 0.817, 0.0}, SE3{}});
	UR5.setJoint<5>({"R6", se3{0.0, 1.0, 0.0, 0.006, 0.0, 0.817}, F6});
	PRINT_SINGLE_ELEMENTS(UR5.forwardSpace("R6", {0.0, -0.5 * PI, 0.0, 0.0, 0.5 * PI, 0.0}), "Forward(R6) = ");
	PRINT_SINGLE_ELEMENTS(UR5.jacobiSpace({0.0, -0.5 * PI, 0.0, 0.0, 0.5 * PI, 0.0}), "Jacobi = ");
	PRINT_SINGLE_ELEMENTS(UR5.jacobiSpace(std::array<std::string, 3>{"R1", "R2", "R3"}, {0.0, -0.5 * PI, 0.0, 0.0, 0.5 * PI, 0.0}), "Jacobi(3) = ");
	SE3 TargetPose{0.0, 1.0, 0.0, 0.0,
				   -1.0, 0.0, 0.0, 0.0,
				   0.0, 0.0, 1.0, 0.0,
				   0.095, 0.109, 0.988, 1.0};
	PRINT_SINGLE_ELEMENTS(UR5.inverseSpace(TargetPose, {0.0, -1.5, 0.0, 0.0, 1.5, 0.0}), "IKSpace = ");

}



int main()
{
	test_expr();
	test_matrix();
	test_robotic();


	return 0;
}