#include "doctest/doctest.h"
#include "PPX/ppx.h"
#include "CHEN/division.h"

TEST_CASE("expr test"){
	using namespace ppx;
	MatrixS<3, 4> m;
	ones(m);
	m = m.eye();

	m.sub<2, 2, false>(0, 0) = {2, 3, 4};
	m.sub<3, 1, false>(0, 3) = MatrixS<3, 1>::zeros() - MatrixS<3, 1>::eye();

	MatrixS<3, 4> m2 = {2, 3, 0, 4, 0, 0, 0, 0, 1, -1, 0, 0};
	CHECK(m == m2);

}

TEST_CASE("Robot FK")
{
	using namespace ppx;
	kinematics<7> SC;
	SE3 F7{-1.0, 0.0, 0.0, 0.0,
		   0.0, -1.0, 0.0, 0.0,
		   0.0, 0.0, 1.0, 0.0,
		   0.33, 0.0, -0.3525, 1.0};
	SC.setJoint<0>({"R1", se3{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, SE3()});
	SC.setJoint<1>({"R2", se3{0.0, -1.0, 0.0, -0.241, 0.0, 0.0}, SE3{}});
	SC.setJoint<2>({"R3", se3{0.0, -1.0, 0.0, -0.521, 0.0, 0.0}, SE3()});
	SC.setJoint<3>({"R4", se3{0.0, 0.0, 1.0, 0.0, -0.33, 0.0}, SE3()});
	SC.setJoint<4>({"R5", se3{1.0, 0.0, 0.0, 0.0, -0.3525, 0.0}, SE3()});
	SC.setJoint<5>({"R6", se3{0.0, 0.0, 1.0, 0.0, -0.33, 0.0}, SE3()});
	SC.setJoint<6>({"R7", se3{-1.0, 0.0, 0.0, 0.0, 0.3525, 0.0}, F7});

	auto M = SC.forwardSpace("R7", {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7});

	SE3 M_matlab{-0.288510278793969, -0.810658209754245, -0.509504745795631, 0.0,
				 0.577956031805100, -0.571700459168886, 0.582344752089506, 0.0,
				 -0.763366651308747, -0.126458894286361, 0.633466260921244, 0.0,
				 0.263125324010681, 0.026400593101990, -0.209335552377633, 1.0};

	CHECK(M == M_matlab);

}