
#include <iostream>
#include <random>
#include <future>
#include <fstream>

#include "PPX/ppx.h"
#include "ppxlog.h"

#include "TcEigen/Dense"



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

	PRINT_SINGLE_ELEMENTS(M, "M = ");

	PRINT_SINGLE_ELEMENTS(SC.jacobiSpace({0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7}), "Jacobi = ");
	PRINT_SINGLE_ELEMENTS(SC.jacobiSpace(std::array<std::string, 3>{"R1", "R2", "R3"}, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7}), "Jacobi(3) = ");


}

#include <cmath>
#include <vector>
using namespace Eigen;
using namespace std;

namespace CHEN
{
	// constexpr
	constexpr double M_PI = 3.14159265358979323846;
	constexpr double EPS = 1E-8;
	constexpr double NEAR_ZERO = 1E-5;
	constexpr double EPS_SP = std::numeric_limits<float>::epsilon();
	constexpr double EPS_DP = std::numeric_limits<double>::epsilon();
	constexpr double MAX_SP = std::numeric_limits<float>::max();
	constexpr double MAX_DP = std::numeric_limits<double>::max();

	using vec3 = Eigen::Matrix<double, 3, 1>;
	using vec6 = Eigen::Matrix<double, 6, 1>;
	using so3vec = Eigen::Matrix<double, 3, 1>;
	using se3vec = Eigen::Matrix<double, 6, 1>;
	using so3mat = Eigen::Matrix<double, 3, 3>;
	using se3mat = Eigen::Matrix<double, 4, 4>;
	using SO3 = Eigen::Matrix<double, 3, 3>;
	using SE3 = Eigen::Matrix<double, 4, 4>;

	// 常用矩阵
	const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
	const Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
	const Eigen::Matrix3d O3 = Eigen::Matrix3d::Zero();
	const Eigen::Matrix4d O4 = Eigen::Matrix4d::Zero();


	inline bool NearZero(const double val)
	{
		return std::abs(val) < NEAR_ZERO;
	}

	Eigen::MatrixXd Normalize(Eigen::MatrixXd V)
	{
		V.normalize();
		return V;
	}

	inline so3mat VecToso3(const so3vec omg)
	{
		so3mat m_ret;
		m_ret << 0, -omg(2), omg(1),
				omg(2), 0, -omg(0),
				-omg(1), omg(0), 0;
		return m_ret;
	}

	inline so3vec so3ToVec(const so3mat mat)
	{
		so3vec v_ret;
		v_ret << mat(2, 1), mat(0, 2), mat(1, 0);
		return v_ret;
	}

	inline Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3) {
		Eigen::Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}

	SO3 MatrixExp3(const so3mat& mat)
	{
		Vector3d omgtheta = so3ToVec(mat);
		if (NearZero(mat.norm()))
		{
			return Eigen::Matrix3d::Identity();
		}
		double theta = (AxisAng3(omgtheta))(3);
		so3mat so3 = mat * (1 / theta);
		return I3 + std::sin(theta) * so3 + ((1 - std::cos(theta)) * (so3 * so3));
	}

	inline SO3 MatrixExp3(const so3vec& vec)
	{
		so3mat mat = VecToso3(vec);
		return MatrixExp3(mat);
	}

	so3mat MatrixLog3(const SO3& R)
	{
		double acosinput = (R.trace() - 1) / 2.0;
		auto m_ret = O3;
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
	}

	inline so3vec MatrixLog3vec(const SO3& R)
	{
		return so3ToVec(MatrixLog3(R));
	}

	SE3 RpToTrans(const SO3& R, const vec3& p)
	{
		SE3 m_ret;
		m_ret << R, p,
			0, 0, 0, 1;
		return m_ret;
	}

	std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T) {
		std::vector<Eigen::MatrixXd> Rp_ret;
		Eigen::Matrix3d R_ret;
		// Get top left 3x3 corner
		R_ret = T.block<3, 3>(0, 0);

		Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

		Rp_ret.push_back(R_ret);
		Rp_ret.push_back(p_ret);

		return Rp_ret;
	}






}




#include "CHEN/calc.hpp"
int main()
{
	using namespace robot;
	using namespace Eigen;
	Vector3d v = {0.1,0.2,0.3};
	auto R = MatrixExp3(v);
	cout << R << endl;

    Mat3 R2;
    R2 = R;
    R2(0,0) = -R(0,0);
    R2(0,1) = -R(0,1);
    R2(0,2) = -R(0,2);

    cout << "distant " << DistanceToSO3(R2) << endl;
    cout << isInf(DistanceToSO3(R2)) << endl;



	return 0;
}