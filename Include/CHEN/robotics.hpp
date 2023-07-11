//
// Created by Administrator on 2023/7/6.
//

#ifndef TCCPPLIB_INCLUDE_CHEN_ROBOTICS_HPP_
#define TCCPPLIB_INCLUDE_CHEN_ROBOTICS_HPP_

#include "calc.hpp"
#include <string>
#include <complex>
#include <memory>
#include <array>
#include <vector>
#include <cassert>
#include <limits>
#include <cmath>
#include <algorithm>

namespace robot
{
	using namespace robot_basic_calc;
	struct joint
	{
		std::string name{"default"};
		std::pair<double, double> range{-robot_basic_calc::MAX_SP, robot_basic_calc::MAX_SP};
		Vec6 screw;
		Mat4 se3;
		Mat4 M;
		joint() = default;
		joint(const std::string &name_, const Vec6 &screw_, const Mat4 &M_)
			: name(name_), screw(screw_), M(M_)
		{
			se3 = VecTose3(screw_);
		}
	};

	template <size_t N>
	class kinematics
	{
	 private:
		template <size_t L, typename RT=void>
		using idx_available_t = typename std::enable_if<(L < N), RT>::type;
		std::array<joint, N> JList;

	 public:
		using Q = Eigen::Vector<double, N>;

		template<size_t L>
		idx_available_t<L, joint &> getJoint()
		{
			return JList[L];
		}

		template <size_t L>
		idx_available_t<L, const joint &> getJoint() const
		{
			return JList[L];
		}

		template <size_t L>
		idx_available_t<L> setJoint(const joint &j)
		{
			JList[L] = j;
		}

		Mat4 forwardSpace(const std::string &jointName, const Q &jointAngle) const
		{
			auto effector_idx = std::find_if(JList.begin(), JList.end(), [&jointName](const joint &elem)
			{ return jointName == elem.name; });
			if (effector_idx == JList.end())
			{
				return O4;
			}
			Mat4 effector_pose = effector_idx->M;
			for (int i = (int)std::distance(JList.begin(), effector_idx); i > -1; i--)
			{
				effector_pose = MatrixExp6(JList[i].se3 * jointAngle[i]) * effector_pose;
			}
			return effector_pose;
		}

		Mat4 forwardSpace(const Q &jointAngle) const
		{
			Mat4 effector_pose = JList.back().M;
			for (int i = N - 1; i > -1; i--)
			{
				effector_pose = MatrixExp6(JList[i].se3 * jointAngle[i]) * effector_pose;
			}
			return effector_pose;
		}



		Eigen::Matrix<double, 6, N> jacobiSpace(const Q &jointAngle)
		{
			Eigen::Matrix<double, 6, N> Slist;
			for (int i = 0; i < N; ++i)
			{
				Slist.col(i) = JList[i].screw;
			}
			return JacobianSpace(Slist, jointAngle);
		}


		template <size_t L>
		idx_available_t<L, Eigen::Matrix<double, 6, L>> jacobiSpace(const std::array<std::string, L> &namelist, const Q &jointAngle)
		{
			Eigen::Matrix<double, 6, L> Js;
			Eigen::Matrix<double, 6, N> JsAll = jacobiSpace(jointAngle);
			for (size_t i = 0; i < L; i++)
			{
				auto iter = std::find_if(JList.begin(), JList.end(), [&namelist, i](const joint &elem)
				{ return namelist[i] == elem.name; });
				if (iter != JList.end())
				{
					auto col_idx = std::distance(JList.begin(), iter);
					Js.col(i) = JsAll.col(col_idx);
				}
			}
			return Js;
		}


	};


}

#endif //TCCPPLIB_INCLUDE_CHEN_ROBOTICS_HPP_
