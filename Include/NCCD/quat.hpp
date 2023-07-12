//
// Created by Administrator on 2023/7/11.
//

#ifndef TCCPPLIB_INCLUDE_NCCD_QUAT_HPP_
#define TCCPPLIB_INCLUDE_NCCD_QUAT_HPP_
#include "vec.hpp"
namespace NCCD
{
	using quat_t = Eigen::Quaterniond;
	const auto QuatLen = [](const quat_t& q) -> real
	{
	  return q.norm();
	};
	const auto QuatLen2 = [](const quat_t& q) -> real
	{
	  return SQUARE(q.norm());
	};
	const auto QuatScale = [](quat_t &q, real k)
	{
	  q.x() *=k;
	  q.y() *=k;
	  q.z() *=k;
	  q.w() *=k;
	};

	inline void QuatSetAngleAxis(quat_t & q, const real angle, const vec3_t & axis)
	{
		real a, x, y, z, n, s;
		a = angle/2;
		x = Vec3X(axis);
		y = Vec3Y(axis);
		z = Vec3Z(axis);
		n = SQRT(x*x + y*y + z*z);

		if (n < EPS){
			q.x() = q.y() = q.z() = ZERO;
			q.w() = ONE;
		}else{
			s = sin(a)/n;

			q.w() = cos(a);
			q.x() = x*s;
			q.y() = y*s;
			q.z() = z*s;
			q.normalize();
		}
	}

	inline void QuatRotVec(vec3_t & v, const quat_t & q)
	{
		real cross1_x, cross1_y, cross1_z, cross2_x, cross2_y, cross2_z;
		real x, y, z, w;
		real vx, vy, vz;

		vx = Vec3X(v);
		vy = Vec3Y(v);
		vz = Vec3Z(v);

		w = q.w();
		x = q.x();
		y = q.y();
		z = q.z();

		cross1_x = y * vz - z * vy + w * vx;
		cross1_y = z * vx - x * vz + w * vy;
		cross1_z = x * vy - y * vx + w * vz;
		cross2_x = y * cross1_z - z * cross1_y;
		cross2_y = z * cross1_x - x * cross1_z;
		cross2_z = x * cross1_y - y * cross1_x;
		Vec3Set(v, vx + 2 * cross2_x, vy + 2 * cross2_y, vz + 2 * cross2_z);
	}


}

#endif //TCCPPLIB_INCLUDE_NCCD_QUAT_HPP_
