//
// Created by Chenfei Wang on 7/10/2023.
//

#ifndef TCCPPLIB_VEC_HPP
#define TCCPPLIB_VEC_HPP

#include <cmath>
#include <cfloat>
#include "TcEigen/Dense"
#include <array>

namespace NCCD
{
	// CCD vec3.h part rewrite

	using real = double;
	constexpr real EPS = DBL_EPSILON;
	constexpr real REAL_MAX = DBL_MAX;
	const auto SQRT = [](real x) -> real
	{ return sqrt(x); };
	const auto FABS = [](real x) -> real
	{ return fabs(x); };
	const auto FMAX = [](real x, real y) -> real
	{ return (x > y) ? x : y; };
	const auto FMIN = [](real x, real y) -> real
	{ return (x < y) ? x : y; };
	constexpr real ONE = 1;
	constexpr real ZERO = 0;
	using vec3_t = Eigen::Vector<real, 3>;
	const auto vec3_origin = vec3_t::Zero();
	const auto IsZero = [](real val) -> int
	{ return FABS(val) < EPS; };
	const auto Sign = [](real val) -> int
	{
	  if (IsZero(val)) return 0;
	  else if (val < ZERO) return -1;
	  else return 1;
	};
	const auto Eq = [](real _a, real _b) -> int
	{
	  real ab;
	  real a, b;
	  ab = FABS(_a - _b);
	  if (FABS(ab) < EPS)
		  return 1;

	  a = FABS(_a);
	  b = FABS(_b);
	  if (b > a)
	  {
		  return ab < EPS * b;
	  }
	  else
	  {
		  return ab < EPS * a;
	  }
	};
	const auto Vec3X = [](const vec3_t& v) -> real
	{ return v(0); };
	const auto Vec3Y = [](const vec3_t& v) -> real
	{ return v(1); };
	const auto Vec3Z = [](const vec3_t& v) -> real
	{ return v(2); };
	const auto Vec3Eq = [](const vec3_t& a, const vec3_t& b) -> int
	{
	  return Eq(Vec3X(a), Vec3X(b))
		  && Eq(Vec3Y(a), Vec3Y(b))
		  && Eq(Vec3Z(a), Vec3Z(b));
	};
	const auto Vec3Len2 = [](const vec3_t& v) -> real
	{ return v.dot(v); };
	const auto Vec3Dist2 = [](const vec3_t& a, const vec3_t& b) -> real
	{ return Vec3Len2(a - b); };
	const auto Vec3Set = [](vec3_t& a, real x, real y, real z) -> void
	{ a << x, y, z; };

	inline real __ccdVec3PointSegmentDist2(const vec3_t& P,
		const vec3_t& x0,
		const vec3_t& b,
		vec3_t* witness)
	{
		real dist, t;
		vec3_t d, a;

		//direction of segment
		d = b - x0;

		// precompute vector from P to x0
		a = x0 - P;

		t = -ONE * a.dot(d);
		t /= Vec3Len2(d);

		if (t < ZERO || IsZero(t))
		{
			dist = Vec3Dist2(x0, P);
			if (witness)
				*witness = x0;
		}
		else if (t > ONE || Eq(t, ONE))
		{
			dist = Vec3Dist2(b, P);
			if (witness)
				*witness = b;
		}
		else
		{
			if (witness)
			{
				*witness = d;
				*witness *= t;
				*witness += x0;
				dist = Vec3Dist2(*witness, P);
			}
			else
			{
				d *= t;
				d += a;
				dist = Vec3Len2(d);
			}
		}

		return dist;
	}

	inline real Vec3PointSegmentDist2(const vec3_t& p,
		const vec3_t& a,
		const vec3_t& b,
		vec3_t* witness)
	{
		return __ccdVec3PointSegmentDist2(p, a, b, witness);
	}

	inline real Vec3PointTriDist2(const vec3_t& P,
		const vec3_t& x0,
		const vec3_t& B,
		const vec3_t& C,
		vec3_t* witness)
	{
		vec3_t d1, d2, a;
		real u, v, w, p, q, r, d;
		real s, t, dist, dist2;
		vec3_t witness2;

		d1 = B - x0;
		d2 = C - x0;
		a = x0 - P;

		u = a.dot(a);
		v = d1.dot(d1);
		w = d2.dot(d2);
		p = a.dot(d1);
		q = a.dot(d2);
		r = d1.dot(d2);

		d = w * v - r * r;
		if (IsZero(d))
		{
			s = t = -1;
		}
		else
		{
			s = (q * r - w * p) / d;
			t = (-s * r - q) / w;
		}

		if ((IsZero(s) || s > ZERO)
			&& (Eq(s, ONE) || s < ONE)
			&& (IsZero(t) || t > ZERO)
			&& (Eq(t, ONE) || t < ONE)
			&& (Eq(t + s, ONE) || t + s < ONE))
		{

			if (witness)
			{
				d1 *= s;
				d2 *= t;
				*witness = x0;
				*witness += d1;
				*witness += d2;
				dist = Vec3Dist2(*witness, P);
			}
			else
			{
				dist = s * s * v;
				dist += t * t * w;
				dist += real(2.) * s * t * r;
				dist += real(2.) * s * p;
				dist += real(2.) * t * q;
				dist += u;
			}

		}
		else
		{
			dist = __ccdVec3PointSegmentDist2(P, x0, B, witness);
			dist2 = __ccdVec3PointSegmentDist2(P, x0, C, &witness2);
			if (dist2 < dist)
			{
				dist = dist2;
				if (witness)
					*witness = witness2;
			}
			dist2 = __ccdVec3PointSegmentDist2(P, B, C, &witness2);
			if (dist2 < dist)
			{
				dist = dist2;
				if (witness)
					*witness = witness2;
			}

		}
		return dist;
	}


	static std::array<vec3_t , 42> points_on_sphere = {
		vec3_t (real( 0.000000), real(-0.000000), real(-1.000000)),
		vec3_t (real( 0.723608), real(-0.525725), real(-0.447219)),
		vec3_t (real(-0.276388), real(-0.850649), real(-0.447219)),
		vec3_t (real(-0.894426), real(-0.000000), real(-0.447216)),
		vec3_t (real(-0.276388), real( 0.850649), real(-0.447220)),
		vec3_t (real( 0.723608), real( 0.525725), real(-0.447219)),
		vec3_t (real( 0.276388), real(-0.850649), real( 0.447220)),
		vec3_t (real(-0.723608), real(-0.525725), real( 0.447219)),
		vec3_t (real(-0.723608), real( 0.525725), real( 0.447219)),
		vec3_t (real( 0.276388), real( 0.850649), real( 0.447219)),
		vec3_t (real( 0.894426), real( 0.000000), real( 0.447216)),
		vec3_t (real(-0.000000), real( 0.000000), real( 1.000000)),
		vec3_t (real( 0.425323), real(-0.309011), real(-0.850654)),
		vec3_t (real(-0.162456), real(-0.499995), real(-0.850654)),
		vec3_t (real( 0.262869), real(-0.809012), real(-0.525738)),
		vec3_t (real( 0.425323), real( 0.309011), real(-0.850654)),
		vec3_t (real( 0.850648), real(-0.000000), real(-0.525736)),
		vec3_t (real(-0.525730), real(-0.000000), real(-0.850652)),
		vec3_t (real(-0.688190), real(-0.499997), real(-0.525736)),
		vec3_t (real(-0.162456), real( 0.499995), real(-0.850654)),
		vec3_t (real(-0.688190), real( 0.499997), real(-0.525736)),
		vec3_t (real( 0.262869), real( 0.809012), real(-0.525738)),
		vec3_t (real( 0.951058), real( 0.309013), real( 0.000000)),
		vec3_t (real( 0.951058), real(-0.309013), real( 0.000000)),
		vec3_t (real( 0.587786), real(-0.809017), real( 0.000000)),
		vec3_t (real( 0.000000), real(-1.000000), real( 0.000000)),
		vec3_t (real(-0.587786), real(-0.809017), real( 0.000000)),
		vec3_t (real(-0.951058), real(-0.309013), real(-0.000000)),
		vec3_t (real(-0.951058), real( 0.309013), real(-0.000000)),
		vec3_t (real(-0.587786), real( 0.809017), real(-0.000000)),
		vec3_t (real(-0.000000), real( 1.000000), real(-0.000000)),
		vec3_t (real( 0.587786), real( 0.809017), real(-0.000000)),
		vec3_t (real( 0.688190), real(-0.499997), real( 0.525736)),
		vec3_t (real(-0.262869), real(-0.809012), real( 0.525738)),
		vec3_t (real(-0.850648), real( 0.000000), real( 0.525736)),
		vec3_t (real(-0.262869), real( 0.809012), real( 0.525738)),
		vec3_t (real( 0.688190), real( 0.499997), real( 0.525736)),
		vec3_t (real( 0.525730), real( 0.000000), real( 0.850652)),
		vec3_t (real( 0.162456), real(-0.499995), real( 0.850654)),
		vec3_t (real(-0.425323), real(-0.309011), real( 0.850654)),
		vec3_t (real(-0.425323), real( 0.309011), real( 0.850654)),
		vec3_t (real( 0.162456), real( 0.499995), real( 0.850654))};

}


#endif //TCCPPLIB_VEC_HPP
