#ifndef TCCPPLIB_INCLUDE_NCCD_SIMPLEX_HPP_
#define TCCPPLIB_INCLUDE_NCCD_SIMPLEX_HPP_


#include "support.hpp"

#include "array"

namespace NCCD{
	class Simplex
	{
	 public:
		std::array<Support, 4> ps;
		int last; //!< index of last added point

		//ccdSimplexInit 初始化
		Simplex() : last(-1) {};

		//ccdSimplexSize 得到大小
		int Size()
		{
			return last+1;
		}

		//ccdSimplexLast 返回最后一个
		Support Last()
		{
			return ps[last];
		}

		//ccdSimplexPoint 返回指定的一个 只能做右值
		const Support & Point(const int idx) const
		{
			return ps[idx];
		}
		//ccdSimplexPointW 返回指定的一个 可以做左值
		Support & PointW(const int idx)
		{
			return ps[idx];
		}

		//ccdSimplexAdd 添加
		void Add(const Support & v)
		{
			++last;
			ps[last] = v;
		}

		//ccdSimplexSet 给指定位置写
		void Set(size_t pos, const Support & a)
		{
			ps[pos] = a;
		}

		//ccdSimplexSetSize 设置大小
		void SetSize(int size)
		{
			last = size - 1;
		}

		//ccdSimplexSwap 交换两个位置的内容
		void Swap(size_t pos1, size_t pos2)
		{
			Support supp;
			supp = ps[pos1];
			ps[pos1] = ps[pos2];
			ps[pos2] = supp;
		}


		// the following is a part of GJK
		int doSimplex2(vec3_t & dir)
		{
			Support A, B;
			vec3_t AB, AO, tmp;
			real dot;

			// get last added as A
			A = Last();
			// get the other point
			B = Point(0);
			// compute AB oriented segment
			AB = B.v - A.v;
			// compute AO vector
			AO = -A.v;

			// dot product AB . AO
			dot = AB.dot(AO);

			// check if origin doesn't lie on AB segment
			tmp = AB.cross(AO);

			if (IsZero(Vec3Len2(tmp)) && dot > ZERO){
				return 1;
			}

			// check if origin is in area where AB segment is
			if (IsZero(dot) || dot < ZERO){
				// origin is in outside are of A
				Set(0, A);
				SetSize(1);
				dir = AO;
			}else{
				// origin is in area where AB segment is

				// keep simplex untouched and set direction to
				// AB x AO x AB
				dir = AB.cross(AO).cross(AB);
			}

			return 0;

		}

		int doSimplex3(vec3_t & dir)
		{
			Support A, B, C;
			vec3_t AO,AB,AC,ABC,tmp;
			real dot, dist;

			// get last added as A
			A = Last();
			// get the other points
			B = Point(1);
			C = Point(0);

			// check touching contact
			dist = Vec3PointTriDist2(vec3_origin, A.v, B.v, C.v, nullptr);
			if (IsZero(dist)){
				return 1;
			}

			// check if triangle is really triangle (has area > 0)
			// if not simplex can't be expanded and thus no itersection is found
			if (Vec3Eq(A.v, B.v) || Vec3Eq(A.v, C.v)){
				return -1;
			}

			// compute AO vector
			AO = -A.v;

			// compute AB and AC segments and ABC vector (perpendircular to triangle)
			AB = B.v - A.v;
			AC = C.v - A.v;
			ABC = AB.cross(AC);

			tmp = ABC.cross(AC);
			dot = tmp.dot(AO);

			if (IsZero(dot) || dot > ZERO){
				dot = AC.dot(AO);
				if (IsZero(dot) || dot > ZERO){
					// C is already in place
					Set(1, A);
					SetSize(2);
					dir = AC.cross(AO).cross(AC);
				}else{
					ccd_do_simplex3_45:
					dot = AB.dot(AO);
					if (IsZero(dot) || dot > ZERO){
						Set(0, B);
						Set(1, A);
						SetSize(2);
						dir = AB.cross(AO).cross(AB);
					}else{
						Set(0, A);
						SetSize(1);
						dir = AO;
					}
				}
			}else{
				ABC = tmp.cross(AB);
				dot = tmp.dot(AO);
				if (IsZero(dot) || dot > ZERO){
					goto ccd_do_simplex3_45;
				}else{
					dot = ABC.dot(AO);
					if (IsZero(dot) || dot > ZERO){
						dir = ABC;
					}else{
						Support Ctmp;
						Ctmp = C;
						Set(0, B);
						Set(1, Ctmp);
						dir = -ABC;
					}
				}
			}
			return 0;
		}

		int doSimplex4(vec3_t & dir)
		{
			Support A, B, C, D;
			vec3_t AO, AB, AC, AD, ABC, ACD, ADB;
			int B_on_ACD, C_on_ADB, D_on_ABC;
			int AB_O, AC_O, AD_O;
			real dist;

			// get last added as A
			A = Last();
			// get the other points
			B = Point(2);
			C = Point(1);
			D = Point(0);

			// check if tetrahedron is really tetrahedron (has volume > 0)
			// if it is not simplex can't be expanded and thus no intersection is
			// found
			dist = Vec3PointTriDist2(A.v, B.v, C.v, D.v, nullptr);
			if (IsZero(dist)){
				return -1;
			}

			// check if origin lies on some of tetrahedron's face - if so objects
			// intersect
			dist = Vec3PointTriDist2(vec3_origin, A.v, B.v, C.v, nullptr);
			if (IsZero(dist))
				return 1;
			dist = Vec3PointTriDist2(vec3_origin, A.v, C.v, D.v, nullptr);
			if (IsZero(dist))
				return 1;
			dist = Vec3PointTriDist2(vec3_origin, A.v, B.v, D.v, nullptr);
			if (IsZero(dist))
				return 1;
			dist = Vec3PointTriDist2(vec3_origin, B.v, C.v, D.v, nullptr);
			if (IsZero(dist))
				return 1;

			// compute AO, AB, AC, AD segments and ABC, ACD, ADB normal vectors
			AO = -A.v;
			AB = B.v - A.v;
			AC = C.v - A.v;
			AD = D.v - A.v;
			ABC = AB.cross(AC);
			ACD = AC.cross(AD);
			ADB = AD.cross(AB);

			// side (positive or negative) of B, C, D relative to planes ACD, ADB
			// and ABC respectively
			B_on_ACD = Sign( ACD.dot(AB));
			C_on_ADB = Sign( ADB.dot(AC));
			D_on_ABC = Sign( ABC.dot(AD));

			// whether origin is on same side of ACD, ADB, ABC as B, C, D
			// respectively
			AB_O = Sign(ACD.dot(AO)) == B_on_ACD;
			AC_O = Sign(ADB.dot(AO)) == C_on_ADB;
			AD_O = Sign(ABC.dot(AO)) == D_on_ABC;

			if (AB_O && AC_O && AD_O){
				// origin is in tetrahedron
				return 1;

			// rearrange simplex to triangle and call doSimplex3()
			}else if (!AB_O){
				// B is farthest from the origin among all of the tetrahedron's
				// points, so remove it from the list and go on with the triangle
				// case

				// D and C are in place
				Set(2, A);
				SetSize(3);
			}else if (!AC_O){
				// C is farthest
				Set(1, D);
				Set(0, B);
				Set(2, A);
				SetSize(3);
			}else{ // (!AD_O)
				Set(0, C);
				Set(1, B);
				Set(2, A);
				SetSize(3);
			}
			return doSimplex3(dir);
		}

		int doSimplex(vec3_t & dir)
		{
			if (Size() == 2){
				// simplex contains segment only one segment
				return doSimplex2(dir);
			}else if (Size() == 3){
				// simplex contains triangle
				return doSimplex3(dir);
			}else{ // ccdSimplexSize(simplex) == 4
				// tetrahedron - this is the only shape which can encapsule origin
				// so doSimplex4() also contains test on it
				return doSimplex4(dir);
			}
		}
	};

}





#endif //TCCPPLIB_INCLUDE_NCCD_SIMPLEX_HPP_
