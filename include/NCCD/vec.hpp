//
// Created by Chenfei Wang on 7/10/2023.
//

#ifndef TCCPPLIB_VEC_HPP
#define TCCPPLIB_VEC_HPP

#include <cmath>
#include <cfloat>
#include "TcEigen/Dense"

namespace NCCD
{

    // CCD vec3.h part rewrite

    using real = double;
    constexpr real EPS = DBL_EPSILON;
    constexpr real REAL_MAX = DBL_MAX;
    constexpr auto SQRT = sqrt;
    constexpr auto FABS = fabs;
    constexpr auto FMAX = [](real x, real y)->real {return (x>y)?x:y;};
    constexpr auto FMIN = [](real x, real y)->real {return (x<y)?x:y;};
    constexpr real ONE = 1;
    constexpr real ZERO = 0;
    using vec3_t = Eigen::Vector<real, 3>;
    const auto vec3_origin = vec3_t::Zero();
    vec3_t ccd_points_on_sphere;
    constexpr auto IsZero = [](real val)->int {return FABS(val) < EPS;};
    constexpr auto Sign = [](real val)->int
    {
        if(IsZero(val)) return 0;
        else if (val < ZERO) return -1;
        else return 1;
    };
    constexpr auto Eq = [](real _a, real _b)->int
    {
        real ab;
        real a, b;
        ab = FABS(_a - _b);
        if (FABS(ab) < EPS)
            return 1;

        a = FABS(_a);
        b = FABS(_b);
        if (b > a){
            return ab < EPS * b;
        }else{
            return ab < EPS * a;
        }
    };
    constexpr auto Vec3X = [](const vec3_t & v)->real {return v(0);};
    constexpr auto Vec3Y = [](const vec3_t & v)->real {return v(1);};
    constexpr auto Vec3Z = [](const vec3_t & v)->real {return v(2);};
    constexpr auto Vec3Eq = [](const vec3_t & a, const vec3_t & b)->int
    {
        return Eq(Vec3X(a), Vec3X(b))
               && Eq(Vec3Y(a), Vec3Y(b))
               && Eq(Vec3Z(a), Vec3Z(b));
    };
    constexpr auto Vec3Len2 = [](const vec3_t & v)->real {return v.norm();};
    constexpr auto Vec3Dist2 = [](const vec3_t & a, const vec3_t & b)->real {return (a - b).norm();};
    constexpr auto Vec3Set = [](vec3_t & a, real x, real y, real z)->void {a << x,y,z;};


    // CCD vec3.c rewrite
    









}




#endif //TCCPPLIB_VEC_HPP
