
#pragma once
#ifndef TCCPPLIB_VEC3_HPP
#define TCCPPLIB_VEC3_HPP


#include <cmath>
#include <cfloat>
#include <cstdlib>
#include "basic.hpp"

namespace CCD {
# define CCD_FMIN(x, y) ((x) < (y) ? (x) : (y))
    typedef double ccd_real_t;

    //# define CCD_EPS 1E-10
# define CCD_EPS DBL_EPSILON

# define CCD_REAL_MAX DBL_MAX

# define CCD_REAL(x) (x)       /*!< form a constant */
# define CCD_SQRT(x) (sqrt(x)) /*!< square root */
# define CCD_FABS(x) (fabs(x)) /*!< absolute value */
# define CCD_FMAX(x, y) (fmax((x), (y))) /*!< maximum of two floats */

#define CCD_ONE CCD_REAL(1.)
#define CCD_ZERO CCD_REAL(0.)


    struct _ccd_vec3_t {
        ccd_real_t v[3];
    };
    typedef struct _ccd_vec3_t ccd_vec3_t;


    /**
     * Holds origin (0,0,0) - this variable is meant to be read-only!
     */
    CCD_EXPORT extern ccd_vec3_t *ccd_vec3_origin;

    /**
     * Array of points uniformly distributed on unit sphere.
     */
    CCD_EXPORT extern ccd_vec3_t *ccd_points_on_sphere;
    CCD_EXPORT extern size_t ccd_points_on_sphere_len;

    /** Returns sign of value. */
    _ccd_inline int ccdSign(ccd_real_t val);
    /** Returns true if val is zero. **/
    _ccd_inline int ccdIsZero(ccd_real_t val);
    /** Returns true if a and b equal. **/
    _ccd_inline int ccdEq(ccd_real_t a, ccd_real_t b);


#define CCD_VEC3_STATIC(x, y, z) \
    { { (x), (y), (z) } }

#define CCD_VEC3(name, x, y, z) \
    ccd_vec3_t name = CCD_VEC3_STATIC((x), (y), (z))

    _ccd_inline ccd_real_t ccdVec3X(const ccd_vec3_t *v);

    _ccd_inline ccd_real_t ccdVec3Y(const ccd_vec3_t *v);

    _ccd_inline ccd_real_t ccdVec3Z(const ccd_vec3_t *v);

    /**
     * Returns true if a and b equal.
     */
    _ccd_inline int ccdVec3Eq(const ccd_vec3_t *a, const ccd_vec3_t *b);

    /**
     * Returns squared length of vector.
     */
    _ccd_inline ccd_real_t ccdVec3Len2(const ccd_vec3_t *v);

    /**
     * Returns distance between a and b.
     */
    _ccd_inline ccd_real_t ccdVec3Dist2(const ccd_vec3_t *a, const ccd_vec3_t *b);


    _ccd_inline void ccdVec3Set(ccd_vec3_t *v, ccd_real_t x, ccd_real_t y, ccd_real_t z);

    /**
     * v = w
     */
    _ccd_inline void ccdVec3Copy(ccd_vec3_t *v, const ccd_vec3_t *w);

    /**
     * Substracts coordinates of vector w from vector v. v = v - w
     */
    _ccd_inline void ccdVec3Sub(ccd_vec3_t *v, const ccd_vec3_t *w);

    /**
     * Adds coordinates of vector w to vector v. v = v + w
     */
    _ccd_inline void ccdVec3Add(ccd_vec3_t *v, const ccd_vec3_t *w);

    /**
     * d = v - w
     */
    _ccd_inline void ccdVec3Sub2(ccd_vec3_t *d, const ccd_vec3_t *v, const ccd_vec3_t *w);

    /**
     * d = d * k;
     */
    _ccd_inline void ccdVec3Scale(ccd_vec3_t *d, ccd_real_t k);

    /**
     * Normalizes given vector to unit length.
     */
    _ccd_inline void ccdVec3Normalize(ccd_vec3_t *d);


    /**
     * Dot product of two vectors.
     */
    _ccd_inline ccd_real_t ccdVec3Dot(const ccd_vec3_t *a, const ccd_vec3_t *b);

    /**
     * Cross product: d = a x b.
     */
    _ccd_inline void ccdVec3Cross(ccd_vec3_t *d, const ccd_vec3_t *a, const ccd_vec3_t *b);


    /**
     * Returns distance^2 of point P to segment ab.
     * If witness is non-NULL it is filled with coordinates of point from which
     * was computed distance to point P.
     */
    CCD_EXPORT ccd_real_t ccdVec3PointSegmentDist2(const ccd_vec3_t *P,
                                                   const ccd_vec3_t *a,
                                                   const ccd_vec3_t *b,
                                                   ccd_vec3_t *witness);

    /**
     * Returns distance^2 of point P from triangle formed by triplet a, b, c.
     * If witness vector is provided it is filled with coordinates of point
     * from which was computed distance to point P.
     */
    CCD_EXPORT ccd_real_t ccdVec3PointTriDist2(const ccd_vec3_t *P,
                                               const ccd_vec3_t *a,
                                               const ccd_vec3_t *b,
                                               const ccd_vec3_t *c,
                                               ccd_vec3_t *witness);


    /**** INLINES ****/
    _ccd_inline int ccdSign(ccd_real_t val) {
        if (ccdIsZero(val)) {
            return 0;
        } else if (val < CCD_ZERO) {
            return -1;
        }
        return 1;
    }

    _ccd_inline int ccdIsZero(ccd_real_t val) {
        return CCD_FABS(val) < CCD_EPS;
    }

    _ccd_inline int ccdEq(ccd_real_t _a, ccd_real_t _b) {
        ccd_real_t ab;
        ccd_real_t a, b;

        ab = CCD_FABS(_a - _b);
        if (CCD_FABS(ab) < CCD_EPS)
            return 1;

        a = CCD_FABS(_a);
        b = CCD_FABS(_b);
        if (b > a) {
            return ab < CCD_EPS * b;
        } else {
            return ab < CCD_EPS * a;
        }
    }


    _ccd_inline ccd_real_t ccdVec3X(const ccd_vec3_t *v) {
        return v->v[0];
    }

    _ccd_inline ccd_real_t ccdVec3Y(const ccd_vec3_t *v) {
        return v->v[1];
    }

    _ccd_inline ccd_real_t ccdVec3Z(const ccd_vec3_t *v) {
        return v->v[2];
    }

    _ccd_inline int ccdVec3Eq(const ccd_vec3_t *a, const ccd_vec3_t *b) {
        return ccdEq(ccdVec3X(a), ccdVec3X(b))
               && ccdEq(ccdVec3Y(a), ccdVec3Y(b))
               && ccdEq(ccdVec3Z(a), ccdVec3Z(b));
    }

    _ccd_inline ccd_real_t ccdVec3Len2(const ccd_vec3_t *v) {
        return ccdVec3Dot(v, v);
    }

    _ccd_inline ccd_real_t ccdVec3Dist2(const ccd_vec3_t *a, const ccd_vec3_t *b) {
        ccd_vec3_t ab;
        ccdVec3Sub2(&ab, a, b);
        return ccdVec3Len2(&ab);
    }

    _ccd_inline void ccdVec3Set(ccd_vec3_t *v, ccd_real_t x, ccd_real_t y, ccd_real_t z) {
        v->v[0] = x;
        v->v[1] = y;
        v->v[2] = z;
    }

    _ccd_inline void ccdVec3Copy(ccd_vec3_t *v, const ccd_vec3_t *w) {
        *v = *w;
    }

    _ccd_inline void ccdVec3Sub(ccd_vec3_t *v, const ccd_vec3_t *w) {
        v->v[0] -= w->v[0];
        v->v[1] -= w->v[1];
        v->v[2] -= w->v[2];
    }

    _ccd_inline void ccdVec3Sub2(ccd_vec3_t *d, const ccd_vec3_t *v, const ccd_vec3_t *w) {
        d->v[0] = v->v[0] - w->v[0];
        d->v[1] = v->v[1] - w->v[1];
        d->v[2] = v->v[2] - w->v[2];
    }

    _ccd_inline void ccdVec3Add(ccd_vec3_t *v, const ccd_vec3_t *w) {
        v->v[0] += w->v[0];
        v->v[1] += w->v[1];
        v->v[2] += w->v[2];
    }

    _ccd_inline void ccdVec3Scale(ccd_vec3_t *d, ccd_real_t k) {
        d->v[0] *= k;
        d->v[1] *= k;
        d->v[2] *= k;
    }

    _ccd_inline void ccdVec3Normalize(ccd_vec3_t *d) {
        ccd_real_t k = CCD_ONE / CCD_SQRT(ccdVec3Len2(d));
        ccdVec3Scale(d, k);
    }

    _ccd_inline ccd_real_t ccdVec3Dot(const ccd_vec3_t *a, const ccd_vec3_t *b) {
        ccd_real_t dot;

        dot = a->v[0] * b->v[0];
        dot += a->v[1] * b->v[1];
        dot += a->v[2] * b->v[2];
        return dot;
    }

    _ccd_inline void ccdVec3Cross(ccd_vec3_t *d, const ccd_vec3_t *a, const ccd_vec3_t *b) {
        d->v[0] = (a->v[1] * b->v[2]) - (a->v[2] * b->v[1]);
        d->v[1] = (a->v[2] * b->v[0]) - (a->v[0] * b->v[2]);
        d->v[2] = (a->v[0] * b->v[1]) - (a->v[1] * b->v[0]);
    }


    static CCD_VEC3(__ccd_vec3_origin, CCD_ZERO, CCD_ZERO, CCD_ZERO);
    ccd_vec3_t *ccd_vec3_origin = &__ccd_vec3_origin;

    static ccd_vec3_t points_on_sphere[] = {
            CCD_VEC3_STATIC(CCD_REAL(0.000000), CCD_REAL(-0.000000), CCD_REAL(-1.000000)),
            CCD_VEC3_STATIC(CCD_REAL(0.723608), CCD_REAL(-0.525725), CCD_REAL(-0.447219)),
            CCD_VEC3_STATIC(CCD_REAL(-0.276388), CCD_REAL(-0.850649), CCD_REAL(-0.447219)),
            CCD_VEC3_STATIC(CCD_REAL(-0.894426), CCD_REAL(-0.000000), CCD_REAL(-0.447216)),
            CCD_VEC3_STATIC(CCD_REAL(-0.276388), CCD_REAL(0.850649), CCD_REAL(-0.447220)),
            CCD_VEC3_STATIC(CCD_REAL(0.723608), CCD_REAL(0.525725), CCD_REAL(-0.447219)),
            CCD_VEC3_STATIC(CCD_REAL(0.276388), CCD_REAL(-0.850649), CCD_REAL(0.447220)),
            CCD_VEC3_STATIC(CCD_REAL(-0.723608), CCD_REAL(-0.525725), CCD_REAL(0.447219)),
            CCD_VEC3_STATIC(CCD_REAL(-0.723608), CCD_REAL(0.525725), CCD_REAL(0.447219)),
            CCD_VEC3_STATIC(CCD_REAL(0.276388), CCD_REAL(0.850649), CCD_REAL(0.447219)),
            CCD_VEC3_STATIC(CCD_REAL(0.894426), CCD_REAL(0.000000), CCD_REAL(0.447216)),
            CCD_VEC3_STATIC(CCD_REAL(-0.000000), CCD_REAL(0.000000), CCD_REAL(1.000000)),
            CCD_VEC3_STATIC(CCD_REAL(0.425323), CCD_REAL(-0.309011), CCD_REAL(-0.850654)),
            CCD_VEC3_STATIC(CCD_REAL(-0.162456), CCD_REAL(-0.499995), CCD_REAL(-0.850654)),
            CCD_VEC3_STATIC(CCD_REAL(0.262869), CCD_REAL(-0.809012), CCD_REAL(-0.525738)),
            CCD_VEC3_STATIC(CCD_REAL(0.425323), CCD_REAL(0.309011), CCD_REAL(-0.850654)),
            CCD_VEC3_STATIC(CCD_REAL(0.850648), CCD_REAL(-0.000000), CCD_REAL(-0.525736)),
            CCD_VEC3_STATIC(CCD_REAL(-0.525730), CCD_REAL(-0.000000), CCD_REAL(-0.850652)),
            CCD_VEC3_STATIC(CCD_REAL(-0.688190), CCD_REAL(-0.499997), CCD_REAL(-0.525736)),
            CCD_VEC3_STATIC(CCD_REAL(-0.162456), CCD_REAL(0.499995), CCD_REAL(-0.850654)),
            CCD_VEC3_STATIC(CCD_REAL(-0.688190), CCD_REAL(0.499997), CCD_REAL(-0.525736)),
            CCD_VEC3_STATIC(CCD_REAL(0.262869), CCD_REAL(0.809012), CCD_REAL(-0.525738)),
            CCD_VEC3_STATIC(CCD_REAL(0.951058), CCD_REAL(0.309013), CCD_REAL(0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(0.951058), CCD_REAL(-0.309013), CCD_REAL(0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(0.587786), CCD_REAL(-0.809017), CCD_REAL(0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(0.000000), CCD_REAL(-1.000000), CCD_REAL(0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(-0.587786), CCD_REAL(-0.809017), CCD_REAL(0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(-0.951058), CCD_REAL(-0.309013), CCD_REAL(-0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(-0.951058), CCD_REAL(0.309013), CCD_REAL(-0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(-0.587786), CCD_REAL(0.809017), CCD_REAL(-0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(-0.000000), CCD_REAL(1.000000), CCD_REAL(-0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(0.587786), CCD_REAL(0.809017), CCD_REAL(-0.000000)),
            CCD_VEC3_STATIC(CCD_REAL(0.688190), CCD_REAL(-0.499997), CCD_REAL(0.525736)),
            CCD_VEC3_STATIC(CCD_REAL(-0.262869), CCD_REAL(-0.809012), CCD_REAL(0.525738)),
            CCD_VEC3_STATIC(CCD_REAL(-0.850648), CCD_REAL(0.000000), CCD_REAL(0.525736)),
            CCD_VEC3_STATIC(CCD_REAL(-0.262869), CCD_REAL(0.809012), CCD_REAL(0.525738)),
            CCD_VEC3_STATIC(CCD_REAL(0.688190), CCD_REAL(0.499997), CCD_REAL(0.525736)),
            CCD_VEC3_STATIC(CCD_REAL(0.525730), CCD_REAL(0.000000), CCD_REAL(0.850652)),
            CCD_VEC3_STATIC(CCD_REAL(0.162456), CCD_REAL(-0.499995), CCD_REAL(0.850654)),
            CCD_VEC3_STATIC(CCD_REAL(-0.425323), CCD_REAL(-0.309011), CCD_REAL(0.850654)),
            CCD_VEC3_STATIC(CCD_REAL(-0.425323), CCD_REAL(0.309011), CCD_REAL(0.850654)),
            CCD_VEC3_STATIC(CCD_REAL(0.162456), CCD_REAL(0.499995), CCD_REAL(0.850654))
    };
    ccd_vec3_t *ccd_points_on_sphere = points_on_sphere;
    size_t ccd_points_on_sphere_len = sizeof(points_on_sphere) / sizeof(ccd_vec3_t);


    _ccd_inline ccd_real_t __ccdVec3PointSegmentDist2(const ccd_vec3_t *P,
                                                      const ccd_vec3_t *x0,
                                                      const ccd_vec3_t *b,
                                                      ccd_vec3_t *witness) {
        // The computation comes from solving equation of segment:
        //      S(t) = x0 + t.d
        //          where - x0 is initial point of segment
        //                - d is direction of segment from x0 (|d| > 0)
        //                - t belongs to <0, 1> interval
        //
        // Than, distance from a segment to some point P can be expressed:
        //      D(t) = |x0 + t.d - P|^2
        //          which is distance from any point on segment. Minimization
        //          of this function brings distance from P to segment.
        // Minimization of D(t) leads to simple quadratic equation that's
        // solving is straightforward.
        //
        // Bonus of this method is witness point for free.

        ccd_real_t dist, t;
        ccd_vec3_t d, a;

        // direction of segment
        ccdVec3Sub2(&d, b, x0);

        // precompute vector from P to x0
        ccdVec3Sub2(&a, x0, P);

        t = -CCD_REAL(1.) * ccdVec3Dot(&a, &d);
        t /= ccdVec3Len2(&d);

        if (t < CCD_ZERO || ccdIsZero(t)) {
            dist = ccdVec3Dist2(x0, P);
            if (witness)
                ccdVec3Copy(witness, x0);
        } else if (t > CCD_ONE || ccdEq(t, CCD_ONE)) {
            dist = ccdVec3Dist2(b, P);
            if (witness)
                ccdVec3Copy(witness, b);
        } else {
            if (witness) {
                ccdVec3Copy(witness, &d);
                ccdVec3Scale(witness, t);
                ccdVec3Add(witness, x0);
                dist = ccdVec3Dist2(witness, P);
            } else {
                // recycling variables
                ccdVec3Scale(&d, t);
                ccdVec3Add(&d, &a);
                dist = ccdVec3Len2(&d);
            }
        }

        return dist;
    }

    ccd_real_t ccdVec3PointSegmentDist2(const ccd_vec3_t *P,
                                        const ccd_vec3_t *x0, const ccd_vec3_t *b,
                                        ccd_vec3_t *witness) {
        return __ccdVec3PointSegmentDist2(P, x0, b, witness);
    }

    ccd_real_t ccdVec3PointTriDist2(const ccd_vec3_t *P,
                                    const ccd_vec3_t *x0, const ccd_vec3_t *B,
                                    const ccd_vec3_t *C,
                                    ccd_vec3_t *witness) {
        // Computation comes from analytic expression for triangle (x0, B, C)
        //      T(s, t) = x0 + s.d1 + t.d2, where d1 = B - x0 and d2 = C - x0 and
        // Then equation for distance is:
        //      D(s, t) = | T(s, t) - P |^2
        // This leads to minimization of quadratic function of two variables.
        // The solution from is taken only if s is between 0 and 1, t is
        // between 0 and 1 and t + s < 1, otherwise distance from segment is
        // computed.

        ccd_vec3_t d1, d2, a;
        ccd_real_t u, v, w, p, q, r, d;
        ccd_real_t s, t, dist, dist2;
        ccd_vec3_t witness2;

        ccdVec3Sub2(&d1, B, x0);
        ccdVec3Sub2(&d2, C, x0);
        ccdVec3Sub2(&a, x0, P);

        u = ccdVec3Dot(&a, &a);
        v = ccdVec3Dot(&d1, &d1);
        w = ccdVec3Dot(&d2, &d2);
        p = ccdVec3Dot(&a, &d1);
        q = ccdVec3Dot(&a, &d2);
        r = ccdVec3Dot(&d1, &d2);

        d = w * v - r * r;
        if (ccdIsZero(d)) {
            // To avoid division by zero for zero (or near zero) area triangles
            s = t = -1.;
        } else {
            s = (q * r - w * p) / d;
            t = (-s * r - q) / w;
        }

        if ((ccdIsZero(s) || s > CCD_ZERO)
            && (ccdEq(s, CCD_ONE) || s < CCD_ONE)
            && (ccdIsZero(t) || t > CCD_ZERO)
            && (ccdEq(t, CCD_ONE) || t < CCD_ONE)
            && (ccdEq(t + s, CCD_ONE) || t + s < CCD_ONE)) {

            if (witness) {
                ccdVec3Scale(&d1, s);
                ccdVec3Scale(&d2, t);
                ccdVec3Copy(witness, x0);
                ccdVec3Add(witness, &d1);
                ccdVec3Add(witness, &d2);

                dist = ccdVec3Dist2(witness, P);
            } else {
                dist = s * s * v;
                dist += t * t * w;
                dist += CCD_REAL(2.) * s * t * r;
                dist += CCD_REAL(2.) * s * p;
                dist += CCD_REAL(2.) * t * q;
                dist += u;
            }
        } else {
            dist = __ccdVec3PointSegmentDist2(P, x0, B, witness);

            dist2 = __ccdVec3PointSegmentDist2(P, x0, C, &witness2);
            if (dist2 < dist) {
                dist = dist2;
                if (witness)
                    ccdVec3Copy(witness, &witness2);
            }

            dist2 = __ccdVec3PointSegmentDist2(P, B, C, &witness2);
            if (dist2 < dist) {
                dist = dist2;
                if (witness)
                    ccdVec3Copy(witness, &witness2);
            }
        }

        return dist;
    }


}


#endif //TCCPPLIB_VEC3_HPP
