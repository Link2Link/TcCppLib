#ifndef TCCPPLIB_INCLUDE_NCCD_CCDC_HPP_
#define TCCPPLIB_INCLUDE_NCCD_CCDC_HPP_

#include "ccd.hpp"
#include "simplex.hpp"
#include "polytope.hpp"
namespace NCCD
{

	static int __GJK(const void *obj1, const void *obj2, const CCD & ccd, Simplex & simplex);


/**
 * Returns true if two given objects interest.
 */
	static int GJKIntersect(const void *obj1, const void *obj2, const CCD & ccd)
	{
		Simplex simplex;
		return __GJK(obj1, obj2, ccd, simplex) == 0;
	}


	static int __GJK(const void *obj1, const void *obj2, const CCD & ccd, Simplex & simplex)
	{
		unsigned long iterations;
		vec3_t dir; //direction vector
		Support last; //last support point
		int do_simplex_res;

		// get first direction
		ccd.first_dir(obj1, obj2, dir);
		// get first support point
		last.getSupport(obj1, obj2, dir, ccd);


		// and add this point to simplex as last one
		simplex.Add(last);

		// set up direction vector to as (O - last) which is exactly -last
		dir = -last.v;

		// start iterations
		for (iterations = 0UL; iterations < ccd.max_iterations; ++iterations)
		{
			// obtain support point
			last.getSupport(obj1, obj2, dir, ccd);

			// check if farthest point in Minkowski difference in direction dir
			// isn't somewhere before origin (the test on negative dot product)
			// - because if it is, objects are not intersecting at all.

			if (last.v.dot(dir) < ZERO){
				return -1; // intersection not found
			}

			// add last support vector to simplex
			simplex.Add(last);

			// if doSimplex returns 1 if objects intersect, -1 if objects don't
			// intersect and 0 if algorithm should continue
			do_simplex_res = simplex.doSimplex(dir);
			if (do_simplex_res == 1){
				return 0; // intersection found
			}else if (do_simplex_res == -1){
				return -1; // intersection not found
			}

			if (IsZero(Vec3Len2(dir))){
				return -1; // intersection not found
			}
		}
		return -1;
	}

}

#endif //TCCPPLIB_INCLUDE_NCCD_CCDC_HPP_
