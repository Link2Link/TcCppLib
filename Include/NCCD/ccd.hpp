#ifndef TCCPPLIB_INCLUDE_NCCD_CCD_HPP_
#define TCCPPLIB_INCLUDE_NCCD_CCD_HPP_

#include "vec.hpp"

namespace NCCD
{
/**
 * Type of *support* function that takes pointer to 3D object and direction
 * and returns (via vec argument) furthest point from object in specified
 * direction.
 */
	typedef void (*ccd_support_fn)(const void *obj, const vec3_t & dir,
		vec3_t &vec);

/**
 * Returns (via dir argument) first direction vector that will be used in
 * initialization of algorithm.
 */
	typedef void (*ccd_first_dir_fn)(const void *obj1, const void *obj2, vec3_t & dir);

/**
 * Returns (via center argument) geometric center (some point near center)
 * of given object.
 */
	typedef void (*ccd_center_fn)(const void *obj1, vec3_t & center);

/**
* Main structure of CCD algorithm.
*/
	class CCD
	{
	 protected:
		static void ccdFirstDirDefault(const void *o1, const void *o2, vec3_t & dir)
		{
			Vec3Set(dir, ONE, ZERO, ZERO);
		}
	 public:
		ccd_first_dir_fn first_dir; //!< Returns initial direction where first support point will be searched
		ccd_support_fn support1; //!< Function that returns support point of first object
		ccd_support_fn support2; //!< Function that returns support point of second object
		ccd_center_fn center1; //!< Function that returns geometric center of first object
		ccd_center_fn center2; //!< Function that returns geometric center of second object
		unsigned long max_iterations; //!< Maximal number of iterations
		real epa_tolerance;
		real mpr_tolerance; //!< Boundary tolerance for MPR algorithm
		real dist_tolerance;

		CCD()
		{
			first_dir = ccdFirstDirDefault;
			support1 = nullptr;
        	support2 = nullptr;
        	center1  = nullptr;
        	center2  = nullptr;

        	max_iterations = (unsigned long)-1;
        	epa_tolerance = real(0.0001);
        	mpr_tolerance = real(0.0001);
        	dist_tolerance = real(1E-6);
		}

	};
}





#endif //TCCPPLIB_INCLUDE_NCCD_CCD_HPP_
