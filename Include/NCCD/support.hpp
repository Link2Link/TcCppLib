#ifndef TCCPPLIB_INCLUDE_NCCD_SUPPORT_HPP_
#define TCCPPLIB_INCLUDE_NCCD_SUPPORT_HPP_

#include "ccd.hpp"
namespace NCCD
{
	class Support
	{
	 public:
		vec3_t v;  //!< Support point in minkowski sum
		vec3_t v1; //!< Support point in obj1
		vec3_t v2; //!< Support point in obj2

		Support() = default;

		Support(const void *obj1, const void *obj2, const vec3_t & dir, const CCD & ccd)
		{
			getSupport(obj1, obj2, dir, ccd);
		}

		void getSupport(const void *obj1, const void *obj2, const vec3_t & _dir, const CCD & ccd)
		{
			vec3_t dir;
			dir = _dir;
			ccd.support1(obj1, dir, v1);
			ccd.support2(obj2, -dir, v2);
			v = v1 - v2;
		}
	};
}





#endif //TCCPPLIB_INCLUDE_NCCD_SUPPORT_HPP_
