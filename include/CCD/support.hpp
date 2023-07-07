//
// Created by Chenfei Wang on 7/7/2023.
//

#ifndef TCCPPLIB_SUPPORT_HPP
#define TCCPPLIB_SUPPORT_HPP

#include "ccd.hpp"

namespace CCD {
    struct _ccd_support_t {
        ccd_vec3_t v;  //!< Support point in minkowski sum
        ccd_vec3_t v1; //!< Support point in obj1
        ccd_vec3_t v2; //!< Support point in obj2
    };
    typedef struct _ccd_support_t ccd_support_t;

    _ccd_inline void ccdSupportCopy(ccd_support_t *, const ccd_support_t *s);

/**
 * Computes support point of obj1 and obj2 in direction dir.
 * Support point is returned via supp.
 */
    CCD_EXPORT void __ccdSupport(const void *obj1, const void *obj2,
                                 const ccd_vec3_t *dir, const ccd_t *ccd,
                                 ccd_support_t *supp);


/**** INLINES ****/
    _ccd_inline void ccdSupportCopy(ccd_support_t *d, const ccd_support_t *s) {
        *d = *s;
    }


    void __ccdSupport(const void *obj1, const void *obj2,
                      const ccd_vec3_t *_dir, const ccd_t *ccd,
                      ccd_support_t *supp) {
        ccd_vec3_t dir;

        ccdVec3Copy(&dir, _dir);

        ccd->support1(obj1, &dir, &supp->v1);

        ccdVec3Scale(&dir, -CCD_ONE);
        ccd->support2(obj2, &dir, &supp->v2);

        ccdVec3Sub2(&supp->v, &supp->v1, &supp->v2);
    }

}


#endif //TCCPPLIB_SUPPORT_HPP
