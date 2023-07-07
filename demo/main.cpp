
#include <iostream>
#include "CCD/include.hpp"

using namespace std;

using namespace CCD;

typedef struct
{
    double x;
    double y;
    double z;
    ccd_vec3_t pos;
    ccd_quat_t quat;
}obj_t;


void support(const void *_obj, const ccd_vec3_t *_dir, ccd_vec3_t *vec)
{
    // assume that obj_t is user-defined structure that holds info about
    // object (in this case box: x, y, z, pos, quat - dimensions of box,
    // position and rotation)
    obj_t *box = (obj_t *)_obj;
    ccd_vec3_t dir;
    ccd_quat_t qinv;

    // apply rotation on direction vector
    ccdVec3Copy(&dir, _dir);
    ccdQuatInvert2(&qinv, &box->quat);
    ccdQuatRotVec(&dir, &qinv);

    // compute support point in specified direction
    ccdVec3Set(vec, ccdSign(ccdVec3X(&dir)) * box->x * CCD_REAL(0.5),
               ccdSign(ccdVec3Y(&dir)) * box->y * CCD_REAL(0.5),
               ccdSign(ccdVec3Z(&dir)) * box->z * CCD_REAL(0.5));

    // transform support point according to position and rotation of object
    ccdQuatRotVec(vec, &box->quat);
    ccdVec3Add(vec, &box->pos);
}


int main()
{
    ccd_t ccd;
    CCD_INIT(&ccd); // initialize ccd_t struct

    // set up ccd_t struct
    ccd.support1       = support; // support function for first object
    ccd.support2       = support; // support function for second object
    ccd.max_iterations = 100;     // maximal number of iterations


    obj_t obj1, obj2;

    obj1.x = 1;
    obj1.y = 1;
    obj1.z = 1;
    obj1.pos.v[0] = 0;
    obj1.pos.v[1] = 0;
    obj1.pos.v[2] = 0;
    obj1.quat.q[0] = 0;
    obj1.quat.q[1] = 0;
    obj1.quat.q[2] = 0;
    obj1.quat.q[3] = 1;

    obj2.x = 1;
    obj2.y = 1;
    obj2.z = 1;
    obj2.pos.v[0] = 1;
    obj2.pos.v[1] = 1;
    obj2.pos.v[2] = 0;
    obj2.quat.q[0] = 0;
    obj2.quat.q[1] = 0;
    obj2.quat.q[2] = 0;
    obj2.quat.q[3] = 1;



    int intersect = ccdGJKIntersect(&obj1, &obj2, &ccd);

    cout << intersect << endl;

	return 0;
}