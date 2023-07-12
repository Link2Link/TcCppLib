
#include <iostream>
#include "NCCD/vec.hpp"
#include "NCCD/quat.hpp"
#include "NCCD/ccd.hpp"
#include "NCCD/support.hpp"
#include "NCCD/simplex.hpp"
#include "NCCD/algorithm.hpp"


using namespace NCCD;
typedef struct
{
	real x;
	real y;
	real z;
	vec3_t pos;
	quat_t quat;
}obj_t;


void support(const void *_obj, const vec3_t &_dir, vec3_t & vec)
{
	// assume that obj_t is user-defined structure that holds info about
	// object (in this case box: x, y, z, pos, quat - dimensions of box,
	// position and rotation)
	obj_t *box = (obj_t *)_obj;
	vec3_t dir;
	quat_t qinv;

	// apply rotation on direction vector
	dir = _dir;
	qinv = box->quat.inverse();

	QuatRotVec(dir, qinv);
	// compute support point in specified direction
	Vec3Set(vec, Sign(Vec3X(dir)) * box->x * real(0.5),
		Sign(Vec3Y(dir)) * box->y * real(0.5),
		Sign(Vec3Z(dir)) * box->z * real(0.5));

	// transform support point according to position and rotation of object
	QuatRotVec(vec, box->quat);

	vec += box->pos;
}

int main() {

    using namespace std;
	CCD ccd;
	ccd.support1       = support; // support function for first object
	ccd.support2       = support; // support function for second object
	ccd.max_iterations = 100;     // maximal number of iterations

	for (double x = -2; x < 2; x+=1E-3)
	{
		obj_t obj1 = {1,1,1, vec3_t(0,0,0), quat_t(0.9239,0,0,0.3827)};
		obj_t obj2 = {1,1,1, vec3_t(x, 0,0), quat_t(0,0,0,1)};
		int intersect = GJKIntersect(&obj1, &obj2, ccd);
		cout << "x : " << x << " interset : " << intersect << endl;

	}

	return 0;
}