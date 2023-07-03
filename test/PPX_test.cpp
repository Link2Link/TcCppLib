#include "doctest/doctest.h"
#include "PPX/ppx.h"
#include "CHEN/division.h"

TEST_CASE("expr test"){
	using namespace ppx;
	MatrixS<3, 4> m;
	ones(m);
	m = m.eye();

	m.sub<2, 2, false>(0, 0) = {2, 3, 4};
	m.sub<3, 1, false>(0, 3) = MatrixS<3, 1>::zeros() - MatrixS<3, 1>::eye();

	MatrixS<3, 4> m2 = {2, 3, 0, 4, 0, 0, 0, 0, 1, -1, 0, 0};
	CHECK(m == m2);

}