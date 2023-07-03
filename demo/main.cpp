
#include <iostream>
#include <random>
#include <future>
#include <fstream>

#include "PPX/ppx.h"
#include "ppxlog.h"



using namespace std;
using namespace ppx;


template <typename T>
inline void PRINT_SINGLE_ELEMENTS(const T &coll, const std::string &optcsrt = "")
{
	std::cout << optcsrt << coll << std::endl;
}

template <typename T>
inline void PRINT_LISTED_ELEMENTS(const T &coll, const std::string &optcsrt = "")
{
	std::cout << optcsrt;
	for (const auto ele : coll)
	{
		std::cout << ele << ' ';
	}
	std::cout << std::endl;
}

void test_expr()
{
	MatrixS<3, 4> m;
	ones(m);
	m = m.eye();


	PRINT_SINGLE_ELEMENTS(m, "m = ");
	// elem 0
	m.sub<2, 2, false>(0, 0) = {2, 3, 4};
	m.sub<3, 1, false>(0, 3) = MatrixS<3, 1>::zeros() - MatrixS<3, 1>::eye();
	PRINT_SINGLE_ELEMENTS(m, "m = ");

	MatrixS<3, 4> m2 = {2, 3, 0, 4, 0, 0, 0, 0, 1, -1, 0, 0};
	PRINT_SINGLE_ELEMENTS(m2, "m2 = ");
	cout << (m == m2);


}

void test_matrix()
{
}


int main()
{
	test_expr();
	test_matrix();
	return 0;
}