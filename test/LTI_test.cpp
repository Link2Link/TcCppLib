#include "doctest/doctest.h"
#include "CHEN/statespace.hpp"
#include "iostream"


TEST_CASE("State Space Test : Second Order Integrator"){
    using namespace LTI;
    StateSpace<2, 1, 1> ss;
    ss.A << 0, 1,
            0, 0;
    ss.B << 0, 1;
    ss.C << 1, 0;

    ss.x(0) = 0;
    ss.u(0) = 1;

    State<1> y;

    for (; ss.t < 5; )
    {
        ss.step(1E-3);
//        std::cout << " t: " << ss.t << " y: "  << ss.y << std::endl;
    }
    y(0) = 0.5* pow(ss.t, 2);   // 解析表达式 y = 1/2*t^2
    CHECK(ss.y(0) == doctest::Approx(y(0)).epsilon(1E-3));


}