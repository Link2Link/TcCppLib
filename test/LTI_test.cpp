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

TEST_CASE("State Space Test : Linear Check"){
    using namespace LTI;
    StateSpace<2, 1, 1> ss;
    ss.A << -1, -2,
            -3, -4;
    ss.B << -1, 1;
    ss.C << 1, 0;

    ss.x(0) = 0;

    auto ss2 = ss;
    auto ss3 = ss;

    for (; ss.t < 5; )
    {
        ss.u(0) = sin(ss.t);
        ss2.u(0) = cos(ss2.t);
        ss3.u(0) = sin(ss.t) + 2 * cos(ss2.t);

        ss.step(1E-3);
        ss2.step(1E-3);
        ss3.step(1E-3);
    }
    CHECK(ss.y(0)+ss2.y(0)*2 == doctest::Approx(ss3.y(0)).epsilon(1E-3));    // Check linearity

//    std::cout << ss.y(0)+ss2.y(0)*2 << " |||| "  << ss3.y(0) << std::endl;

}
