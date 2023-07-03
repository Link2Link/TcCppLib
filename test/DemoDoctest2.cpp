#include "doctest/doctest.h"

#include "division.h"

TEST_CASE("Demo2"){
  INFO("show information");
  REQUIRE("foobar" == doctest::Contains("foo"));

  REQUIRE_MESSAGE("foobar" == doctest::Contains("foo"), "same as INFO");

  INFO("float approx compare");
  REQUIRE(22.0/7 == doctest::Approx(3.141).epsilon(0.01)); // allow for a 1% error



}

TEST_CASE("Division test")
{
    double a , b;
    a = 1;
    b = 2;
    double c = calc(1, a, b);
    CHECK(c == 2);

    a = 1;
    b = 2;
    c = calc(2, a, b);
    CHECK(c == 3);


}