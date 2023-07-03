#include "doctest/doctest.h"


int factorial(int number) { return number <= 1 ? number : factorial(number - 1) * number; }

/*
 * REQUIRE：这个等级算是最高的，如果断言失败，不仅会标记为测试不通过，而且会强制退出测试
 * CHECK：如果断言失败，标记为测试不通过，但不会强制退出（也就是后续的测试还是会执行）。
 * WARN：如果断言失败，不会标记测试不通过，也不会强制退出，但是会给出对应的提示。
 * */

TEST_CASE("Demo"){
  INFO("show information");
  REQUIRE("foobar" == doctest::Contains("foo"));

  REQUIRE_MESSAGE("foobar" == doctest::Contains("foo"), "same as INFO");

  INFO("float approx compare");
  REQUIRE(22.0/7 == doctest::Approx(3.141).epsilon(0.01)); // allow for a 1% error

}


TEST_CASE("testing the factorial function") {
  CHECK(factorial(1) == 1);
  CHECK(factorial(2) == 2);
  CHECK(factorial(3) == 6);
  CHECK(factorial(10) == 3628800);
}


#include <vector>
TEST_CASE("vectors can be sized and resized") {
  std::vector<int> v(5);

  REQUIRE(v.size() == 5);
  REQUIRE(v.capacity() >= 5);
  SUBCASE("adding to the vector increases it's size"){
    v.push_back(1);
    CHECK(v.size() == 6);
  }

  SUBCASE("reserving increases just the capacity") {
    v.reserve(6);
    CHECK(v.size() == 5);
    CHECK(v.capacity() >= 6);
  }
}



class UniqueTestsFixture
{
private:
  static int uniqueID;
protected:
  int conn;
public:
  UniqueTestsFixture(): conn(10){}
protected:
  static int getID(){return ++uniqueID;}
};

int UniqueTestsFixture::uniqueID = 0;

/*
 * TEST_CASE_FIXTURE 用来测试类，相当于是通过继承的方式创建了一个新的类
 * */

TEST_CASE_FIXTURE(UniqueTestsFixture, "test get ID")
{
      REQUIRE(getID() == 1);
}

