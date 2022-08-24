#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Vectors") {
  SUBCASE("Testing == and !=") {
    Vector v1{1., 2.};
    Vector v2{3., 4.};
    Vector v3{1., 2.};
    CHECK(v1 == v3);
    CHECK(v2 != v3);
  }
  SUBCASE("Testing x() and y() methods") {  // serve?
    Vector v1{1., 2.};
    CHECK(v1.x() == doctest::Approx(1.));
    CHECK(v1.y() == doctest::Approx(2.));
  }
  SUBCASE("Testing += and -=") {
    Vector v1{1., 2.};
    Vector v2{3., 4.};
    Vector v3{4., 6.};
    Vector v4{-1., -2.};
    v1 += v2;
    v2 -= v3;
    CHECK(v1 == v3);
    CHECK(v2 == v4);
    CHECK_FALSE(v1 == v2);
  }
  SUBCASE("Testing + and -") {
    Vector v1{1., 2.};
    Vector v2{3., 4.};
    Vector v3 = v1 + v2;
    Vector v4 = v1 - v2;
    CHECK(v3.x() == doctest::Approx(4.));
    CHECK(v3.y() == doctest::Approx(6.));
    CHECK(v4.x() == doctest::Approx(-2.));
    CHECK(v4.y() == doctest::Approx(-2.));
  }
  SUBCASE("Testing *= and /=") {
    Vector v1{1., 2.};
    double scalar = 2.;
    Vector v2{2., 4.};
    Vector v3{0.5, 1.};
    CHECK((v1 *= scalar) == v2);
    scalar = 4;
    v2 /= scalar;
    CHECK(v2 == v3);
    //CHECK(v2.y() == doctest::Approx(2.));
    scalar = 0.;
    CHECK_THROWS(v1 /= scalar);
  }
  SUBCASE("Testing * and /") {
    Vector v1{1., 2.};
    Vector v2{3., 4.};
    double scalar = v1 * v2;
    CHECK(scalar == doctest::Approx(11.));
    scalar = 2.;
    Vector v3{2., 4.};
    Vector v4{0.5, 1.};
    CHECK((v1 * scalar) == v3);
    CHECK((v1 / scalar) == v4);
    scalar = 0.;
    CHECK_THROWS(v1 / scalar);
  }
  SUBCASE("Testing norm2") {
    Vector v{2., 3.};
    CHECK(norm2(v) == 13.);
  }
}