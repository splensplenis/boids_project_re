#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("Testing Vectors") {
    SUBCASE("Testing operator== and operator!=") {
        Vector v1{1., 2.};
        Vector v2{3., 4.};
        Vector v3{1., 2.};
        CHECK(v1 == v3);
        CHECK(v2 != v3);
    }
    SUBCASE("Testing x() and y() methods") {//va bene scritto cosi?
        Vector v1{1., 2.};
        CHECK(v1.x() == doctest::Approx(1.));
        CHECK(v1.y() == doctest::Approx(2.));
    }
    SUBCASE("Testing operator+= and operator-=") {
        Vector v1{1., 2.};
        Vector v2{3., 4.};
        Vector v3{4., 6.};
        Vector v4{-1., -2.};
        v1 += v2;
        v2 -= v3;
        CHECK(v1 == v3);
        CHECK(v2 == v4);
        CHECK_FALSE(v1==v2);
    }
    SUBCASE("Testing operator+ and operator-") {
        Vector v1{1., 2.};
        Vector v2{3., 4.};
        Vector v3 = v1 + v2;
        Vector v4 = v1 - v2;
        CHECK(v3.x() == doctest::Approx(4.));
        CHECK(v3.y() == doctest::Approx(6.));
        CHECK(v4.x() == doctest::Approx(-2.));
        CHECK(v4.y() == doctest::Approx(-2.));
    }
}

