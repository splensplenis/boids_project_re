#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include<sstream>

#include "io.hpp"
#include "doctest.h"

TEST_CASE("Testing input files"){
    std::string test_dir{"file_test/"};
    SUBCASE("Inexistent file"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"inexistent"));
    }
    SUBCASE("Empty name"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"empty_name.txt"));
    }
    SUBCASE("Negative vision angle"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"negative_angle.txt"));
    }
    SUBCASE("Negative red"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"negative_red.txt"));
    }
    SUBCASE("Negative green"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"negative_green.txt"));
    }
    SUBCASE("Negative blue"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"negative_blue.txt"));
    }
    SUBCASE("Red over one"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"red_over_one.txt"));
    }
    SUBCASE("Green over one"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"green_over_one.txt"));
    }
    SUBCASE("Blue over one"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"blue_over_one.txt"));
    }
    SUBCASE("Short file"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"short.txt"));
    }
    SUBCASE("Long file"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"long.txt"));
    }
    SUBCASE("Invalid angle"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"invalid_angle.txt"));
    }
    SUBCASE("Invalid red"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"invalid_red.txt"));
    }
    SUBCASE("Invalid green"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"invalid_green.txt"));
    }
    SUBCASE("Invalid blue"){
        CHECK_THROWS(LoadBoidFromFile(test_dir+"invalid_blue.txt"));
    }
    SUBCASE("Correct"){
        CHECK_NOTHROW(LoadBoidFromFile(test_dir+"correct.txt"));
        BoidSpec::boid_specie input{LoadBoidFromFile(test_dir+"correct.txt")};
        CHECK(input.GetName() == "Nuovo Boid");
        CHECK(input.GetVisionAngle() == 260);
        CHECK(input.GetColor() == std::array<float,3>{1,1,0});
    }
}