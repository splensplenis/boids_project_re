#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include<array>

#include "boid_spec.hpp"
#include "doctest.h"

using namespace BoidSpec;

TEST_CASE("Constructor"){
    std::array<float,3> color {0.0,0.0,0.0};
    SUBCASE("Testing name"){
        CHECK_NOTHROW(boid_specie{"",color,360});
        boid_specie test1{"",color,360};
        CHECK(test1.GetName()=="");
    }
    SUBCASE("Testing colors"){
        SUBCASE("Negative red"){
            std::array<float,3>prova_color {-4.0,0.0,0.0};
            CHECK_THROWS(boid_specie{"a",prova_color,360});
        }
        SUBCASE("Red bigger than one"){
            std::array<float,3>prova_color{2.0,0.0,0.0};
            CHECK_THROWS(boid_specie{"a",prova_color,360});
        }
        SUBCASE("Negative green"){
            std::array<float,3>prova_color{0.0,-3.5,0.0};
            CHECK_THROWS(boid_specie{"a",prova_color,360});
        }
        SUBCASE("Green bigger than one"){
            std::array<float,3>prova_color {0.0,3.5,0.0};
            CHECK_THROWS(boid_specie{"a",prova_color,360});
        }
        SUBCASE("Negative blue"){
            std::array<float,3>prova_color {0.0,0.0,-0.1};
            CHECK_THROWS(boid_specie{"a",prova_color,360});
        }
        SUBCASE("Blue bigger than one"){
            std::array<float,3>prova_color {0.0,0.0,1.1};
            CHECK_THROWS(boid_specie{"a",prova_color,360});
        }
        SUBCASE("All wrong"){
            std::array<float,3>prova_color {-1.0,-3.5,4.0};
            CHECK_THROWS(boid_specie{"a",prova_color,360});
        }
        SUBCASE("Correct"){
            std::array<float,3>prova_color {0.3,0.0,1.0};
            CHECK_NOTHROW(boid_specie{"a",prova_color,360});
        }

    }
    SUBCASE ("Testing vision angle"){
        SUBCASE("Negative angle"){
            CHECK_THROWS(boid_specie{"a",color,-4});
        }
        SUBCASE("Border line angles"){
            CHECK_NOTHROW(boid_specie{"a",color,0});
            CHECK_NOTHROW(boid_specie{"a",color,360});
            CHECK_NOTHROW(boid_specie{"a",color,361});
        }
        SUBCASE("Angles bigger than 360"){

            boid_specie test1{"a",color,361};
            CHECK(test1.GetVisionAngle() == 1);

            boid_specie test2 {"a", color, 520};
            CHECK(test2.GetVisionAngle() == 160);
            
            boid_specie test3 {"a", color, 360};
            CHECK(test3.GetVisionAngle() == 360);
            
            boid_specie test4 {"a", color, 720};
            CHECK(test4.GetVisionAngle() == 360);
            
            boid_specie test5 {"a", color, 721};
            CHECK(test5.GetVisionAngle() == 1);
        }
    }
    SUBCASE("Default"){
        boid_specie basic{};
        CHECK(basic.GetName() == "Std Boid");
        CHECK(basic.GetColor() == std::array<float,3>{0.0,0.0,0.0});
        CHECK(basic.GetVisionAngle() == 360);
    }
}

TEST_CASE("SetName"){
    boid_specie test{};
    CHECK(test.GetName() == "Std Boid");
    CHECK_NOTHROW(test.SetName(""));
    test.SetName("");
    CHECK(test.GetName() == "");
    test.SetName("abcdefg");
    CHECK(test.GetName() == "abcdefg");
}

TEST_CASE("SetColor"){
    boid_specie test{};
    CHECK(test.GetColor() == std::array<float,3>{0.0,0.0,0.0});
        SUBCASE("Negative red"){
            std::array<float,3>prova_color {-4.0,0.0,0.0};
            CHECK_THROWS(test.SetColor(prova_color));
        }
        SUBCASE("Red bigger than one"){
            std::array<float,3>prova_color{2.0,0.0,0.0};
            CHECK_THROWS(test.SetColor(prova_color));
        }
        SUBCASE("Negative green"){
            std::array<float,3>prova_color{0.0,-3.5,0.0};
            CHECK_THROWS(test.SetColor(prova_color));
        }
        SUBCASE("Green bigger than one"){
            std::array<float,3>prova_color {0.0,3.5,0.0};
            CHECK_THROWS(test.SetColor(prova_color));
        }
        SUBCASE("Negative blue"){
            std::array<float,3>prova_color {0.0,0.0,-0.1};
            CHECK_THROWS(test.SetColor(prova_color));
        }
        SUBCASE("Blue bigger than one"){
            std::array<float,3>prova_color {0.0,0.0,1.1};
            CHECK_THROWS(test.SetColor(prova_color));
        }
        SUBCASE("All wrong"){
            std::array<float,3>prova_color {-1.0,-3.5,4.0};
            CHECK_THROWS(test.SetColor(prova_color));
        }
        SUBCASE("Correct"){
            std::array<float,3>prova_color {0.3,0.0,1.0};
            CHECK_NOTHROW(test.SetColor(prova_color));
            CHECK(test.GetColor() == prova_color);
        }
}

TEST_CASE("SetVisionAngle"){
    boid_specie test{};
    CHECK(test.GetVisionAngle() == 360);
    SUBCASE("Negative angle"){
        CHECK_THROWS(test.SetVisionAngle(-3));

        CHECK_NOTHROW(test.SetVisionAngle(20));
        CHECK(test.GetVisionAngle() == 20);
        
        CHECK_NOTHROW(test.SetVisionAngle(0));
        CHECK(test.GetVisionAngle() == 0);

        CHECK_NOTHROW(test.SetVisionAngle(360));
        CHECK(test.GetVisionAngle() == 360);

        CHECK_NOTHROW(test.SetVisionAngle(361));
        CHECK(test.GetVisionAngle() == 1);

        CHECK_NOTHROW(test.SetVisionAngle(720));
        CHECK(test.GetVisionAngle() == 360);

        CHECK_NOTHROW(test.SetVisionAngle(727));
        CHECK(test.GetVisionAngle() == 7);
    }
}
