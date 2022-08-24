#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "multiflock.hpp"
#include "rules.hpp"
#include "boids.hpp"
#include "doctest.h"
#include "rules.hpp"
#include "vector.hpp"

// test should be compiled with: g++ vector.cpp boids.cpp boids.test.cpp

TEST_CASE("Testing Boids") {
  SUBCASE("Testing operator== and operator!= ") { //i metodi di boids vanno testati?
    Boid b1{Vector{1., 2.}, Vector{1., 1.}};
    Boid b2{Vector{1., 2.}, Vector{1., 1.}};
    Boid b3{Vector{5., 5.}, Vector{1., 2.}}; 
    CHECK(b1 == b2);
    CHECK(b1 != b3);
    //Vector v = applied_distance(b1, b3);
   // CHECK(v.x() == doctest::Approx(4.));
    //CHECK(v.y() == doctest::Approx(3.));
  }
}
TEST_CASE("Testing rules") {
  Boid b1{Vector{1., 2.}, Vector{1., 0}};
  Boid b2{Vector{1., 2.}, Vector{1., 0}};
  Boid b3{Vector{5., 5.}, Vector{1., 2.}}; 
  Boid b4{Vector{3., 0.}, Vector{0., 0.}};
  std::vector<Boid> boids{};
  Options boids_options{3., 0.5, 0.3, 0.1, 0.8}; //0.8 invece di 1 per ultimo
  double alpha{180.};
  Flock flock{boids, boids_options, alpha};
  SUBCASE("Testing is_member") {
    flock.add(b1);
    flock.add(b3);
    CHECK(is_member(flock.get_boids(), b2)); //b1 e b2 hanno stesse coord quindi dice che b2 c'è, va bene?
    CHECK((is_member(flock.get_boids(), b4)) == false);
  }
  SUBCASE("Testing get_neighbours_of") {
    flock.add(b1);
    flock.add(b3);
    flock.add(b4);
    std::vector<Boid> neighbours = get_neighbours_of(flock, b1);
    CHECK((is_member(neighbours, b3)) == false); 
    CHECK(is_member(neighbours, b4));
    flock.add(b2);
    neighbours = get_neighbours_of(flock, b1);
    CHECK((is_member(neighbours, b2)) == false);
   // Boid b6{Vector{0., 0.}, Vector{0., 0.}};
    //flock.add(b6);
    //neighbours = get_neighbours_of(flock, b1);
   // CHECK(is_member(neighbours, b6));
  }
  SUBCASE("Testing view_neighbours") {
    Flock flock_1{boids, boids_options, 90.}; //90 degrees per side
    flock_1.add(b1);
    flock_1.add(b4);
    Boid b5{Vector{3., 2.}, Vector{0., 0.}};
    flock_1.add(b5);
    Boid b6{Vector{0., 0.}, Vector{0., 0.}};
    flock_1.add(b6);
    Boid b7{Vector{0., 2.}, Vector{0., 0.}};
    flock_1.add(b7);
    std::vector<Boid> neighbours = view_neighbours(flock_1, b1);
    CHECK(is_member(neighbours, b4));
    CHECK(is_member(neighbours, b5));
    CHECK((is_member(neighbours, b6)) == false);
    CHECK((is_member(neighbours, b7)) == false); 
  } //altro?
  SUBCASE("Testing separation") {}
  SUBCASE("Testing alignment") {}
  SUBCASE("Testing cohesion") {}

}
/*
//Boid b3{Vector{10, 10}, Vector{0, 0}};
Boid b4{Vector{10, 0}, Vector{0, 0}};

Boid b5{Vector{8, 7}, Vector{0, 0}};
Boid b6{Vector{2, 3}, Vector{0, 0}};
Boid b7{Vector{4, 4}, Vector{0, 0}};
Boid b8{Vector{1.6, 3.8}, Vector{0, 0}};

Options sp{10., 0.6, 0.4, 0.1, 0.2};  //

Flock f1{std::vector<Boid>{b1, b2, b3, b4}, sp, 45};
Flock f2{std::vector<Boid>{b5, b6, b7, b8}, sp, 45}
*/