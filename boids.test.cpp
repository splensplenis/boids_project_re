#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "multiflock.hpp"
#include "rules.hpp"
#include "boids.hpp"
#include "doctest.h"
#include "rules.hpp"
#include "vector.hpp"

// test should be compiled with: g++ vector.cpp boids.cpp boids.test.cpp

TEST_CASE("Testing Boids Project") {
  SUBCASE("Testing Vectors") {}
  SUBCASE("Testing Boids") {}
  SUBCASE("Testing rules") {}
}

Boid b1{Vector{1, 1}, Vector{0, 0}};
Boid b2{Vector{5, 5}, Vector{0, 0}};
Boid b3{Vector{10, 10}, Vector{0, 0}};
Boid b4{Vector{10, 0}, Vector{0, 0}};

Boid b5{Vector{8, 7}, Vector{0, 0}};
Boid b6{Vector{2, 3}, Vector{0, 0}};
Boid b7{Vector{4, 4}, Vector{0, 0}};
Boid b8{Vector{1.6, 3.8}, Vector{0, 0}};

Options sp{10., 0.6, 0.4, 0.1, 0.2};

Flock f1{std::vector<Boid>{b1, b2, b3, b4}, sp, 45};
Flock f2{std::vector<Boid>{b5, b6, b7, b8}, sp, 45};  

