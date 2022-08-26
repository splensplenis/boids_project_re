#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
#include "multiflock.hpp"
#include "rules.hpp"
#include "vector.hpp"

// test should be compiled with: g++ vector.cpp boids.cpp boids.test.cpp

TEST_CASE("Testing Boids") {
  SUBCASE("Testing operator== and operator!= ") {
    Boid b1{Vector{1., 2.}, Vector{1., 1.}};
    Boid b2{Vector{1., 2.}, Vector{1., 1.}};
    Boid b3{Vector{5., 5.}, Vector{1., 2.}};
    CHECK(b1 == b2);
    CHECK(b1 != b3);
  }
  SUBCASE("Testing defualt initialisation") {
    Vector zero{};
    Boid b0{};
    CHECK(b0.position == zero && b0.velocity == zero);
  }
  SUBCASE("Testing dist, speed and appl dist functions") {
    Boid b1{Vector{1., 1.}, Vector{0., 0.}};
    Boid b2{Vector{2., 2.}, Vector{-3., -9.}};
    CHECK(distance(b1, b2) == distance(b2, b1));
    CHECK(distance(b1, b2) == doctest::Approx(1.4142).epsilon(0.0001));
    CHECK(speed(b1) == 0);
    CHECK(speed(b2) == doctest::Approx(9.487).epsilon(0.001));
    Vector v = applied_distance(b1, b2);
    CHECK(v.x() == 1.);
    CHECK(v.y() == 1.);
    Vector w = applied_distance(b2, b1);
    CHECK(w.x() == -1.);
    CHECK(w.y() == -1.);
  }
}
TEST_CASE("Testing rules") {
  Boid b1{Vector{1., 2.}, Vector{1., 0}};
  Boid b2{Vector{1., 2.}, Vector{1., 0}};
  Boid b3{Vector{5., 5.}, Vector{1., 2.}};
  Boid b4{Vector{3., 0.}, Vector{0., -1.}};
  Boid b5{Vector{3., 2.}, Vector{-1., 2.}};
  Boid b6{Vector{0., 0.}, Vector{0., 0.}};
  Boid b7{Vector{0., 2.}, Vector{0., 0.}};
  Boid b8{Vector{1., 2.5}, Vector{2., 2.}};

  std::vector<Boid> boids{};
  Options boids_options{3., 0.5, 0.3, 0.1, 0.8};  // 0.8 invece di 1 per ultimo
  double alpha{90.};
  Flock flock{boids, boids_options, alpha};
  SUBCASE("Testing is_member") {
    flock.add(b1);
    flock.add(b3);
    CHECK(is_member(flock.get_boids(), b2) == true);
    // b1 e b2 hanno stesse coord quindi dice che b2 c'è, va bene?
    // direi di sì, alla fine se b2 "appartiene" al flock significa che
    // si comporta esattamente come b1 e quindi chiamando b2 dopo un evolve
    // equivale a chiamare b1 evoluto nel suo flock
    CHECK((is_member(flock.get_boids(), b4)) == false);
  }
  SUBCASE("Testing get_neighbours_of") {
    flock.add(b1);
    flock.add(b3);
    flock.add(b4);
    std::vector<Boid> neighbours = get_neighbours_of(flock, b1);
    CHECK((is_member(neighbours, b3)) == false);
    CHECK(is_member(neighbours, b4) == true);
    flock.add(b2);
    neighbours = get_neighbours_of(flock, b1);
    CHECK((is_member(neighbours, b2)) == false);
  }
  SUBCASE("Testing view_neighbours") {
    flock.add(b1);
    flock.add(b4);
    flock.add(b5);
    flock.add(b6);
    flock.add(b7);
    std::vector<Boid> neighbours = view_neighbours(flock, b1);
    CHECK(is_member(neighbours, b4) == true);
    CHECK(is_member(neighbours, b5) == true);
    CHECK((is_member(neighbours, b6)) == false);
    CHECK((is_member(neighbours, b7)) == false);
    // cosa succede con angoli diversi?
  }
  SUBCASE("Testing separation") {
    flock.add(b4);
    flock.add(b5);
    Vector velocity = separation(boids_options, b1, view_neighbours(flock, b1));
    Vector v1{0., 0.};  // calcolato??
    CHECK(velocity == v1);
    flock.add(b8);
    velocity = separation(boids_options, b1, view_neighbours(flock, b1));
    CHECK(velocity == Vector{0., -0.15});
  }
  SUBCASE("Testing alignment") {
    flock.add(b4);
    flock.add(b5);
    flock.add(b8);
    Vector velocity = alignment(boids_options, b1, view_neighbours(flock, b1));
    Vector v{-0.06666, 0.};  // 5 cifre approssima bene
    CHECK(doctest::Approx(velocity.x()) == v.x());
    CHECK(doctest::Approx(velocity.y()) == v.y());
  }
  SUBCASE("Testing cohesion") {
    flock.add(b4);
    flock.add(b5);
    flock.add(b8);
    Vector velocity = cohesion(boids_options, b1, view_neighbours(flock, b1));
    Vector v{1.06666, -0.4};
    CHECK(doctest::Approx(velocity.x()) == v.x());
    CHECK(doctest::Approx(velocity.y()) == v.y());
  }
  SUBCASE("Testing speed control") {}
}
TEST_CASE("Testing flock and multiflock classes") {
  SUBCASE("Testing 3-parameters flock ctor") {}
  SUBCASE("Testing 2-paramerers flock ctor") {}
  // se è vuoto, che cosa restituisce get boids?
  SUBCASE("Testing flock evolve") {}
  SUBCASE("Testing multiflock ctor") {}
  SUBCASE("Testing multiflock evolve") {}
  SUBCASE("Testing get statistics") {}
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