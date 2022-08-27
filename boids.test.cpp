#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
#include "multiflock.hpp"
#include "rules.hpp"
#include "vector.hpp"


// to test boids, compile with: g++ -Wall -Wextra -fsanitize=address vector.cpp boids.cpp boids.test.cpp


TEST_CASE("Testing Boids") {
  SUBCASE("Testing == and !=") {
    Boid b1{Vector{1., 2.}, Vector{1., 1.}};
    Boid b2{Vector{1., 2.}, Vector{1., 1.}};
    Boid b3{Vector{5., 5.}, Vector{1., 2.}};
    CHECK(b1 == b2);
    CHECK(b1 != b3);
  }
  SUBCASE("Testing defualt initialisation") {
    Vector zero{};
    Boid b0{};
    CHECK(b0.position == zero);
    CHECK(b0.velocity == zero);
  }
  SUBCASE("Testing distance, speed and appllied_distance functions") {
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
  Boid b1{Vector{1., 2.}, Vector{1., 1.}};
  Boid b2{Vector{1., 2.}, Vector{1., 1.}};
  Boid b3{Vector{5., 5.}, Vector{1., 2.}};
  Boid b4{Vector{3., 0.}, Vector{0., -1.}};
  Boid b5{Vector{3., 2.}, Vector{-1., 2.}};
  Boid b6{Vector{0., 0.}, Vector{0., 0.}};
  Boid b7{Vector{0., 2.}, Vector{0., 0.}};
  Boid b8{Vector{1., 2.5}, Vector{2., 2.}};
  Boid b9{Vector{1.3, 2.1}, Vector{-1, -1}};

  std::vector<Boid> boids{};
  Options boids_options{3., 0.5, 0.3, 0.1, 0.8};
  double alpha{90.};
  Flock flock{boids, boids_options, alpha};
  SUBCASE("Testing is_member") {
    flock.add(b1);
    flock.add(b3);
    CHECK(is_member(flock.get_boids(), b1) == true);
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
    // alpha uguale a zero non vede nessuno, è giusto cosi?
    /*Flock flock_i{boids, boids_options, 0.};
    Boid b{Vector{2., 3.}, Vector{0., 0.}};
    flock_i.add(b1);
    flock_i.add(b);
    std::vector<Boid> neighbours = get_neighbours_of(flock_i, b1);
    std::vector<Boid> neighbours_1 = view_neighbours(flock_i, b1);
    CHECK(neighbours.size() == 1);
    CHECK(is_member(neighbours_1, b));*/
  }
  SUBCASE("Testing separation") {
    //flock.add(b1);
    flock.add(b4);
    flock.add(b5);
    Vector velocity = separation(boids_options, b1, view_neighbours(flock, b1));
    Vector v1{0., 0.};  // boids do not affect b1 (dist > separation_dist)
    CHECK(velocity == v1);
    flock.add(b8);
    velocity = separation(boids_options, b1, view_neighbours(flock, b1));
    CHECK(velocity == Vector{0., -0.15});  // with two boids
    Flock flock2{std::vector<Boid>{b1, b7, b8}, boids_options, 180.};
    //flock2.add(b9);
    Vector velocity_b1 =
        separation(boids_options, b1, view_neighbours(flock2, b1));
    Vector velocity_b8 =
        separation(boids_options, b8, view_neighbours(flock2, b8));
    CHECK(velocity_b1.x() == doctest::Approx(-0.09));
    CHECK(velocity_b1.y() == doctest::Approx(-0.18));  // with three boids
    CHECK(velocity_b8.x() == doctest::Approx(-0.09));
    CHECK(velocity_b8.y() == doctest::Approx(0.27));
  }
  SUBCASE("Testing alignment") {
    //flock.add(b1);
    flock.add(b4);
    flock.add(b5);
    flock.add(b8);
    Vector velocity = alignment(boids_options, b1, view_neighbours(flock, b1));
    Vector v{-0.06666, 0.};
    CHECK(doctest::Approx(velocity.x()) == v.x());
    flock.add(b7);
    Vector velocity2 = alignment(boids_options, b1, view_neighbours(flock, b1));
    CHECK(velocity == velocity2); //bool true ti basta scrivere cosi
    // adding b7 to the flock has not change b1 behaviour
    // because b7 is not seen by b1
  }
  SUBCASE("Testing cohesion") {
    flock.add(b1);
    flock.add(b4);
    flock.add(b5);
    flock.add(b8);
    Vector velocity = cohesion(boids_options, b1, view_neighbours(flock, b1));
    Vector v{1.06666, -0.4};
    CHECK(doctest::Approx(velocity.x()) == v.x());
    CHECK(doctest::Approx(velocity.y()) == v.y());
  }
  SUBCASE("Testing speed control") {}
  SUBCASE("G") {
    Flock flock_i{boids, boids_options};   //!!!!!!!!!!!!!!
    CHECK(flock_i.get_alpha() == 180.);  
  }
}
TEST_CASE("Testing flock and multiflock classes") {
  SUBCASE("Testing 3-parameters flock ctor") {}
  SUBCASE("Testing flock evolve") {
    /*
    Options op{10, 0.1, 0.9, 0.1, 0.3};
    Flock myflock{std::vector<Boid>{b1,b3,b4}, op , 180};

    double delta_t = 0.2;
    myflock.evolve(delta_t);
    auto boid_a = (myflock.get_boids())[0];
    auto boid_b = (myflock.get_boids())[1];
    auto boid_c = (myflock.get_boids())[2];
    CHECK((boid_a.position).x() == doctest::Approx(1.2));
    CHECK((boid_a.position).y() == doctest::Approx(2.2));
    CHECK((boid_b.position).x() == doctest::Approx(5.2));
    CHECK((boid_b.position).y() == doctest::Approx(5.4));
    CHECK((boid_c.position).x() == doctest::Approx(3));
    CHECK((boid_c.position).y() == doctest::Approx(-0.2));
    CHECK((boid_a.velocity).x() == doctest::Approx(1.85));
    CHECK((boid_a.velocity).y() == doctest::Approx(1.1));
    CHECK((boid_b.velocity).x() == doctest::Approx(0.05));
    CHECK((boid_b.velocity).y() == doctest::Approx(0.6));
    CHECK((boid_c.velocity).x() == doctest::Approx(0.1));
    CHECK((boid_c.velocity).y() == doctest::Approx(0.3));
    */

    /*Boid boid_a{Vector{2., 1.}, Vector{0., 1.}};
    Boid boid_b{Vector{4., 3.}, Vector{-1., 0.}};
    Boid boid_c{Vector{1., 4.}, Vector{2., 0.5}};

    Options op{10, 0.1, 0.9, 0.1, 0.3};

    Flock flock{std::vector<Boid>{boid_a, boid_b, boid_c}, op};

    double delta_t = 0.2;
    flock.evolve(delta_t);
    boid_a = (flock.get_boids())[0];
    boid_b = (flock.get_boids())[1];
    boid_c = (flock.get_boids())[2];
    CHECK((boid_a.position).x() == doctest::Approx(2.));
    CHECK((boid_a.position).y() == doctest::Approx(1.2));
    CHECK((boid_b.position).x() == doctest::Approx(3.8));
    CHECK((boid_b.position).y() == doctest::Approx(3));
    CHECK((boid_c.position).x() == doctest::Approx(1.4));
    CHECK((boid_c.position).y() == doctest::Approx(4.1));
    CHECK((boid_a.velocity).x() == doctest::Approx(0.2));
    CHECK((boid_a.velocity).y() == doctest::Approx(1.675));
    CHECK((boid_b.velocity).x() == doctest::Approx(-1.55));
    CHECK((boid_b.velocity).y() == doctest::Approx(-0.075));
    CHECK((boid_c.velocity).x() == doctest::Approx(2.35));
    CHECK((boid_c.velocity).y() == doctest::Approx(-0.1));
    */
  }
  // se è vuoto, che cosa restituisce get boids? vettore di boids vuoto suppongo
  SUBCASE("Testing multiflock ctor") {}
  SUBCASE("Testing multiflock evolve") {}
  SUBCASE("Testing get statistics") {}
  
}
