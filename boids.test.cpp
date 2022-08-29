#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
#include "multiflock.hpp"
#include "rules.hpp"
#include "vector.hpp"

// to test boids, compile with:
// g++ -Wall -Wextra -fsanitize=address vector.cpp boids.cpp boids.test.cpp

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
  SUBCASE("Testing distance, speed and applied_distance functions") {
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
  Boid b9{Vector{1.3, 2.1}, Vector{-1, -1}};

  std::vector<Boid> boids{};
  Options boids_options{3., 0.5, 0.3, 0.1, 0.8};
  double alpha{90.};
  Flock flock{boids, boids_options, alpha};
  SUBCASE("Testing is_member") {
    flock.add(b1);
    flock.add(b3);
    CHECK(is_member(flock.get_boids(), b2) == true);
    // note: since b1 and b2 are "different" boids (built with same values)
    // it would be desirable that the test is false;
    // however, since the only important thing is how they behave in a flock,
    // in this perspective b1 and b2 are actually the same
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
  }
  SUBCASE("Testing separation") {
    flock.add(b1);
    flock.add(b4);
    flock.add(b5);
    Vector velocity = separation(boids_options, b1, view_neighbours(flock, b1));
    Vector v1{0., 0.};  // boids do not affect b1 (dist > separation_dist)
    CHECK(velocity == v1);
    flock.add(b8);
    velocity = separation(boids_options, b1, view_neighbours(flock, b1));
    // with two boids
    CHECK(velocity == Vector{0., -0.15});
    Flock flock2{std::vector<Boid>{b1, b7, b8}, boids_options, 180.};
    flock2.add(b9);
    Vector velocity_b1 =
        separation(boids_options, b1, view_neighbours(flock2, b1));
    Vector velocity_b8 =
        separation(boids_options, b8, view_neighbours(flock2, b8));
    // with three boids
    CHECK(velocity_b1.x() == doctest::Approx(-0.09));
    CHECK(velocity_b1.y() == doctest::Approx(-0.18));
    CHECK(velocity_b8.x() == doctest::Approx(-0.09));
    CHECK(velocity_b8.y() == doctest::Approx(0.27));
  }
  SUBCASE("Testing alignment") {
    flock.add(b1);
    flock.add(b4);
    flock.add(b5);
    flock.add(b8);
    Vector velocity = alignment(boids_options, b1, view_neighbours(flock, b1));
    Vector v{-0.06666, 0.};
    CHECK(doctest::Approx(velocity.x()) == v.x());
    flock.add(b7);
    Vector velocity2 = alignment(boids_options, b1, view_neighbours(flock, b1));
    CHECK(velocity == velocity2);
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
  SUBCASE("Testing speed control") {
    Boid fast_boid{Vector{0., 0.}, Vector{10., 8.}};
    Boid slow_boid{Vector{0., 0.}, Vector{0.1, 0.1}};
    Boid still_boid{Vector{0., 0.}, Vector{0., 0.}};
    speed_control(fast_boid, 8, 2);
    speed_control(slow_boid, 8, 2);
    speed_control(still_boid, 8, 2);
    CHECK(speed(fast_boid) < norm2(Vector{10., 8.}));
    CHECK(speed(slow_boid) > norm2(Vector{0.1, 0.1}));
    CHECK(speed(still_boid) == 0.);
  }
}
TEST_CASE("Testing flock and multiflock classes") {
  Boid b1{Vector{1., 2.}, Vector{1., 1}};
  Boid b2{Vector{5., 5.}, Vector{1., 2.}};
  Boid b3{Vector{3., 0.}, Vector{0., -1.}};
  Boid b4{Vector{3., 2.}, Vector{-1., 2.}};
  Boid b5{Vector{0., 2.}, Vector{0., 0.}};
  Boid b6{Vector{2., 3.}, Vector{4., 2.}};
  Boid b7{Vector{6., 3.}, Vector{1.5, 3.}};
  Boid b8{Vector{5., 4.}, Vector{4., 5.}};

  Options boid_options_def{};
  Options boids_options_1{3., 0.5, 0.3, 0.1, 0.8};
  double alpha_1{90.};

  SUBCASE("Testing Options class") {
    CHECK(boids_options_1.distance == 3.);
    CHECK(boid_options_def.distance == 0.);
    CHECK(boid_options_def.separation_distance == 0.);
    CHECK(boid_options_def.separation == 0.);
    CHECK(boid_options_def.alignment == 0.);
    CHECK(boid_options_def.cohesion == 0.);

    // assert da testare??
  }
  SUBCASE("Testing 3-parameters flock ctor") {
    Flock flock{std::vector<Boid>{b1, b3, b4}, boids_options_1, alpha_1};
    CHECK(flock.size() == 3);
    flock.add(b2);
    CHECK(flock.size() == 4);
    auto boids = flock.get_boids();
    CHECK(boids[0] == b1);
    CHECK(boids[3] == b2);
    CHECK(flock.get_alpha() == 90.);
    std::vector<Boid> empty_boids{};
    Flock flock2{empty_boids, boids_options_1, alpha_1};
    auto boids_vector = flock2.get_boids();
    bool is_empty = (boids_vector.begin() == boids_vector.end());
    CHECK(flock2.size() == 0);
    CHECK(is_empty == true);
  }
  SUBCASE("Testing 2-paramerers flock ctor") {
    Flock flock{std::vector<Boid>{b1, b3, b4}, boids_options_1};
    CHECK(flock.size() == 3);
    CHECK(flock.get_alpha() == 180.);
  }
  SUBCASE("Testing flock evolve with 180 degree view") {
    Options op{10, 1.5, 0.3, 0.1, 0.3};
    Flock myflock{std::vector<Boid>{b6, b7, b8}, op, 180};
    // 180 degrees angle, all the boids are in view region

    double delta_t = 0.1;
    myflock.evolve(delta_t, 8, 2);
    myflock.evolve(delta_t, 8, 2);
    myflock.evolve(delta_t, 8, 2);
    myflock.evolve(delta_t, 8, 2);

    auto boid_a = (myflock.get_boids())[0];
    auto boid_b = (myflock.get_boids())[1];
    auto boid_c = (myflock.get_boids())[2];
    CHECK(speed(boid_a) <= 8.);
    CHECK(speed(boid_b) >= 2.);
    CHECK((boid_a.position).x() == doctest::Approx(4.08298));
    CHECK((boid_a.position).y() == doctest::Approx(4.01131));
    CHECK((boid_b.position).x() == doctest::Approx(6.49403));
    CHECK((boid_b.position).y() == doctest::Approx(4.12935));
    CHECK((boid_c.position).x() == doctest::Approx(6.22299));
    CHECK((boid_c.position).y() == doctest::Approx(5.85934));
    CHECK((boid_a.velocity).x() == doctest::Approx(6.59196));
    CHECK((boid_a.velocity).y() == doctest::Approx(3.379694));
    CHECK((boid_b.velocity).x() == doctest::Approx(0.83436));
    CHECK((boid_b.velocity).y() == doctest::Approx(2.939825));
    CHECK((boid_c.velocity).x() == doctest::Approx(2.07368));
    CHECK((boid_c.velocity).y() == doctest::Approx(3.680481));
  }
  SUBCASE("Testing flock evolve with different angle") {
    Options op{8, 1, 0.3, 0.8, 0.8};
    Boid b9{Vector{1., 10.}, Vector{2., 0.}};
    Boid b10{Vector{1., 4.}, Vector{2., 0.}};
    Flock entire_view_flock{std::vector<Boid>{b9, b10}, op, 180};
    double delta_t = 1.;
    entire_view_flock.evolve(delta_t, 8, 2);
    entire_view_flock.evolve(delta_t, 8, 2);
    Boid boid9 = (entire_view_flock.get_boids())[0];
    Boid boid10 = (entire_view_flock.get_boids())[1];
    Vector later_b9 = boid9.position;
    Vector later_b10 = boid10.position;
    CHECK(later_b9.y() != doctest::Approx(10.));
    CHECK(later_b10.y() != doctest::Approx(4.));
    // these boids follow a parallel horizontal motion, since their angle of
    // view is 180 they see each other and modify their vertical position to
    // reach eachother;

    Flock partial_view_flock{std::vector<Boid>{b9, b10}, op, 45};
    partial_view_flock.evolve(delta_t, 8, 2);
    partial_view_flock.evolve(delta_t, 8, 2);
    Boid other_boid9 = (partial_view_flock.get_boids())[0];
    Boid other_boid10 = (partial_view_flock.get_boids())[1];
    Vector other_later_b9 = other_boid9.position;
    Vector other_later_b10 = other_boid10.position;
    CHECK(other_later_b9.y() == doctest::Approx(10.));
    CHECK(other_later_b10.y() == doctest::Approx(4.));
    // instead, the same boids with only 45 degree angle of view
    // will never see eachother and not change their vertical position
    // accordingly;
  }
  SUBCASE("Testing multiflock ctor") {
    Flock flock1{std::vector<Boid>{b1, b2, b3}, boids_options_1, alpha_1};
    Flock flock2{std::vector<Boid>{b4, b5, b6}, boids_options_1, alpha_1};
    Flock empty_flock{std::vector<Boid>{}, boids_options_1, alpha_1};
    MultiFlock multi{std::vector<Flock>{flock1, flock2}};
    CHECK(multi.size() == 2);
    multi.add(empty_flock);
    CHECK(multi.size() == 3);
    std::vector<Boid> boid_vector1{b1, b2, b3, b4, b5, b6};
    std::vector<Boid> boid_vector2{b2, b4, b1, b3, b5, b6};
    CHECK(multi.get_all_boids() == boid_vector1);
    bool confront = (multi.get_all_boids() == boid_vector2);
    CHECK(confront == false);
  }
  SUBCASE("Testing get_other_neighbours") {
    Options boids_options_2{10, 1, 0.3, 0.1, 0.3};
    Flock flock1{std::vector<Boid>{b1}, boids_options_2, alpha_1};
    Flock flock2{std::vector<Boid>{b2, b3, b4, b5}, boids_options_2, alpha_1};
    Flock flock3{std::vector<Boid>{b6, b7, b8}, boids_options_2, alpha_1};
    MultiFlock multi{std::vector<Flock>{flock1, flock2, flock3}};
    auto neigh = get_other_neighbours(multi, b1);
    std::vector<Boid> boids{b2, b3, b4, b5, b6, b7, b8};
    CHECK(neigh == boids);
  }
  SUBCASE("Testing multiflock evolve") {
    Boid b1{Vector{1., 2.}, Vector{1., 1}};
    Boid b2{Vector{5., 5.}, Vector{1., 2.}};
    Boid b3{Vector{3., 0.}, Vector{0., -1.}};
    Boid b4{Vector{3., 2.}, Vector{-1., 2.}};
    Boid b5{Vector{0., 2.}, Vector{0., 0.}};
    Boid b6{Vector{2., 3.}, Vector{4., 2.}};
    Boid b7{Vector{6., 3.}, Vector{1.5, 3.}};
    Boid b8{Vector{5., 4.}, Vector{4., 5.}};

    double delta_t = 0.5;
    Options op{20, 2.5, 12, 0.2, 0.3};
    // very high separation coefficient to make the effect evident

    Flock flock1{std::vector<Boid>{b1, b3, b4, b5, b6}, op, alpha_1};
    Flock flock2{std::vector<Boid>{b2, b7, b8}, op, alpha_1};
    MultiFlock multi1{std::vector<Flock>{flock1}};
    MultiFlock multi2{std::vector<Flock>{flock1, flock2}};
    for (int steps{}; steps != 100; ++steps) {
      multi1.evolve(delta_t, 8, 2);
    }
    auto flock_status = multi1.get_all_boids();
    for (int steps{}; steps != 100; ++steps) {
      multi2.evolve(delta_t, 8, 2);
    }
    auto flock_status_2 = ((multi2.get_flocks())[0].get_boids());
    CHECK(flock_status.size() == flock_status_2.size());
    CHECK(flock_status != flock_status_2);
    // f1 behaves differently when evolves in m1 (where it is the only flock)
    // and when evolves in m2 (two flocks, following separation rule between
    // them)
  }
  SUBCASE("Testing statistics functions") {
    Flock empty_f{std::vector<Boid>{}, boids_options_1, alpha_1};
    MultiFlock empty_multi{std::vector<Flock>{}};
    CHECK(empty_f.get_distance_mean_RMS().x() == 0.);
    CHECK(empty_f.get_distance_mean_RMS().y() == 0.);
    CHECK(empty_f.get_speed_mean_RMS() == Vector{0., 0.});
    empty_f.add(b1);
    Vector res{speed(b1), 0.};
    CHECK(empty_f.get_speed_mean_RMS() == res);
  }
}