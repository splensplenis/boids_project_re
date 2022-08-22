#include "boids.hpp"

#include <iostream>

#include "rules.hpp"
#include "vector.hpp"

bool operator==(Boid const& boid1, Boid const& boid2) {
  return (boid1.position == boid2.position && boid1.velocity == boid2.velocity);
}
bool operator!=(Boid const& boid1, Boid const& boid2) {
  return !(boid1 == boid2);
}

Flock::Flock(std::vector<Boid> const& boids, Options const& boids_options,
             double alpha = 180.)
    : boids_{boids}, boids_options_{boids_options}, alpha_{alpha} {} //should check values
int Flock::size() const { return boids_.size(); }
std::vector<Boid> Flock::get_boids() const { return boids_; }
Options Flock::get_options() const { return boids_options_; }
double Flock::get_alpha() const { return alpha_; }
void Flock::add(Boid const& boid) { boids_.push_back(boid); }
void Flock::evolve(Ambient const& amb, double delta_t) {
  std::vector<Boid> copy{boids_};
  for (int i{}; i != this->size(); ++i) {
    // corrections read from copy (old state)
    // and written to flock (updated state);
    auto boid = boids_[i];
    auto boid_copied = copy[i];
    /*
    boid.velocity +=
        (separation(*this, boid_copied, get_neighbours_of(*this, boid_copied)) +
         alignment(*this, boid_copied, get_neighbours_of(*this, boid_copied)) +
         cohesion(*this, boid_copied, get_neighbours_of(*this, boid_copied)));
    */
    ///*
    boid.velocity += 
        (separation(boids_options_, boid_copied, view_neighbours(*this, boid_copied)) +
         alignment(boids_options_, boid_copied, view_neighbours(*this, boid_copied)) +
         cohesion(boids_options_, boid_copied, view_neighbours(*this, boid_copied)));
    //*/
    speed_control(*this, boid);
    boid.position += (boid_copied.velocity * delta_t);
    //keep_centred(amb, boid); 
    avoid_boundaries(amb, boid);
    boids_[i] = boid;
  }
}