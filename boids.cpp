#include "boids.hpp"

#include <cassert>
#include <iostream>

#include "cassert"
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
    : boids_{boids}, boids_options_{boids_options}, alpha_{alpha} {
  assert((boids_options.distance > 0.) &&
         (boids_options.separation_distance > 0.) &&
         (boids_options.separation > 0.) && (boids_options.alignment > 0.) &&
         (boids_options.cohesion > 0.) && (alpha > 0.));
}
Flock::Flock(std::vector<Boid> const& boids, Options const& boids_options)
    : boids_{boids}, boids_options_{boids_options} {
  assert((boids_options.distance > 0.) &&
         (boids_options.separation_distance > 0.) &&
         (boids_options.separation > 0.) && (boids_options.alignment > 0.) &&
         (boids_options.cohesion > 0.));
}
int Flock::size() const { return boids_.size(); }
std::vector<Boid> Flock::get_boids() const { return boids_; }
Options Flock::get_options() const { return boids_options_; }
double Flock::get_alpha() const { return alpha_; }
void Flock::add(Boid const& boid) { boids_.push_back(boid); }
void Flock::evolve(double delta_t) {  // Ambient const& amb,
  std::vector<Boid> copy{boids_};
  for (int i{}; i != this->size(); ++i) {
    // corrections read from copy (old state)
    // and written to flock (updated state);
    auto boid = boids_[i];
    auto boid_copied = copy[i];
    boid.velocity += (separation(boids_options_, boid_copied,
                                 view_neighbours(*this, boid_copied)) +
                      alignment(boids_options_, boid_copied,
                                view_neighbours(*this, boid_copied)) +
                      cohesion(boids_options_, boid_copied,
                               view_neighbours(*this, boid_copied)));
    speed_control(boid);
    boid.position += (boid_copied.velocity * delta_t);
    // avoid_boundaries(amb, boid);
    boids_[i] = boid;
  }
}
Vector Flock::get_distance_mean_RMS() const {
  // filling a histogram with distances between all boids of the flock
  // then calculating its mean and standard deviation for a given time
  std::vector<double> dist_histo{};
  int N =
      this->size() * (this->size() - 1) /
      2;  // # of distances calculated (combinations of n boids taken 2 by 2)
  double partial_sum{};
  double partial_sum2{};
  for (int i{}; i != this->size(); ++i) {
    for (int j{i + 1}; j != this->size(); ++j) {
      double value = distance(boids_[i], boids_[j]);
      dist_histo.push_back(value);
      // this way distance bewtween boid 0 and boid 1 is calculated only once
      partial_sum2 += value;
    }
  }
  double mean_distance = partial_sum2 / N;
  for (int i{}; i != this->size(); ++i) {
    partial_sum +=
        (dist_histo[i] - mean_distance) * (dist_histo[i] - mean_distance);
  }
  double distance_RMS = partial_sum / (N - 1);
  return Vector{mean_distance, distance_RMS};
}
Vector Flock::get_speed_mean_RMS() const {
  std::vector<double> speed_histo{};
  double partial_sum{};
  double partial_sum2{};
  for (int i{}; i != this->size(); ++i) {
    double value = speed(boids_[i]);
    speed_histo.push_back(value);
    partial_sum2 += value;
  }
  double mean_speed = partial_sum2 / this->size();
  for (int i{}; i != this->size(); ++i) {
    partial_sum +=
        (speed_histo[i] - mean_speed) * (speed_histo[i] - mean_speed);
  }
  double speed_RMS = partial_sum / (this->size() - 1);
  return Vector{mean_speed, speed_RMS};
}