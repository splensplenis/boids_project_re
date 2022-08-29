#include "boids.hpp"

#include <iostream>
#include<stdexcept>
#include<cassert>
#include<numeric>

#include "rules.hpp"
#include "vector.hpp"

bool operator==(Boid const& boid1, Boid const& boid2) {
  return (boid1.position == boid2.position && boid1.velocity == boid2.velocity);
}
bool operator!=(Boid const& boid1, Boid const& boid2) {
  return !(boid1 == boid2);
}

Flock::Flock(std::vector<Boid> const& boids, Options const& boids_options,
             double alpha)
    : boids_{boids},
      boids_options_{boids_options},
      alpha_{alpha} {
        if (boids_options_.alignment < 0){
          throw std::invalid_argument{"alignment cannot be less than 0"};
        }
        if (boids_options_.cohesion < 0){
          throw std::invalid_argument{"cohesion cannot be less than 0"};
        }
        if (boids_options_.separation < 0){
          throw std::invalid_argument{"separation cannot be less than 0"};
        }
        if (boids_options_.separation_distance < 0){
          throw std::invalid_argument{"separation_distance cannot be less than 0"};
        }
        if (boids_options_.distance < 0){
          throw std::invalid_argument{"distance cannot be less than 0"};
        }
        if (alpha_ < 0 || alpha > 180){
          throw std::invalid_argument{"angle must be between 0 and 180"};
        }
        assert(boids_options_.alignment >= 0);
        assert(boids_options_.cohesion >= 0);
        assert(boids_options_.separation >= 0);
        assert(boids_options_.separation_distance >= 0);
        assert(boids_options_.distance >= 0);
        assert(alpha_ >= 0 && alpha_ <= 180);
      }  // should check values
Flock::Flock(std::vector<Boid> const& boids, Options const& boids_options)
: boids_{boids}, boids_options_{boids_options}, alpha_{180} {
        if (boids_options_.alignment < 0){
          throw std::invalid_argument{"alignment cannot be less than 0"};
        }
        if (boids_options_.cohesion < 0){
          throw std::invalid_argument{"cohesion cannot be less than 0"};
        }
        if (boids_options_.separation < 0){
          throw std::invalid_argument{"separation cannot be less than 0"};
        }
        if (boids_options_.separation_distance < 0){
          throw std::invalid_argument{"separation_distance cannot be less than 0"};
        }
        if (boids_options_.distance < 0){
          throw std::invalid_argument{"distance cannot be less than 0"};
        }  
        assert(boids_options_.alignment >= 0);
        assert(boids_options_.cohesion >= 0);
        assert(boids_options_.separation >= 0);
        assert(boids_options_.separation_distance >= 0);
        assert(boids_options_.distance >= 0);
}
int Flock::size() const { return boids_.size(); }
std::vector<Boid> Flock::get_boids() const { return boids_; }
Options Flock::get_options() const { return boids_options_; }
double Flock::get_alpha() const { return alpha_; }
void Flock::add(Boid const& boid) { boids_.push_back(boid); }
void Flock::evolve(double delta_t,double max_speed,double min_speed) {
  std::vector<Boid> copy{boids_};
  Flock old_flock{*this};
  for (int i{}; i != this->size(); ++i) {
    // corrections read from copy (old state)
    // and written to flock (updated state);
    auto boid = boids_[i];
    auto boid_copied = copy[i];
    boid.velocity += (separation(boids_options_, boid_copied,
                                 view_neighbours(old_flock, boid_copied)) +
                      alignment(boids_options_, boid_copied,
                                view_neighbours(old_flock, boid_copied)) +
                      cohesion(boids_options_, boid_copied,
                               view_neighbours(old_flock, boid_copied)));
    speed_control(boid,max_speed,min_speed);
    boid.position += (boid_copied.velocity * delta_t);
    boids_[i] = boid;
  }
}

//calculating mean distance and its variance for a given time
Vector Flock::get_distance_mean_RMS() const {
  if (this->size() > 1){
    std::vector<double> dist_holder{};
    double rms_partial_sum{};
    double mean_partial_sum{};
    for (int first_boid{}; first_boid != this->size(); ++first_boid) {
      // this way distance bewtween boid 0 and boid 1 is calculated only once
      for (int second_boid{first_boid + 1}; second_boid != this->size(); ++second_boid) {
        double value = distance(boids_[first_boid], boids_[second_boid]);
        dist_holder.push_back(value); //save distance value
        mean_partial_sum += value;
      }
    }
    double mean_distance = mean_partial_sum / this->size(); //compute mean
    for (int i{}; i != this->size(); ++i) { 
      rms_partial_sum +=
          (dist_holder[i] - mean_distance) * (dist_holder[i] - mean_distance); //sum of the squared deviation from mean
    }
    double distance_RMS = rms_partial_sum / (this->size() - 1); //compute variance
    return Vector{mean_distance, distance_RMS};
  } else {
    return Vector{0.0,0.0};
  }
}

//calculating mean velocity and its variance for a given time
Vector Flock::get_speed_mean_RMS() const {
  std::vector<double> speed_holder{};
  if (this->size() > 1){
    double rms_partial_sum{};
    double mean_partial_sum{};
    for (int boid{}; boid != this->size(); ++boid) {
      double value = speed(boids_[boid]);
      speed_holder.push_back(value); //save velocity value
      mean_partial_sum += value;
    }
    double mean_speed = mean_partial_sum / this->size(); //compute mean
    for (int i{}; i != this->size(); ++i) {
      rms_partial_sum +=
          (speed_holder[i] - mean_speed) * (speed_holder[i] - mean_speed); //sum of the squared deviation from mean
    }
    double speed_RMS = rms_partial_sum / (this->size() - 1); //compute variance
    return Vector{mean_speed, speed_RMS};
  } else {
    if (this -> size() == 0){
      return Vector{0.0,0.0};
    } else {
      return Vector{speed(boids_[0]),0.0};
    }
  }
}