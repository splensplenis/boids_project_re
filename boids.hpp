#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <vector>

#include "vector.hpp"

struct Boid {
  Vector position{};
  Vector velocity{};
};

bool operator==(Boid const&, Boid const&);
bool operator!=(Boid const&, Boid const&);

struct Options {
  double distance;
  double separation_distance;
  double separation;
  double alignment;
  double cohesion;
};

class Flock {
  std::vector<Boid> boids_{};
  Options boids_options_{};
  // cretaed from values taken from input in range (0,1)
  double alpha_{} /* = 180 */; //in degrees

 public:
  Flock(std::vector<Boid> const&, Options const&, double);
  Flock(std::vector<Boid> const&, Options const&);
  int size() const;
  std::vector<Boid> get_boids() const;
  Options get_options() const;
  double get_alpha() const;
  void add(Boid const&);
  void evolve(double, double, double); //Ambient const&, 
  Vector get_distance_mean_RMS() const;
  Vector get_speed_mean_RMS() const;
};

#endif


