#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "vector.hpp"

struct Ambient {
 Vector top_left_corner{};
 Vector bottom_right_corner{};
};

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
  double alpha_;

 public:
  Flock(std::vector<Boid> const&, Options const&, double);
  // Flock(Flock const&);
  int size() const;
  std::vector<Boid> get_boids() const;
  Options get_options() const;
  double get_alpha() const;
  void add(Boid const&);
  // operator[] () { return flock_[];}
  void evolve(Ambient, double);
};

/* ->see "rules.hpp"
double distance(Boid const&, Boid const&);
double speed(Boid const&);
Vector applied_distance(Boid const&, Boid const&);

// control of neighbourhood:
bool are_neighbours(Flock, Boid const&, Boid const&); //Flock argument should be const&?
bool is_member(Flock, Boid const&);
auto get_neighbours_of(Flock, Boid const&); //auto here stands for std::vector<Boid>
//auto view_neighbours(Flock, Boid const&);

// rules:
Vector separation(Flock, Boid const&, std::vector<Boid>);  // separation
Vector alignment(Flock, Boid const&, std::vector<Boid>);   // alignment
Vector cohesion(Flock, Boid const&, std::vector<Boid>);    // cohesion

// behaviour at edges:
Vector avoid_boundaries(Ambient, Boid&);

// evolution in time and its parameters:
//Flock evolve(Flock, double);
Vector distance_parameters(Flock);
Vector velocity_parameters(Flock);
*/

#endif