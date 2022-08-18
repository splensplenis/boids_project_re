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

 public:
  Flock(std::vector<Boid>, Species);
  // Flock(Flock const&);
  int size() const;
  std::vector<Boid> get_flock() const;
  Species get_species() const;
  void add(Boid const&);
  // operator[] () { return flock_[];}
  void evolve(Ambient, double);
};

/*
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

// evolution in time and its parameters:
//Flock evolve(Flock, double);
Vector distance_parameters(Flock);
Vector velocity_parameters(Flock);
*/

#endif