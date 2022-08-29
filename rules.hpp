#ifndef RULES_HPP
#define RULES_HPP

#include "boids.hpp"
#include "vector.hpp"
#include <cmath>
#include <algorithm>

// module of the distance between boids
inline double distance(Boid const& boid1, Boid const& boid2) {
  double d = sqrt(norm2(boid1.position - boid2.position));
  return d;
}

// module of the speed vector
inline double speed(Boid const& boid) {  
  double s = sqrt(norm2(boid.velocity));
  return s;
}

// vector which represent the distance between boids, direction is from b1 to b2
inline Vector applied_distance(Boid const& boid1, Boid const& boid2) {
  Vector diff = boid2.position - boid1.position;
  return diff;
}

//check if a boid is a member of a boid vector
inline bool is_member(std::vector<Boid> const& boids, Boid const& boid) {
  auto member = std::find(boids.begin(), boids.end(), boid);
  if (member != boids.end()) {
    return true;
  }
  return false;
}

//get all neighbours of a boid
inline std::vector<Boid> get_neighbours_of(Flock const& flock,
                                           Boid const& boid) {
  auto boids = flock.get_boids();
  auto boids_options = flock.get_options();
  std::vector<Boid> neighbours{};
  //get all boids which are within a fixed distance from the boid
  for (double i{}; i != flock.size(); ++i) {
    if ((boids[i]) != boid && distance(boid, boids[i]) <= boids_options.distance) {
      neighbours.push_back(boids[i]);
    }
  }
  return neighbours;
}

//get the neighbours that a boid can view
inline std::vector<Boid> view_neighbours(Flock const& flock, Boid const& boid) {
  auto boids = flock.get_boids();
  double pi = std::acos(-1.0);
  std::vector<Boid> view_neighbours{};
  std::vector<Boid> neighbours = get_neighbours_of(flock, boid);
  for (double i{}; i != neighbours.size();
       ++i) {  
    // scalar product gives the angle
    double theta =
        std::acos((boid.velocity * applied_distance(boid, neighbours[i])) /
                  (speed(boid) * distance(boid, neighbours[i])));
    theta = 180 * theta / pi;
    double alpha = flock.get_alpha();  // alpha is half the plain angle
    //neighbours viewed are the neighbours which its position makes an angle with the position of the considered boid 
    //smaller than the vision angle
    if (theta <= alpha) {
      view_neighbours.push_back(neighbours[i]);
    }
  }
  return view_neighbours;
}

//rules regolating boids movement:

//the nearby boids apply on the considered boid a speed change proportional to their distance towards him
inline Vector separation(Options const& boid_options, Boid const& boid,
                         std::vector<Boid> const& neighbours) {
  Vector partial_sum{};
  std::for_each(neighbours.begin(), neighbours.end(), [&](Boid const& boid1) {
    if (distance(boid1, boid) <= boid_options.separation_distance) {
      partial_sum += applied_distance(boid, boid1);
    }
  });
  Vector corrected_velocity = partial_sum * (-boid_options.separation);
  return corrected_velocity;
}

//from the mean of the nearby boids is subtracted the speed of the considered boid multiplied by a fixed factor
inline Vector alignment(Options const& boid_options, Boid const& boid,
                        std::vector<Boid> neighbours) {
  if (neighbours.size() != 0) {
    Vector sum_velocity{};
    std::for_each(neighbours.begin(), neighbours.end(),
                  [&](Boid const& boid1) { sum_velocity += boid1.velocity; });
    Vector corrected_velocity =
        (sum_velocity / neighbours.size() - boid.velocity) *
        boid_options.alignment;
    return corrected_velocity;
  } else {
    return Vector{0, 0};
  }
}

//at the considered boid position is subtracted the centre of mass position of the nearby boids multiplied by a fixed factor
inline Vector cohesion(Options const& boid_options, Boid const& boid,
                       std::vector<Boid> neighbours) {
  if (neighbours.size() != 0) {
    Vector partial_sum{};
    std::for_each(neighbours.begin(), neighbours.end(),
                  [&](Boid const& boid1) { partial_sum += boid1.position; });
    Vector centre_of_mass = partial_sum / neighbours.size();
    Vector corrected_velocity =
        (centre_of_mass - boid.position) * boid_options.cohesion;
    return corrected_velocity;
  } else {
    return Vector{0, 0};
  }
}

/* if the boid goes too much slow or too much fast:
    - its velocity components are divided by the module of the speed
    - its velocity components are multiplied by min/max speed
  Doing so the boid accelerates/decelerates smoothly if he goes above or beyond these limits */
inline void speed_control(Boid& boid, double max_speed,double min_speed) {
/*   double max_speed = 200.;                
  double min_speed = 20.; */
  if (speed(boid) > max_speed) {
    boid.velocity /= speed(boid);
    boid.velocity *= max_speed;
  }
  if ((speed(boid) < min_speed) && (speed(boid)!=0)) {
    boid.velocity /= speed(boid);
    boid.velocity *= min_speed;
  }
}

#endif

