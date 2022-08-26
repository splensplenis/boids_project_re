#ifndef RULES_HPP
#define RULES_HPP

#include "boids.hpp"
#include "vector.hpp"
#include <cmath>
#include <algorithm>

inline double distance(Boid const& boid1, Boid const& boid2) {
  // scalar distance between boids: magnitude of distance vector
  double d = sqrt(norm2(boid1.position - boid2.position));
  return d;
}
inline double speed(Boid const& boid) {
  // magnitude of boid's velocity
  double s = sqrt(norm2(boid.velocity));
  return s;
}
inline Vector applied_distance(Boid const& boid1, Boid const& boid2) {
  // distance vector between boids, applied to boid1 towards boid2
  Vector diff = boid2.position - boid1.position;
  return diff;
}

/*
//control of neighbourhood:
inline bool are_neighbours(Options const& boids_options, Boid const& boid1,
                           Boid const& boid2) {
  return (distance(boid1, boid2) <= boids_options.distance);
}*/
inline bool is_member(std::vector<Boid> const& boids, Boid const& boid) {
  auto member = std::find(boids.begin(), boids.end(), boid);
  if (member != boids.end()) {
    return true;
  }
  return false;
}
inline std::vector<Boid> get_neighbours_of(Flock const& flock,
                                           Boid const& boid) {
  auto boids = flock.get_boids();
  auto boids_options = flock.get_options();
  /*if (is_member(flock, boid) == false) {
    throw std::runtime_error{"Boid is not in the flock"};
  }*/
  std::vector<Boid> neighbours{};
  for (double i{}; i != flock.size(); ++i) {
    if ((boids[i]) != boid && distance(boid, boids[i]) <= boids_options.distance) {  //i.e. are_neighbours(boids_options, boid, boids[i])
      neighbours.push_back(boids[i]);
    }
  }
  return neighbours;
}
inline std::vector<Boid> view_neighbours(Flock const& flock, Boid const& boid) {
  auto boids = flock.get_boids();
  double pi = std::acos(-1.0);
  std::vector<Boid> view_neighbours{};
  std::vector<Boid> neighbours = get_neighbours_of(flock, boid);
  for (double i{}; i != neighbours.size();
       ++i) {  // scalar product gives the angle
    double theta =
        std::acos((boid.velocity * applied_distance(boid, neighbours[i])) /
                  (speed(boid) * distance(boid, neighbours[i])));
    theta = 180 * theta / pi;
    double alpha = flock.get_alpha();  // alpha is half the plain angle!
    if (theta <= alpha) {
      view_neighbours.push_back(neighbours[i]);
    }
  }
  return view_neighbours;
}

//rules regolating boids movement:
inline Vector separation(Options const& boid_options, Boid const& boid,
                         std::vector<Boid> neighbours) {
  Vector partial_sum{};
  std::for_each(neighbours.begin(), neighbours.end(), [&](Boid const& boid1) {
    if (distance(boid1, boid) <= boid_options.separation_distance) {
      partial_sum += applied_distance(boid, boid1);
    }
  });
  Vector corrected_velocity = partial_sum * (-boid_options.separation);
  return corrected_velocity;
}
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
inline void speed_control(Boid& boid) {
  double max_speed = 8.;                 // should not use hard-coded numbers!
  // double max_speed = velocity_parameters(flock).x() + 3 *
  // velocity_parameters(flock).y(); double min_speed =
  // velocity_parameters(flock).x() - 3 *velocity_parameters(flock).y();
  double min_speed = 2.;
  if (speed(boid) > max_speed) {
    boid.velocity /= speed(boid);
    boid.velocity *= max_speed;
  }
  if (speed(boid) < min_speed) {
    boid.velocity /= speed(boid);
    boid.velocity *= min_speed;
  }
}

//behaviour at edges: see graphics.cpp

#endif

// COMPORTAMENTO AI BORDI: "rimbalzo elastico"
// se la posizione del boid supera il bordo alla successiva iterazione,
// lo si riporta al bordo con velocità opposta (come se avesse urtato)

// funzione che allontana dal bordo con spinta opposta:

/*inline void keep_centred(Ambient const& ambient, Boid& boid) {
  //...
}

inline void avoid_boundaries(Ambient const& ambient, Boid& boid) {
  if ((boid.position).x() > (ambient.bottom_right_corner).x()) {
    Vector v1{(ambient.bottom_right_corner).x(), (boid.position).y()};
    // in realtà anche y non è quella...approssimazione
    Vector v2{-(boid.velocity).x(), (boid.velocity).y()};
    boid.position = v1;
    boid.velocity = v2;
  }
  if ((boid.position).y() > (ambient.bottom_right_corner).y()) {
    Vector v1{(boid.position).x(), (ambient.bottom_right_corner).y()};
    Vector v2{(boid.velocity).x(), -(boid.velocity).y()};
    boid.position = v1;
    boid.velocity = v2;
  }
  if ((boid.position).x() < (ambient.top_left_corner).x()) {
    Vector v1{(ambient.top_left_corner).x(), (boid.position).y()};
    Vector v2{-(boid.velocity).x(), (boid.velocity).y()};
    boid.position = v1;
    boid.velocity = v2;
  }
  if ((boid.position).y() < (ambient.top_left_corner).y()) {
    Vector v1{(boid.position).x(), (ambient.top_left_corner).y()};
    Vector v2{(boid.velocity).x(), -(boid.velocity).y()};
    boid.position = v1;
    boid.velocity = v2;
  }
}*/

