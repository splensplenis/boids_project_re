#ifndef RULES_HPP
#define RULES_HPP

#include "boids.hpp"
#include "vector.hpp"

inline double distance(Boid const& boid1, Boid const& boid2) {
  // modulo del vettore distanza tra boids
  double d = sqrt(norm2(boid1.position - boid2.position));
  return d;
}
inline double speed(Boid const& boid) {  // modulo del vettore velocità
  double s = sqrt(norm2(boid.velocity));
  return s;
}
inline Vector applied_distance(Boid const& boid1, Boid const& boid2) {
  // vettore distanza tra boids
  Vector diff = boid2.position - boid1.position;
  return diff;
}
inline bool are_neighbours(Flock flock, Boid const& boid1, Boid const& boid2) {
  auto boids_options = flock.get_options();
  return (distance(boid1, boid2) <= boids_options.distance);
}
inline bool is_member(Flock flock, Boid const& boid) {
  auto boids = flock.get_boids();
  auto member = std::find(boids.begin(), boids.end(), boid);
  if (member != boids.end()) {
    return true;
  }
  return false;
}
inline auto get_neighbours_of(Flock flock, Boid const& boid) {
  auto boids = flock.get_boids();
  if (is_member(flock, boid) == false) {
    throw std::runtime_error{"Boid is not in the flock"};
  }
  std::vector<Boid> neighbours{};
  for (double i{}; i != flock.size(); ++i) {
    if ((boids[i]) != boid && are_neighbours(flock, boid, boids[i])) {
      neighbours.push_back(boids[i]);
    }
  }
  return neighbours;
}
inline auto view_neighbours(Flock flock, Boid const& boid) {
  auto boids = flock.get_boids();
  // auto boids_options = flock.get_options();
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
inline Vector separation(Flock flock, Boid const& boid,
                         std::vector<Boid> neighbours) {
  // auto flock = f.get_flock();
  auto boids_options = flock.get_options();
  Vector partial_sum{};
  std::for_each(neighbours.begin(), neighbours.end(), [&](Boid const& boid1) {
    if (distance(boid1, boid) <= boids_options.separation_distance) {
      partial_sum += applied_distance(boid, boid1);
    }
  });
  Vector v1_corr = partial_sum * (-boids_options.separation);
  return v1_corr;
}
inline Vector alignment(Flock flock, Boid const& boid,
                        std::vector<Boid> neighbours) {
  if (neighbours.size() != 0) {
    // auto flock = f.get_flock();
    auto boids_options = flock.get_options();
    Vector sum_velocity{};
    std::for_each(neighbours.begin(), neighbours.end(),
                  [&](Boid const& boid1) { sum_velocity += boid1.velocity; });
    Vector v2_corr = (sum_velocity / neighbours.size() - boid.velocity) *
                     boids_options.alignment;
    return v2_corr;
  } else {
    return Vector{0, 0};
  }
}
inline Vector cohesion(Flock flock, Boid const& boid,
                       std::vector<Boid> neighbours) {
  if (neighbours.size() != 0) {
    // auto flock = f.get_flock();
    auto boids_options = flock.get_options();
    Vector partial_sum{};
    std::for_each(neighbours.begin(), neighbours.end(),
                  [&](Boid const& boid1) { partial_sum += boid1.position; });
    Vector centre_of_mass = partial_sum / neighbours.size();
    Vector v3_corr = (centre_of_mass - boid.position) * boids_options.cohesion;
    return v3_corr;
  } else {
    return Vector{0, 0};
  }
}
inline Vector distance_parameters(Flock flock) {
  // filling a histogram with distances between all boids of the flock
  // then calculating its mean and standard deviation for a given time
  std::vector<double> dist_histo{};
  double partial_sum{};
  double partial_sum2{};
  auto boids = flock.get_boids();
  for (int i{}; i != flock.size(); ++i) {
    for (int j{i + 1}; j != flock.size(); ++j) {
      double value = distance(boids[i], boids[j]);
      dist_histo.push_back(value);
      // this way distance bewtween boid 0 and boid 1 is calculated only once
      // non dovremmo mettere un i!=j ?
      partial_sum2 += value;
    }
  }
  double mean_distance = partial_sum2 / flock.size();
  for (int i{}; i != flock.size(); ++i) {
    partial_sum +=
        (dist_histo[i] - mean_distance) * (dist_histo[i] - mean_distance);
  }
  double stddev_distance = partial_sum / (flock.size() - 1);
  return Vector{mean_distance, stddev_distance};
}
inline Vector velocity_parameters(Flock flock) {
  std::vector<double> speed_histo{};
  double partial_sum{};
  double partial_sum2{};
  auto boids = flock.get_boids();
  for (int i{}; i != flock.size(); ++i) {
    double value = speed(boids[i]);
    speed_histo.push_back(value);
    partial_sum2 += value;
  }
  double mean_speed = partial_sum2 / flock.size();
  for (int i{}; i != flock.size(); ++i) {
    partial_sum +=
        (speed_histo[i] - mean_speed) * (speed_histo[i] - mean_speed);
  }
  double stddev_speed = partial_sum / (flock.size() - 1);
  return Vector{mean_speed, stddev_speed};
}
// COMPORTAMENTO AI BORDI: "rimbalzo elastico"
// se la posizione del boid supera il bordo alla successiva iterazione,
// lo si riporta al bordo con velocità opposta (come se avesse urtato)
/*
inline bool out_of_borders(Ambient ambient, Boid& boid) {
  if ((boid.position).x() < (ambient.top_left_corner).x() ||
      (boid.position).x() > (ambient.bottom_right_corner).x())
    return true;
  if ((boid.position).y() < (ambient.top_left_corner).y() ||
      (boid.position).y() > (ambient.bottom_right_corner).y())
    return true;
}
*/
inline Boid avoid_boundaries(Ambient ambient, Boid& boid) {
  // should use bool out_of_borders?
  if ((boid.position).x() > (ambient.bottom_right_corner).x()) {
    Vector v1{(ambient.bottom_right_corner).x(), (boid.position).y()};
    // in realtà anche y non è quella...approssimazione
    Vector v2{-(boid.velocity).x(), (boid.velocity).y()};
    return Boid{v1, v2};
  }
  if ((boid.position).y() > (ambient.bottom_right_corner).y()) {
    Vector v1{(boid.position).x(), (ambient.bottom_right_corner).y()};
    Vector v2{(boid.velocity).x(), -(boid.velocity).y()};
    return Boid{v1, v2};
  }
  if ((boid.position).x() < (ambient.top_left_corner).x()) {
    Vector v1{(ambient.top_left_corner).x(), (boid.position).y()};
    Vector v2{-(boid.velocity).x(), (boid.velocity).y()};
    return Boid{v1, v2};
  }
  if ((boid.position).y() < (ambient.top_left_corner).y()) {
    Vector v1{(boid.position).x(), (ambient.top_left_corner).y()};
    Vector v2{(boid.velocity).x(), -(boid.velocity).y()};
    return Boid{v1, v2};
  } else {
    return boid;
  }
}

inline Vector air_resistance(Flock flock, Boid& boid) {
  double max_speed = 2.; //should not use hard-coded numbers!
  // double max_speed = velocity_parameters(flock).x() + 3 * velocity_parameters(flock).y();
  // double min_speed = velocity_parameters(flock).x() - 3 *velocity_parameters(flock).y();
  if ( speed(boid) > max_speed) {
    boid.velocity /= speed (boid);
    boid.velocity *= max_speed;
    return boid.velocity;
  }
  // should handle the case with min_speed?
  else { return boid.velocity; }
}

inline Vector air_resistance(Flock flock, Boid& boid) {
  double max_speed = 2.; //should not use hard-coded numbers!
  // double max_speed = velocity_parameters(flock).x() + 3 * velocity_parameters(flock).y();
  // double min_speed = velocity_parameters(flock).x() - 3 *velocity_parameters(flock).y();
  if ( speed(boid) > max_speed) {
    boid.velocity /= speed (boid);
    boid.velocity *= max_speed;
    return boid.velocity;
  }
  // should handle the case with min_speed?
  else { return boid.velocity; }
}
#endif