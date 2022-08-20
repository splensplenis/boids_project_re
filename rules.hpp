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
  /*if (is_member(flock, boid) == false) {
    throw std::runtime_error{"Boid is not in the flock"};  // MODIFICA
  }*/
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
    double alpha = flock.get_alpha();
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
  // auto flock = f.get_flock();
  auto boids_options = flock.get_options();
  Vector sum_velocity{};
  std::for_each(neighbours.begin(), neighbours.end(),
                [&](Boid const& boid1) { sum_velocity += boid1.velocity; });
  Vector v2_corr =
      (sum_velocity / neighbours.size() - boid.velocity) *
      boids_options.alignment;  // COSA FARE SE NON HA NEIGHBOURS(diviso per 0)?
  return v2_corr;
}
inline Vector cohesion(Flock flock, Boid const& boid,
                       std::vector<Boid> neighbours) {
  // auto flock = f.get_flock();
  auto boids_options = flock.get_options();
  Vector partial_sum{};
  std::for_each(neighbours.begin(), neighbours.end(),
                [&](Boid const& boid1) { partial_sum += boid1.position; });
  Vector centre_of_mass = partial_sum / neighbours.size();
  Vector v3_corr = (centre_of_mass - boid.position) * boids_options.cohesion;
  return v3_corr;
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
// COMPORTAMENTO AI BORDI

#endif