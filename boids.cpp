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
             double alpha)
    : boids_{boids}, boids_options_{boids_options}, alpha_{alpha} {}
Flock::Flock(Flock const& flock) : boids_{flock.boids_} {}
int Flock::size() const { return boids_.size(); }
std::vector<Boid> Flock::get_boids() const { return boids_; }
Options Flock::get_options() const { return boids_options_; }
double Flock::get_alpha() const { return alpha_; }
void Flock::add(Boid const& b1) { boids_.push_back(b1); }
void Flock::evolve(double delta_t) {
  std::vector<Boid> copy{boids_};
  for (int i{}; i != this->size(); ++i) {
    auto boid = boids_[i];
    auto boid_copied = copy[i];
    boid.velocity +=
        (separation(*this, boid_copied, get_neighbours_of(*this, boid_copied)) +
         alignment(*this, boid_copied, get_neighbours_of(*this, boid_copied)) +
         cohesion(*this, boid_copied, get_neighbours_of(*this, boid_copied)));
    boid.position += (boid_copied.velocity * delta_t);
    boids_[i] = boid;
  }
}

/*double distance(Boid const& boid1, Boid const& boid2) {
  // modulo del vettore distanza tra boids
  double d = sqrt(norm2(boid1.position - boid2.position));
  return d;
}
double speed(Boid const& boid) {  // modulo del vettore velocità
  double s = sqrt(norm2(boid.velocity));
  return s;
}
Vector applied_distance(Boid const& boid1, Boid const& boid2) {
  // vettore distanza tra boids
  Vector diff = boid2.position - boid1.position;
  return diff;
}

bool are_neighbours(Flock f, Boid const& boid1, Boid const& boid2) {
  auto sp = f.get_species();
  return (distance(boid1, boid2) <= sp.distance);
}
bool is_member(Flock f, Boid const& boid) {
  auto flock = f.get_boids();
  auto i = std::find(flock.begin(), flock.end(), boid);
  if (i != flock.end()) {
    return true;
  }
  return false;
}
auto get_neighbours_of(Flock f, Boid const& boid) {
  auto flock = f.get_boids();
  if (is_member(f, boid) == false) {
    throw std::runtime_error{"Boid is not in the flock"};
  }
  std::vector<Boid> neighbours{};
  for (double i{}; i != f.size(); ++i) {
    if ((flock[i]) != boid && are_neighbours(f, boid, flock[i])) {
      neighbours.push_back(flock[i]);
    }
  }
  return neighbours;
}

auto view_neighbours(Flock f, Boid const& boid) {
  auto flock = f.get_flock();
  auto sp = f.get_species();
  double pi = std::acos(-1.0);
  std::vector<Boid> view_neighbours{};
  std::vector<Boid> neighbours = get_neighbours_of(f, boid);
  for (double i{}; i != neighbours.size();
       ++i) {  // scalar product gives the angle
    double theta =
        std::acos((boid.velocity * applied_distance(boid, neighbours[i])) /
                  (speed(boid) * distance(boid, neighbours[i])));
    theta = 180 * theta / pi;
    if (theta <= sp.alpha_) {
      view_neighbours.push_back(neighbours[i]);
    }
  }
  return view_neighbours;
}


Vector separation(Flock f, Boid const& boid1, std::vector<Boid> neighbours) {
  //auto flock = f.get_flock();
  auto sp = f.get_species();
  Vector partial_sum{};
  std::for_each(neighbours.begin(), neighbours.end(), [&](Boid const& boid) {
    if (distance(boid, boid1) <= sp.separation_distance) {
      partial_sum += applied_distance(boid1, boid);
    }
  });
  Vector v1_corr = partial_sum * (-sp.separation);
  return v1_corr;
}
Vector alignment(Flock f, Boid const& boid1, std::vector<Boid> neighbours) {
 // auto flock = f.get_flock();
  auto sp = f.get_species();
  Vector sum_velocity{};
  std::for_each(neighbours.begin(), neighbours.end(),
                [&](Boid const& boid) { sum_velocity += boid.velocity; });
  Vector v2_corr =
      (sum_velocity / neighbours.size() - boid1.velocity) * sp.alignment;
  return v2_corr;
}
Vector cohesion(Flock f, Boid const& boid1, std::vector<Boid> neighbours) {
  //auto flock = f.get_flock();
  auto sp = f.get_species();
  Vector partial_sum{};
  std::for_each(neighbours.begin(), neighbours.end(),
                [&](Boid const& boid) { partial_sum += boid.position; });
  Vector centre_of_mass = partial_sum / neighbours.size();
  Vector v3_corr = (centre_of_mass - boid1.position) * sp.cohesion;
  return v3_corr;
}

Vector distance_parameters(Flock f) {
  // filling a histogram with distances between all boids of the flock
  // then calculating its mean and standard deviation for a given time
  std::vector<double> dist_histo{};
  double partial_sum{};
  double partial_sum2{};
  auto flock = f.get_boids();
  for (int i{}; i != f.size(); ++i) {
    for (int j{i + 1}; j != f.size(); ++j) {
      double value = distance(flock[i], flock[j]);
      dist_histo.push_back(value);
      // this way distance bewtween boid 0 and boid 1 is calculated only once
      // non dovremmo mettere un i!=j ?
      partial_sum2 += value;
    }
  }
  double mean_distance = partial_sum2 / f.size();
  for (int i{}; i != f.size(); ++i) {
    partial_sum += (dist_histo[i] - mean_distance) * (dist_histo[i] -
mean_distance);
  }
  double stddev_distance = partial_sum / (f.size() - 1);
  return Vector{mean_distance, stddev_distance};
}
Vector velocity_parameters(Flock f) {
  std::vector<double> speed_histo{};
  double partial_sum{};
  double partial_sum2{};
  auto flock = f.get_boids();
  for (int i{}; i != f.size(); ++i) {
    double value = speed(flock[i]);
    speed_histo.push_back(value);
    partial_sum2 += value;
  }
  double mean_speed = partial_sum2 / f.size();
  for (int i{}; i != f.size(); ++i) {
    partial_sum +=
        (speed_histo[i] - mean_speed) * (speed_histo[i] - mean_speed);
  }
  double stddev_speed = partial_sum / (f.size() - 1);
  return Vector{mean_speed, stddev_speed};
}
*/