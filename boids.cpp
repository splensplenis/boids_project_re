#include "boids.hpp"

#include <iostream>

#include "vector.hpp"

bool operator==(Boid const& boid1, Boid const& boid2) {
  return (boid1.position == boid2.position && boid1.velocity == boid2.velocity);
}
bool operator!=(Boid const& boid1, Boid const& boid2) {
  return !(boid1 == boid2);
}

double distance(Boid const& boid1, Boid const& boid2) {
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

Flock::Flock(std::vector<Boid> v, Species s) : flock_{v}, species_{s} {};
// Flock::Flock(Flock const& f) : flock_{f.flock_} {}
int Flock::size() const { return flock_.size(); }
std::vector<Boid> Flock::get_flock() const { return flock_; }
Species Flock::get_species() const { return species_; }
void Flock::add(Boid const& b1) { flock_.push_back(b1); }

bool are_neighbours(Flock f, Boid const& boid1, Boid const& boid2) {
  auto sp = f.get_species();
  return (distance(boid1, boid2) <= sp.distance_);
}
bool is_member(Flock f, Boid const& boid) {
  auto flock = f.get_flock();
  auto i = std::find(flock.begin(), flock.end(), boid);
  if (i != flock.end()) {
    return true;
  }
  return false;
}
auto get_neighbours_of(Flock f, Boid const& boid) {
  auto flock = f.get_flock();
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

/*auto view_neighbours(Flock f, Boid const& boid) {
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
*/

Vector separation(Flock f, Boid const& boid1, std::vector<Boid> neighbours) {
  auto flock = f.get_flock();
  auto sp = f.get_species();
  Vector partial_sum{};
  std::for_each(neighbours.begin(), neighbours.end(), [&](Boid const& boid) {
    if (distance(boid, boid1) <= sp.separation_distance_) {
      partial_sum += applied_distance(boid1, boid);
    }
  });
  Vector v1_corr = partial_sum * (-sp.separation_);
  return v1_corr;
}
Vector alignment(Flock f, Boid const& boid1, std::vector<Boid> neighbours) {
  auto flock = f.get_flock();
  auto sp = f.get_species();
  Vector sum_velocity{};
  std::for_each(neighbours.begin(), neighbours.end(),
                [&](Boid const& boid) { sum_velocity += boid.velocity; });
  Vector v2_corr =
      (sum_velocity / neighbours.size() - boid1.velocity) * sp.alignment_;
  return v2_corr;
}
Vector cohesion(Flock f, Boid const& boid1, std::vector<Boid> neighbours) {
  auto flock = f.get_flock();
  auto sp = f.get_species();
  Vector partial_sum{};
  std::for_each(neighbours.begin(), neighbours.end(),
                [&](Boid const& boid) { partial_sum += boid.position; });
  Vector centre_of_mass = partial_sum / neighbours.size();
  Vector v3_corr = (centre_of_mass - boid1.position) * sp.cohesion_;
  return v3_corr;
}

void Flock::evolve(double delta_t) {
  std::vector<Boid> copy{flock_};
  for (int i{}; i != this->size(); ++i) {
    // corrections read from copy (old state)
    // and written to flock (updated state);
    auto boid = flock_[i];
    auto boid_copied = copy[i];
    boid.velocity +=
        (separation(*this, boid_copied, get_neighbours_of(*this, boid_copied)) +
         alignment(*this, boid_copied, get_neighbours_of(*this, boid_copied)) +
         cohesion(*this, boid_copied, get_neighbours_of(*this, boid_copied)));
    boid.position += (boid_copied.velocity * delta_t);
    flock_[i] = boid;
  }
}
Vector distance_parameters(Flock f) {
  // filling a histogram with distances between all boids of the flock
  // then calculating its mean and standard deviation for a given time
  std::vector<double> dist_histo{};
  double partial_sum2{};
  double partial_sum{};
  auto flock = f.get_flock();
  for (int i{}; i != f.size(); ++i) {
    for (int j{i + 1}; j != f.size(); ++j) {
      double value = distance(flock[i], flock[j]);
      dist_histo.push_back(value);
      // this way distance bewtween boid 0 and boid 1 is calculated only once
      // etc
      partial_sum2 += value;
    }
  }
  double mean_dist = partial_sum2 / f.size();
  for (int i{}; i != f.size(); ++i) {
    partial_sum += (dist_histo[i] - mean_dist) * (dist_histo[i] - mean_dist);
  }
  double stddev_dist = partial_sum / (f.size() - 1);
  return Vector{mean_dist, stddev_dist};
}
Vector velocity_parameters(Flock f) {
  std::vector<double> speed_histo{};
  double partial_sum2{};
  double partial_sum{};
  auto flock = f.get_flock();
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