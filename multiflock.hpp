#ifndef MULTIFLOCK_HPP
#define MULTIFLOCK_HPP

#include "boids.hpp"
#include "rules.hpp"
#include "vector.hpp"

class MultiFlock {
  std::vector<Flock> flocks_{};

 public:
  MultiFlock(std::vector<Flock> const&);
  int size() const;
  std::vector<Flock> get_flocks() const;
  std::vector<Boid> get_all_boids() const;
  void add(Flock const&);
  void evolve(double delta_t); //Ambient const& amb, 
  std::vector<Vector> get_all_distance_mean_RMS() const;
  std::vector<Vector> get_all_speed_mean_RMS() const;
};

inline MultiFlock::MultiFlock(std::vector<Flock> const& flocks) : flocks_{flocks} {}
inline int MultiFlock::size() const { return flocks_.size(); }
inline std::vector<Flock> MultiFlock::get_flocks() const { return flocks_; }
inline std::vector<Boid> MultiFlock::get_all_boids() const {
  std::vector<Boid> all_boids{};
  for (int i{}; i != this->size(); ++i) {
    auto each_flock = flocks_[i].get_boids();
    for (int j{}; j != (flocks_[i]).size(); ++j) {
      all_boids.push_back(each_flock[j]);
    }
  }
  return all_boids;
}
inline void MultiFlock::add(Flock const& flock) { flocks_.push_back(flock); }

inline std::vector<Boid> get_other_neighbours(MultiFlock const& multiflocks,
                                              Boid const& boid) {
  std::vector<Boid> other_boids{};
  auto flocks = multiflocks.get_flocks();
  double alpha;
  Options boid_options;
  for (int i{}; i != multiflocks.size(); ++i) {
    auto boids_i = flocks[i].get_boids();
    bool member = is_member(boids_i, boid);
    if (member == false) {
      std::copy(boids_i.begin(), boids_i.end(),
                std::back_inserter(other_boids));
    }
    if (member == true) {
      alpha = flocks[i].get_alpha();
      boid_options = flocks[i].get_options();
    } 
  }
  std::vector<Boid> other_neighbours{};
  Flock other_boids_flock{other_boids, boid_options, alpha};
  other_neighbours = get_neighbours_of(other_boids_flock, boid);
  return other_neighbours;
}  
// this returns all the boids within distance 
// of neighbourhood, regardless of their flock

inline std::vector<Vector> MultiFlock::get_all_distance_mean_RMS() const {
  std::vector<Vector> all_distance_mean_RMS{};
  for (int i{}; i != this->size(); ++i) {
    auto mean_RMS = (flocks_[i]).get_distance_mean_RMS();
    all_distance_mean_RMS.push_back(mean_RMS);
  }
  return all_distance_mean_RMS;
}
inline std::vector<Vector> MultiFlock::get_all_speed_mean_RMS() const {
  std::vector<Vector> all_speed_mean_RMS{};
  for (int i{}; i != this->size(); ++i) {
    auto mean_RMS = (flocks_[i]).get_speed_mean_RMS();
    all_speed_mean_RMS.push_back(mean_RMS);
  }
  return all_speed_mean_RMS;
}
inline void MultiFlock::evolve(double delta_t) { //Ambient const& amb, 
  for (int i{}; i != this->size(); ++i) {
    Flock flock_i = flocks_[i];
    auto boids_i = flock_i.get_boids();
    std::vector<Boid> copy{boids_i};
    for (int j{}; j != flock_i.size(); ++j) {
      auto boid = boids_i[j];
      auto boid_copied = copy[j];
      std::vector<Boid> other_neighbours = get_other_neighbours(*this, boid);
      Options boid_options = flock_i.get_options();
      boid.velocity += separation(boid_options, boid_copied, other_neighbours);
      other_neighbours.clear();
      boids_i[j] = boid;
    }
    flock_i.evolve(delta_t); //amb, 
    flocks_[i] = flock_i;
  }
}

#endif