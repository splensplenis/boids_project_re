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
  void evolve(Ambient const& amb, double delta_t);
};
MultiFlock::MultiFlock(std::vector<Flock> const& flocks) : flocks_{flocks} {}
int MultiFlock::size() const { return flocks_.size(); }
std::vector<Flock> MultiFlock::get_flocks() const { return flocks_; }
std::vector<Boid> MultiFlock::get_all_boids() const {
  std::vector<Boid> all_boids{};
  for (int i{}; i != this->size(); ++i) {
    auto each_flock = flocks_[i].get_boids();
    for (int j{}; j != (flocks_[i]).size(); ++j) {
      all_boids.push_back(each_flock[j]);
    }
  }
  return all_boids;
}
inline std::vector<Boid> get_other_neighbours(MultiFlock const& multiflocks,
                                              Boid const& boid) {
  std::vector<Boid> other_boids{};
  auto flocks = multiflocks.get_flocks();
  double alpha;
  Options boid_options;
  for (int i{}; i != multiflocks.size(); ++i) {
    auto flock_i = flocks[i].get_boids();
    auto member = std::find(flock_i.begin(), flock_i.end(), boid);  // find_if?
    if (member == flock_i.end()) {
      std::copy(flock_i.begin(), flock_i.end(),
                std::back_inserter(other_boids));
    }
    if (member != flock_i.end()) {
      alpha = flocks[i].get_alpha();
      boid_options = flocks[i].get_options();
    }
  }
  std::vector<Boid> other_neighbours{};
  Flock other_boids_flock{other_boids, boid_options, alpha};
  other_neighbours = get_neighbours_of(other_boids_flock, boid);
  return other_neighbours;
}  // cosi ho tutti ii vicini entro una certa distanza

void MultiFlock::evolve(Ambient const& amb, double delta_t) {
  /*
    for(int i{}; i != this->size(); ++i) {
      flocks_[i].evolve(amb, delta_t);
    } //easy version, to see if it's woring
  */
  for (int i{}; i != this->size(); ++i) {
    Flock flock_i = flocks_[i];
    auto boids_i = flock_i.get_boids();
    std::vector<Boid> copy{boids_i};
    for (int j{}; j != flock_i.size(); ++j) {
      auto boid = boids_i[j];
      auto boid_copied = copy[j];
      std::vector<Boid> other_neighbours = get_other_neighbours(*this, boid);
      boid.velocity += separation(flock_i.get_options(), boid_copied, other_neighbours);
      other_neighbours.clear();
      boids_i[j] = boid;
    }
    flock_i.evolve(amb, delta_t);
    flocks_[i] = flock_i;
  }
}

#endif