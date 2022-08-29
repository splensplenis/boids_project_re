#ifndef MULTIFLOCK_HPP
#define MULTIFLOCK_HPP
#include "boids.hpp"
#include "rules.hpp"
#include "vector.hpp"
#include <functional>

class MultiFlock {
  std::vector<Flock> flocks_{};

 public:
  MultiFlock(std::vector<Flock> const&);
  int size() const;
  std::vector<Flock> get_flocks() const;
  std::vector<Boid> get_all_boids() const;
  void add(Flock const&);
  void evolve(double delta_t, double max_speed,double min_speed); 
  void evolve(double, double, double, std::function<void(Flock&)>); //evolve multiflock and apply to each flock a user defined function
  std::vector<Vector> get_all_distance_mean_RMS() const;
  std::vector<Vector> get_all_speed_mean_RMS() const;
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
void MultiFlock::add(Flock const& flock) { flocks_.push_back(flock); }

//get neighbours of a boid excluding the ones from its flock
inline std::vector<Boid> get_other_neighbours(MultiFlock const& multiflocks,
                                              Boid const& boid) {
  std::vector<Boid> other_boids{};
  auto flocks = multiflocks.get_flocks();
  double alpha;
  Options boid_options;
  //for each flock in multiflock
  for (int i{}; i != multiflocks.size(); ++i) {
    auto boids_i = flocks[i].get_boids(); //get all the boid for iteration flock
    bool member = is_member(boids_i, boid); 
    if (member == false) { 
      //if selected boid is not a member of the iteration flock copy all iteration flock boid to other_boid vector
      std::copy(boids_i.begin(), boids_i.end(),
                std::back_inserter(other_boids));
    }
    if (member == true) {
      //if selected boid is a member if the iteration flock get its vision angle and options
      alpha = flocks[i].get_alpha();
      boid_options = flocks[i].get_options();
    } 
  }
  std::vector<Boid> other_neighbours{}; 
  //create a utility flock with all the boids not in the same flock with the selected boid
  //the utility flock has the opt and vision angle of the selected boid's flock in order to call get_neighbours_of
  Flock other_boids_flock{other_boids, boid_options, alpha};
  //get all the effective neighbours for the selected boid which aren't in its same flock
  other_neighbours = get_neighbours_of(other_boids_flock, boid);
  return other_neighbours;
}  


//get mean distance and variance of every flock in the multiflock
std::vector<Vector> MultiFlock::get_all_distance_mean_RMS() const {
  std::vector<Vector> all_distance_mean_RMS{};
  for (int i{}; i != this->size(); ++i) {
    auto mean_RMS = (flocks_[i]).get_distance_mean_RMS();
    all_distance_mean_RMS.push_back(mean_RMS);
  }
  return all_distance_mean_RMS;
}

//get mean velocity and variance of every flock in the multiflock
std::vector<Vector> MultiFlock::get_all_speed_mean_RMS() const {
  std::vector<Vector> all_speed_mean_RMS{};
  for (int i{}; i != this->size(); ++i) {
    auto mean_RMS = (flocks_[i]).get_speed_mean_RMS();
    all_speed_mean_RMS.push_back(mean_RMS);
  }
  return all_speed_mean_RMS;
}


void MultiFlock::evolve(double delta_t,double max_speed,double min_speed) {
  //for each flock in multiflock
  std::for_each(flocks_.begin(),flocks_.end(), [&](Flock& flock){
    std::vector<Boid> boid_vec {flock.get_boids()}; 
    std::vector<Boid> copy_vec {flock.get_boids()};
    for (size_t boid{}; boid != boid_vec.size(); ++boid){ //for each boid in flock
      std::vector<Boid> neighbours {get_other_neighbours(flocks_,boid_vec[boid])}; //get neighbours not in same flock
      boid_vec[boid].velocity += separation(flock.get_options(), copy_vec[boid], neighbours); //apply separation to those boids
    }
    flock = Flock(boid_vec, flock.get_options(), flock.get_alpha());
  });

  //evolve each flock in multiflock
  std::for_each(flocks_.begin(), flocks_.end(), [delta_t,max_speed,min_speed](Flock& flock){
    flock.evolve(delta_t,max_speed,min_speed);
  });

}

//std multiflock evolve with the application of a costraint to apply on flocks after evolution
void MultiFlock::evolve(double delta_t, double max_speed,double min_speed, std::function<void(Flock&)> costraint_applier){
  this->evolve(delta_t,max_speed,min_speed);
  std::for_each(flocks_.begin(),flocks_.end(),[&](Flock& flock){
    costraint_applier(flock);
  });
}

#endif