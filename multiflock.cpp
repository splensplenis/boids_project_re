#include "boids.hpp"
#include "rules.hpp"
class MultiFlocks {
  std::vector<Flock> flocks_{};  //vicini di ogni boid e poi correggi per ogni boid

public:
  MultiFlocks(std::vector<Flock> const&);
  int size() const;
  std::vector<Flock> get_flocks() const;
  double distance; //andrebbe privata
};
MultiFlocks::MultiFlocks(std::vector<Flock> const& flocks) : flocks_{flocks} {}
int MultiFlocks::size() const {return flocks_.size();}
std::vector<Flock> MultiFlocks::get_flocks() const {return flocks_;}
std::vector<Boid> other_neighbours(MultiFlocks const& multiflocks, Boid const& boid) {    
  std::vector<Boid> other_boids{};
  auto flocks = multiflocks.get_flocks();
  for(int i; i!=flocks.size()-1; ++i){ //VA MESSO SIZE-1 NEI VETTORI??
  auto flock_i = flocks[i].get_boids(); 
  auto member = std::find(flock_i.begin(), flock_i.end(), boid);
  if(member == flock_i.end()) {
    std::copy(flock_i.begin(), flock_i.end(), std::back_inserter(other_boids));
  }
 }   
  std::vector<Boid> other_neighbours{};
  for(int i; i!= other_boids.size()-1; ++i){
    if(distance(boid, other_boids[i])<= multiflocks.distance) {
       other_neighbours.push_back(other_boids[i]);
    }
  }
  return other_neighbours;
} //cosi ho tutti ii vicini entro una certa distanza 
//andrebbe messo anche entro l'angolo di visione, ma è variabile privata di flock e così non ho il flock del boid 

/*Flocks flocks_separation(Flocks const& f, ) {
  auto i = f[i]
}*/


/*for(int i; i!=multiflocks.size(); ++i){
    if(is_member(flocks[i],boid)==false) {
        std::copy(flocks.begin(), flocks.end(), other_flocks.begin()); NO
    }*/
    /*std::copy_if(flock_i.begin(), flock_i.end(), std::back_inserter(other_boids), [&](Flock const& flock1) {
    is_member(flock1, boid) == false;}); //flock viene preso da flocks? */
