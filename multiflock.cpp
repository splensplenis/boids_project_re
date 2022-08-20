#include "boids.hpp"
#include "rules.hpp"
class MultiFlocks {
  std::vector<Flock>
      flocks_{};  // vicini di ogni boid e poi correggi per ogni boid

 public:
  MultiFlocks(std::vector<Flock> const&);
  //int size() const;
  std::vector<Flock> get_flocks() const;
  // double distance;  // andrebbe privata
  void evolve();
};
MultiFlocks::MultiFlocks(std::vector<Flock> const& flocks) : flocks_{flocks} {}
//int MultiFlocks::size() const { return flocks_.size(); }
std::vector<Flock> MultiFlocks::get_flocks() const { return flocks_; }
std::vector<Boid> get_other_neighbours(MultiFlocks const& multiflocks,
                                       Boid const& boid) {
  std::vector<Boid> other_boids{};
  auto flocks = multiflocks.get_flocks();
  double alpha;
  Options boid_options;
  for (int i; i != flocks.size() - 1; ++i) {  // VA MESSO SIZE-1 NEI VETTORI??
    auto flock_i = flocks[i].get_boids();     // con find_if?
    auto member = std::find(flock_i.begin(), flock_i.end(), boid);
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

/*for (int i; i != other_boids.size() - 1; ++i) {
if (distance(boid, other_boids[i]) <= multiflocks.distance) {
  other_neighbours.push_back(other_boids[i]);
}
}*/

std::vector<Boid> view_other_neighbours(MultiFlocks const& multiflocks,
                                        Boid const& boid) {
  std::vector<Boid> other_neighbours = get_other_neighbours(multiflocks, boid);
  auto flocks = multiflocks.get_flocks();
  double alpha;
  Options boid_options{};
  for (int i; i != flocks.size() - 1;
       ++i) {  // altro modo per trovare a quale flock appartiene boid?
    auto flock_i = flocks[i].get_boids();
    auto member = std::find(flock_i.begin(), flock_i.end(), boid);
    if (member != flock_i.end()) {
      alpha = flocks[i].get_alpha();
      boid_options = flocks[i].get_options();
    }
  }
  Flock other_neighbours_flock{other_neighbours, boid_options, alpha};
  std::vector<Boid> view_other_neighbours{};
  view_other_neighbours =
      view_neighbours(other_neighbours_flock,
                      boid);  // distance è quella dello stesso flock cosi
  return view_other_neighbours;
}
/*void MultiFlocks::evolve(Boid& boid) {
  for (int i = 0; i != this->size(); ++i) {
    if (is_member(flocks_[i], boid) == true) {
      Flock myflock = flocks_[i];
    }
  }
  std::vector<Boid> tot_neigh{};
  // per riempirlo scorro tutti i flock e di ognuno prendo i suoi vicini
  for (int i = 0; i != this->size(); ++i) {
    std::vector<Boid> neighbours = get_neighbours_of(flocks_[i], boid);
    for (int j = 0; j != neighbours.size(); ++j) {
      tot_neigh.push_back(neighbours[i]);
    }
  }
  // ora ho tutti i vicini del boid, non importa di quale flock siano,
  // li divido: per quelli del suo flock si applicano le tre regole,
  // per tutti gli altri solo la separazione
  std::vector<Boid> other_neighbours{};
  for (int i = 0; i != tot_neigh.size(); ++i) {
    if (is_member(myflock, tot_neigh[i]) == false) {
      other_neighbours.push_back(tot_neigh[i]);
    }
  }

  myflock.evolve();
  boid.velocity += separation(myflock, boid, other_neighbours);
  //devo usare un boid copied come in Flock::evolve, altrimenti
  //si fa evolvere un boid alla volta mentre deve essere contemporaneo
}*/
