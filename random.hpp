#ifndef RANDOM_HPP
#define RANDOM_HPP

#include "boids.hpp"
#include "multiflock.hpp"
#include "rules.hpp"

#include <random>
#include <fstream>

Flock generate_flock(int N, Options const& sp, double angle) {
  std::default_random_engine gen;
  // should generate more flocks randomly
  // this way it only generates the same random flock again and again

  std::normal_distribution<double> Vx(1, 0.05);
  std::normal_distribution<double> Vy(0.5, 0.03);
  std::vector<double> vel_x{};
  std::vector<double> vel_y{};

  std::uniform_real_distribution<double> Xx(0, 10.);
  std::uniform_real_distribution<double> Xy(0., 10.);
  std::vector<double> pos_x{};
  std::vector<double> pos_y{};
  for (int n = 0; n != N; ++n) {  // coordinates are casually generated
    vel_x.push_back(Vx(gen));
    vel_y.push_back(Vy(gen));
    pos_x.push_back(Xx(gen));
    pos_y.push_back(Xy(gen));
  }
  std::vector<Boid> empty{};
  Flock f{empty, sp, angle};
  for (int i = 0.; i != N; ++i) {
    Boid b{Vector{pos_x[i], pos_y[i]}, Vector{vel_x[i], vel_y[i]}};
    f.add(b);
  }
  return f;
}

// anche questo non c'entra con la grafica ma è un'opzione in più da aggiungere
// nel main
void write_to_file(std::vector<Vector> info_position,
                   std::vector<Vector> info_velocity,
                   std::vector<double> info_time) {
  std::ofstream fos;     // file output stream
  fos.open("data.txt");  // statistics printed here, to be used on root to see a
                         // graph
  for (int i = 0; i != info_time.size(); ++i) {
    fos << info_time[i] << '\t' << info_position[i].x() << '\t'
        << info_velocity[i].x() << '\n';
    // fos.close() ci vuole??
  }
}  // questo come fa a printare se ci sono più flock? con più colonne? poi per
   // root è un casino
#endif