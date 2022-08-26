#ifndef BOIDS_RANDOM_HPP
#define BOIDS_RANDOM_HPP

#include "boids.hpp"
#include "multiflock.hpp"
#include "rules.hpp"

#include <random>

// boid random generation:
//è effettivamente indipendente dalla grafica,
// si può mettere da un'altra parte: unico problema
//è che i valori hardcoded dipendono dalla grandezza dello schermo
// gli facciamo prendere un Ambient come argomento?
inline Flock generate_flock(int N, Options const& sp, double angle) {
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

/*inline MultiFlock generate_multiflock(//int n_flock,// int N_boids, Options const& sp,
                               double angle) {
  std::default_random_engine gen;

  std::normal_distribution<double> Vx(1., 0.05); //potrei fillare queste medie con valori random e poi fillare il resto con le medie trovate random
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
//e poi ripeto per tutti
}
*/
#endif