#ifndef BOIDS_RANDOM_HPP
#define BOIDS_RANDOM_HPP

#include <random>

#include "boids.hpp"
#include "multiflock.hpp"
#include "rules.hpp"

// boid random generation:
inline Flock generate_flock(int N, Options const& options, double angle) {
  std::default_random_engine gen;

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
  Flock f{empty, options, angle};
  for (int i = 0.; i != N; ++i) {
    Boid b{Vector{pos_x[i], pos_y[i]}, Vector{vel_x[i], vel_y[i]}};
    f.add(b);
  }
  return f;
}

inline MultiFlock generate_multiflock(int n_flock, int N_boids,
                                      Options const& options, double angle) {
  // each flock's mean and rms is randomly generated according to a uniform
  // distribution, then for each boid it works as above
  std::default_random_engine gen;

  std::uniform_real_distribution<double> flocks_mean(-5., 5.);
  std::uniform_real_distribution<double> flocks_rms(-0.1, 0.1);
  std::vector<double> V_mean{};
  std::vector<double> V_rms{};

  for (int i{}; i != 2 * n_flock; ++i) {
    V_mean.push_back(flocks_mean(gen));
    V_rms.push_back(flocks_rms(gen));
  }

  std::vector<Flock> flock_vector{};
  MultiFlock multi_empty{flock_vector};

  for (int j{}; j != n_flock; ++j) {
    std::normal_distribution<double> Vx(V_mean[j], V_rms[j]);
    std::normal_distribution<double> Vy(V_mean[2 * n_flock - j],
                                        V_rms[2 * n_flock - j]);
    // starting once from begin then from end, it's uniform so no problem

    std::vector<double> vel_x{};
    std::vector<double> vel_y{};

    std::uniform_real_distribution<double> Xx(0, 10.);
    std::uniform_real_distribution<double> Xy(0., 10.);
    std::vector<double> pos_x{};
    std::vector<double> pos_y{};

    for (int n = 0; n != N_boids; ++n) {
      vel_x.push_back(Vx(gen));
      vel_y.push_back(Vy(gen));
      pos_x.push_back(Xx(gen));
      pos_y.push_back(Xy(gen));
    }
    std::vector<Boid> empty{};
    Flock flock_j{empty, options, angle};
    for (int i = 0.; i != N_boids; ++i) {
      Boid b{Vector{pos_x[i], pos_y[i]}, Vector{vel_x[i], vel_y[i]}};
      flock_j.add(b);
    }
    multi_empty.add(flock_j);
  }
  
  return multi_empty;
}

#endif