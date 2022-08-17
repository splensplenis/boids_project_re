#include <iostream>

#include "boids.hpp"
#include "rules.hpp"
#include "vector.hpp"

int main() {
  Boid b1{Vector{2., 1.}, Vector{0.2, 1.}};
  Boid b2{Vector{4., 3.}, Vector{-1., 0.2}};
  Boid b3{Vector{1., 4.}, Vector{2., 0.5}};
  Options boids_options{10., 0.07, 0.9, 0.7, 0.6};
  Flock f{std::vector<Boid>{b1, b2, b3}, boids_options, 180};
  for (int i = 0.; i != 15; ++i) {
    auto flock = f.get_boids();
    std::cout << "----- Iteration #" << i << "-----" << '\n';
    (flock[0].position).print();
    (flock[1].position).print();
    (flock[2].position).print();
    std::cout << "Dist: " << (distance_parameters(f)).x() << " +/-"
              << (distance_parameters(f)).y() << '\n';
    std::cout << "Speed: " << (velocity_parameters(f)).x() << " +/-"
              << (velocity_parameters(f)).y() << '\n';
    f.evolve(0.1);
  }

  /*int main() {
      double N;
      double loop_time;
      std::cout << "How many boids do you want to randomly generate?\n";
      std::cin >> N;
      std::cout << "For how long (seconds) do you want to evolve the flock?\n";
      std::cin >> loop_time;

      // random generation of boids
      std::default_random_engine gen;

      std::normal_distribution<double> Vx(2, 0.5);
      std::normal_distribution<double> Vy(2., 0.5);
      std::vector<double> vel_x{};
      std::vector<double> vel_y{};

      std::uniform_real_distribution<double> Xx(-10., 10.);
      std::uniform_real_distribution<double> Xy(-10., 10.);
      std::vector<double> pos_x{};
      std::vector<double> pos_y{};
      for (int n = 0; n != N; ++n) {  // coordinates are casually generated
        vel_x.push_back(Vx(gen));
        vel_y.push_back(Vy(gen));
        pos_x.push_back(Xx(gen));
        pos_y.push_back(Xy(gen));
      }

      std::vector<Boid> empty{};
      Flock stormo{empty};  // now boids are created in the flock
      for (int i = 0.; i != N; ++i) {
        Boid b{pos_x[i], pos_y[i], vel_x[i], vel_y[i], 90.};
        stormo.add(b);
      }

      Boid bb = stormo.get()[1];
      std::cout << (bb.x()).x <<'\n';

      // the system is ready to be evolved
      double dt = 0.01;
      double long_count{};
      while (long_count != loop) {
        //stormo.evolve(dt);
        long_count += dt;
        if (long_count == 1.0) {
          stato(stormo);
        }
        if (long_count == 2.0) {
          stato(stormo);
        }
        if (long_count == 3.0) {
          stato(stormo);
        }
        if (long_count == 4.0) {
          stato(stormo);
        }
        if (long_count == 5.0) {
          stato(stormo);
        }
      }
  //stato() function prints dist mean and speed mean with std dev
    */
}
