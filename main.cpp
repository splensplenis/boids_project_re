//compilation string:
//g++ -Wall -Wextra -fsanitize=address vector.cpp boids.cpp main.cpp
#include <iostream>
#include <fstream>
//#include <iomanip>

#include "boids.hpp"
#include "multiflock.hpp"
#include "rules.hpp"
#include "boids_random.hpp"

void write_to_file(std::vector<Vector> info_position,
                   std::vector<Vector> info_velocity,
                   std::vector<double> info_time) {
  std::ofstream file_output_stream;
  file_output_stream.open("data.txt");  
  // statistics printed here, to be used on root to see a graph
  for (long unsigned int i{}; i != info_time.size(); ++i) {
    file_output_stream << info_time[i] << '\t' << info_position[i].x() << '\t'
        << info_velocity[i].x() << '\n';
    // fos.close() ci vuole??
  }
}  // questo come fa a printare se ci sono più flock? con più colonne? poi per
   // root è un casino, ma solo per root

int main() {
  // Options simulation_options{3, 0.4, 0.5, 0.4, 0.5};
  std::cout << "------Boid simulation-------" << '\n'
            << "Plase enter values for simulation parameters:" << '\n'
            << "Number of boids for each flock (default value = 40):" << '\n'
            << "Distance of neighbours (default value = 1):" << '\n'
            << "Distance of collision (default value = 0.5):" << '\n'
            << "Separation rule (default value = 0.3):" << '\n'
            << "Alignment rule (default value = 0.1):" << '\n'
            << "Cohesion rule (default value = 1):" << '\n';
  double angle{180};
  int N;
  double d;
  double d_s;
  double s;
  double a;
  double c;
  std::cin >> N >> d >> d_s >> s >> a >> c;
  Options simulation_options{d, d_s, s, a, c};
  std::cout << "Do you want info about the simulation to be stored in a file "
               "data.txt? (Y/N)"
            << '\n';
  char choice;
  std::cin >> choice;
  Flock flock1 = generate_flock(N, simulation_options, angle);

  Boid b1{Vector{1, 1}, Vector{1, 0}};
  Boid b2{Vector{5, 5}, Vector{0, 1}};
  Boid b3{Vector{10, 10}, Vector{2, 2}};
  Boid b4{Vector{10, 0}, Vector{1.5, 9}};

  Boid b5{Vector{8, 7}, Vector{0.3, 2}};
  Boid b6{Vector{2, 3}, Vector{9.6, 4.5}};
  Boid b7{Vector{4, 4}, Vector{2.4, 9.5}};
  Boid b8{Vector{1.6, 3.8}, Vector{1.2, 1}};
  // Flock fprova{std::vector<Boid>{b1,b2}, simulation_options}; we need
  // 2-argument constructor for this
  Flock f1{std::vector<Boid>{b1, b2, b3, b4}, simulation_options, angle};
  Flock f2{std::vector<Boid>{b5, b6, b7, b8}, simulation_options, angle};

  MultiFlock multiflock{std::vector<Flock>{flock1, f1, f2}};
  std::vector<std::vector<Vector>> info_position{};
  std::vector<std::vector<Vector>> info_velocity{};
  std::vector<double> info_time{};
  //graphics_simulation(multiflock, info_position, info_velocity, info_time);
  if (choice == 'Y' && multiflock.size() == 1) {
    std::vector<Vector> first_flock_position = info_position[0];
    std::vector<Vector> first_flock_velocity = info_velocity[0];
    write_to_file(first_flock_position, first_flock_velocity, info_time);
    std::cout << "File data.txt was filled with info about the simulation"
              << '\n';
  }
  return 0;
}