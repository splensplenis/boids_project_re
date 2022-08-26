#include <fstream>
#include <iomanip>
#include <iostream>

#include "boids.hpp"
#include "multiflock.hpp"
#include "rules.hpp"

void write_to_file(std::vector<Vector> info_position,
                   std::vector<Vector> info_velocity,
                   std::vector<double> info_time) {
  std::ofstream fos;     // file output stream
  fos.open("data.txt");  
  // statistics printed here, to be used on root to see a graph
  for (long unsigned int i{}; i != info_time.size(); ++i) {
    fos << info_time[i] << '\t' << info_position[i].x() << '\t'
        << info_velocity[i].x() << '\n';
    // fos.close() ci vuole??
  }
}  // questo come fa a printare se ci sono più flock? con più colonne? poi per
   // root è un casino, ma solo per root

/*std::ostream& operator<<(std::ostream& os, PPState const& p) {
  os << "pps ( m: " << p.m << ", x: " << p.x << ", v: " << p.v << ')';
  return os;
}*/