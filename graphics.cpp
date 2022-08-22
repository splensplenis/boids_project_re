// compile with: g++ graphics.cpp -lsfml-graphics -lsfml-window -lsfml-system
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "rules.hpp"
#include "multiflock.hpp"

auto evolve(Ambient amb, MultiFlock& more_flock, int steps_per_evolution,
            sf::Time delta_t) {
  double const dt{delta_t.asSeconds()};

  for (int i{0}; i != steps_per_evolution; ++i) {
    more_flock.evolve(amb, dt);
  }
  return more_flock.get_all_boids();
}

// boid random generation
Flock generate_flock(int N, Options sp, double angle) {
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
  Flock f{empty, sp, angle};
  for (int i = 0.; i != N; ++i) {
    Boid b{Vector{pos_x[i], pos_y[i]}, Vector{vel_x[i], vel_y[i]}};
    f.add(b);
  }
  return f;
}

void write_to_file(std::vector<Vector> info_position, std::vector<Vector> info_velocity, std::vector<double> info_time) {
  std::ofstream fos;     // file output stream
  fos.open("data.txt");  // statistics printed here, to be used on root to see a graph
  for (int i = 0; i != info_time.size(); ++i) {
    fos << info_time[i] <<'\t' << info_position[i].x() <<'\t' << info_velocity[i].x() <<'\n';
  }
}

void graphics_simulation(MultiFlock& more_flock, std::vector<Vector> info_position,
                         std::vector<Vector> info_velocity, std::vector<double> info_time) {
  double time_count{};

  auto const delta_t{sf::milliseconds(1)};
  int const fps = 30;
  int const steps_per_evolution{1000 / fps};

  unsigned const display_width = 0.7 * sf::VideoMode::getDesktopMode().width;
  unsigned const display_height = 0.7 * sf::VideoMode::getDesktopMode().height;

  auto const min_x{0.};
  auto const max_x{10.1};
  auto const min_y{0.};
  auto const max_y{10.1};
  auto const scale_x = display_width / (max_x - min_x);
  auto const scale_y = display_height / (max_y - min_y);

  sf::RenderWindow window(sf::VideoMode(display_width, display_height),
                          "Boids!");
  window.setFramerateLimit(fps);

  Ambient boundaries{Vector{min_x, min_y}, Vector{max_x, max_y}};
  sf::Texture texture;
  if (!texture.loadFromFile("image.png", sf::IntRect(5, 5, 6, 6))) {
    std::cout << "Cannot load boid graphics" << '\n';
  }
  sf::Sprite sprite{};
  sprite.setTexture(texture);

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    window.clear(sf::Color::White);

    auto const flock_vector =
        evolve(boundaries, more_flock, steps_per_evolution, delta_t);

    for (auto& boid : flock_vector) {
      sprite.setPosition((boid.position).x() * scale_x,
                         (boid.position).y() * scale_y);
      window.draw(sprite);
    }

    window.display();

    if(more_flock.size() == 1) {
    auto f = (more_flock.get_flocks())[0];
    info_position.push_back(distance_parameters(f));
    info_velocity.push_back(velocity_parameters(f));
    }

    time_count += delta_t.asSeconds();
    info_time.push_back(time_count);
  }
}

///*
int main() {
  //Options sp{3, 0.4, 0.5, 0.4, 0.5};
  std::cout << "------Boid simulation-------" <<'\n'
            << "Plase enter values for simulation parameters:" <<'\n'
            << "Number of boids for each flock (default value = 15):" <<'\n'
            << "Distance of neighbours (default value = 3):" <<'\n'
            << "Distance of collision (default value = 0.4):" <<'\n'
            << "Separation rule (default value = 0.5):" << '\n'
            << "Alignment rule (default value = 0.4):" <<'\n'
            << "Cohesion rule (default value = 0.5):" <<'\n';
  //angle of view (180° by default where to be chosen?)
  double angle{90};
  int N;
  double d;
  double d_s;
  double s;
  double a;
  double c;
  std::cin >> N >> d >> d_s >> s >> a >> c;
  Options simulation_options{d,d_s,s,a,c};
  std::cout << "Do you want info about the simulation to be stored in a file data.txt? (Y/N)" <<'\n';
  char choice;
  std::cin >> choice;
  Flock flock = generate_flock(N, simulation_options, angle);
  MultiFlock multiflock{std::vector<Flock>{flock}};
  std::vector<Vector> info_position{};
  std::vector<Vector> info_velocity{};
  std::vector<double> info_time{};
  graphics_simulation(multiflock, info_position, info_velocity, info_time); //this needs a multiflock
  if (choice == 'Y') {
    write_to_file(info_position, info_velocity, info_time);
    std::cout << "File data.txt was filled with info about the simulation" <<'\n';
  }
  return 0;
}
//*/
//main alternativo per piccola prova grafica
/*int main() {
  std::vector<Vector> you;
  std::vector<Vector> are;
  std::vector<double> useless;
Boid b1{Vector{1, 1}, Vector{0, 0}};
Boid b2{Vector{5, 5}, Vector{0, 0}};
Boid b3{Vector{10, 10}, Vector{0, 0}};
Boid b4{Vector{10, 0}, Vector{0, 0}};

Boid b5{Vector{8, 7}, Vector{0, 0}};
Boid b6{Vector{2, 3}, Vector{0, 0}};
Boid b7{Vector{4, 4}, Vector{0, 0}};
Boid b8{Vector{1.6, 3.8}, Vector{0, 0}};

Options sp{10., 0.6, 0.4, 0.1, 0.2};

Flock f1{std::vector<Boid>{b1, b2, b3, b4}, sp, 45};
Flock f2{std::vector<Boid>{b5, b6, b7, b8}, sp, 45};
MultiFlock m{std::vector<Flock>{f1,f2}};
graphics_simulation(m, you, are, useless);
}
*/