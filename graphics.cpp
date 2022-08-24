// compile with: g++ graphics.cpp -lsfml-graphics -lsfml-window -lsfml-system
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "multiflock.hpp"
#include "rules.hpp"

auto evolve(Ambient const& amb, MultiFlock& more_flock, int steps_per_evolution,
            sf::Time delta_t) {
  double const dt{delta_t.asSeconds()};

  for (int i{0}; i != steps_per_evolution; ++i) {
    more_flock.evolve(amb, dt);
  }
  return more_flock.get_all_boids();
}

// boid random generation
//è effettivamente indipendente dalla grafica, 
//si può mettere da un'altra parte: unico problema
//è che i valori hardcoded dipendono dalla grandezza dello schermo
//gli facciamo prendere un Ambient come argomento?
Flock generate_flock(int N, Options const& sp, double angle) {
  std::default_random_engine gen;
  //should generate more flocks randomly
  //this way it only generates the same random flock again and again

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

//anche questo non c'entra con la grafica ma è un'opzione in più da aggiungere nel main
void write_to_file(std::vector<Vector> info_position,
                   std::vector<Vector> info_velocity,
                   std::vector<double> info_time) {
  std::ofstream fos;     // file output stream
  fos.open("data.txt");  // statistics printed here, to be used on root to see a
                         // graph
  for (int i = 0; i != info_time.size(); ++i) {
    fos << info_time[i] << '\t' << info_position[i].x() << '\t'
        << info_velocity[i].x() << '\n';
  }
}

void graphics_simulation(MultiFlock& more_flock,
                         std::vector<Vector> info_position,
                         std::vector<Vector> info_velocity,
                         std::vector<double> info_time) {
  double time_count{};

  auto const delta_t{sf::milliseconds(1)};
  int const fps = 60;
  int const steps_per_evolution{1000 / fps};

  unsigned const display_width = 0.7 * sf::VideoMode::getDesktopMode().width;
  unsigned const display_height = 0.7 * sf::VideoMode::getDesktopMode().height;

  auto const min_x{0.};
  auto const max_x{8.};
  auto const min_y{0.};
  auto const max_y{8.};
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
  //should use texture.getSize() to initialiase d_s (covolume)?

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

//qui come facciamo? write to file ha senso solo e ho due/tre colonne
//così posso farci un TGraph su root,
//forse Fabio ha delle tabelle dove da i valori in ouput per vari flock
//quindi nel caso questo farà lui
    if (more_flock.size() == 1) {
      auto f = (more_flock.get_flocks())[0];
      info_position.push_back(get_distance_mean_RMS(f));
      info_velocity.push_back(get_speed_mean_RMS(f));
    }
    //e se ho più flock? come cambiano info_position e info_velocity?

    time_count += delta_t.asSeconds();
    info_time.push_back(time_count);
  }
}

int main() {
  //Options simulation_options{3, 0.4, 0.5, 0.4, 0.5};
  std::cout << "------Boid simulation-------" << '\n'
            << "Plase enter values for simulation parameters:" << '\n'
            << "Number of boids for each flock (default value = 15):" << '\n'
            << "Distance of neighbours (default value = 3):" << '\n'
            << "Distance of collision (default value = 0.4):" << '\n'
            << "Separation rule (default value = 0.5):" << '\n'
            << "Alignment rule (default value = 0.4):" << '\n'
            << "Cohesion rule (default value = 0.5):" << '\n';
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

  Flock f1{std::vector<Boid>{b1, b2, b3, b4}, simulation_options, angle}; 
  //if this construct works the angle is 180
  Flock f2{std::vector<Boid>{b5, b6, b7, b8}, simulation_options, angle};

  MultiFlock multiflock{std::vector<Flock>{flock1, f1, f2}};
  std::vector<Vector> info_position{};
  std::vector<Vector> info_velocity{};
  std::vector<double> info_time{};
  graphics_simulation(multiflock, info_position, info_velocity,
                      info_time); 
  if (choice == 'Y' && multiflock.size() == 1) {
    //again, what to print if i have many flocks?
    write_to_file(info_position, info_velocity, info_time);
    std::cout << "File data.txt was filled with info about the simulation"
              << '\n';
  }
  return 0;
}