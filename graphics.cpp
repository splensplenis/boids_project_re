// compile with: g++ graphics.cpp -lsfml-graphics -lsfml-window -lsfml-system
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "rules.hpp"

auto evolve(Ambient amb, Flock& flock, int steps_per_evolution,
            sf::Time delta_t) {
  double const dt{delta_t.asSeconds()};

  for (int i{0}; i != steps_per_evolution; ++i) {
    flock.evolve(amb, dt);
  }
  return flock.get_boids();
}

int main(int argc, char* argv[]) {
  std::ofstream fos;     // file output stream
  fos.open("data.txt");  // statistics printed here, to be used on root

  // random boid generation
  int N = 5;
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
  Options sp{3, 0.4, 0.9, 0.9, 0.5};
  Flock f{empty, sp, 180};
  for (int i = 0.; i != N; ++i) {
    Boid b{Vector{pos_x[i], pos_y[i]}, Vector{vel_x[i], vel_y[i]}};
    f.add(b);
  }

  /*
  //questo funziona solo con get neighbours, non view neighbours
  //(angolo di visione ha bisogno di velocità di volo non-nulla)
  Boid b1{Vector{1,1}, Vector{0,0}};
  Boid b2{Vector{5,5}, Vector{0,0}};
  Boid b3{Vector{10,10}, Vector{0,0}};
  Boid b4{Vector{10,0}, Vector{0,0}};

  Boid b5{Vector{8,7}, Vector{0,0}};
  Boid b6{Vector{2,3}, Vector{0,0}};
  Boid b7{Vector{4,4}, Vector{0,0}};
  Boid b8{Vector{1.6,3.8}, Vector{0,0}};

  Options sp{10., 0.6, 0.4, 0.1, 0.2};

  Flock f1{std::vector<Boid>{b1,b2,b3,b4}, sp, 45};
  Flock f2{std::vector<Boid>{b5,b6,b7,b8}, sp, 45};
  */

  auto const delta_t{sf::milliseconds(1)};
  int const fps = 30;
  int const steps_per_evolution{1000 / fps};

  double time_count = 0.;

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

    auto const flock_vector1 =
        // f.get_boids();
        evolve(boundaries, f, steps_per_evolution, delta_t);
    // auto const flock_vector2 = evolve(boundaries, f2, steps_per_evolution,
    // delta_t);

    for (auto& boid : flock_vector1) {
      sprite.setPosition((boid.position).x() * scale_x,
                         (boid.position).y() * scale_y);
      window.draw(sprite);
    }
    /*for (auto& boid : flock_vector2) {
      sprite.setPosition((boid.position).x() * scale_x,
                         (boid.position).y() * scale_y);
      window.draw(sprite);
    }*/

    window.display();
    time_count += delta_t.asSeconds();
    fos <<  time_count << '\t' << (velocity_parameters(f)).x() << '\t' << (distance_parameters(f)).x()
        << '\n';
  }
  return 0;
}