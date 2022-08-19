// compile with: g++ graphics.cpp -lsfml-graphics -lsfml-window -lsfml-system
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "rules.hpp"

auto evolve(Ambient amb, Flock& flock, int steps_per_evolution, sf::Time delta_t) {
  double const dt{delta_t.asSeconds()};

  for (int i{0}; i != steps_per_evolution; ++i) {
    flock.evolve(amb, dt);
  }
  return flock.get_boids();
}

int main() {
  ///*
   //random boid generation
  int N = 10;
  std::default_random_engine gen;

  std::normal_distribution<double> Vx(0.03, 0.05);
  std::normal_distribution<double> Vy(0.03, 0.03);
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
  Options sp{3, 0.4, 0.5, 0.3, 0.5};
  Flock f{empty, sp, 90};
  for (int i = 0.; i != N; ++i) {
    Boid b{Vector{pos_x[i], pos_y[i]}, Vector{vel_x[i], vel_y[i]}};
    f.add(b);
  }
  //*/
  /*
  Boid b1{Vector{1,1}, Vector{0,0}};
  Boid b2{Vector{5,5}, Vector{0,0}};
  Boid b3{Vector{10,10}, Vector{0,0}};
  Boid b4{Vector{10,0}, Vector{0,0}};

  Options sp{10., 0.6, 0.4, 0.1, 0.2};

  Flock f{std::vector<Boid>{b1,b2,b3,b4}, sp, 45};*/

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
    //f.get_boids();
     evolve(boundaries, f, steps_per_evolution, delta_t);

    for (auto& boid : flock_vector) {
      sprite.setPosition((boid.position).x() * scale_x,
                         (boid.position).y() * scale_y);
      window.draw(sprite);
    }

    window.display();
  }
  return 0;
}