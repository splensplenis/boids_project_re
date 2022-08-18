// compile with: g++ graphics.cpp -lsfml-graphics -lsfml-window -lsfml-system
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <iostream>
#include <random>

#include "boids.hpp"

auto evolve(Flock& flock, int steps_per_evolution, sf::Time delta_t) {
  double const dt{delta_t.asSeconds()};

  for (int i{0}; i != steps_per_evolution; ++i) {
    flock.evolve(dt);
  }
  return flock.get_flock();
}

int main() {
  int N = 10;
  std::default_random_engine gen;

  std::normal_distribution<double> Vx(2, 0.5);
  std::normal_distribution<double> Vy(2., 0.5);
  std::vector<double> vel_x{};
  std::vector<double> vel_y{};

  std::uniform_real_distribution<double> Xx(-3., 3.);
  std::uniform_real_distribution<double> Xy(-5., 5.);
  std::vector<double> pos_x{};
  std::vector<double> pos_y{};
  for (int n = 0; n != N; ++n) {  // coordinates are casually generated
    vel_x.push_back(Vx(gen));
    vel_y.push_back(Vy(gen));
    pos_x.push_back(Xx(gen));
    pos_y.push_back(Xy(gen));
  }
  std::vector<Boid> empty{};
  /*
  Boid b1{Vector{2., 1.}, Vector{0., 1.}};
  Boid b2{Vector{4., 3.}, Vector{-1., 0.}};
  Boid b3{Vector{1., 4.}, Vector{2., 0.5}};
  */
  Species sp{10., 0.1, 0.4, 0.6, 0.7, 180};
  Flock f{empty, sp};
  for (int i = 0.; i != N; ++i) {
    Boid b{Vector{pos_x[i], pos_y[i]}, Vector{vel_x[i], vel_y[i]}};
    f.add(b);
  }

  auto const delta_t{sf::milliseconds(1)};
  int const fps = 30;
  int const steps_per_evolution{1000 / fps};

  unsigned const display_width = 0.9 * sf::VideoMode::getDesktopMode().width;
  unsigned const display_height = 0.9 * sf::VideoMode::getDesktopMode().height;

  auto const min_x{-10.};
  auto const max_x{10.};
  auto const min_y{-10.};
  auto const max_y{10.};
  auto const scale_x = display_width / (max_x - min_x);
  auto const scale_y = display_height / (max_y - min_y);

  sf::RenderWindow window(sf::VideoMode(display_width, display_height),
                          "Boids!");
  window.setFramerateLimit(fps);

  sf::Texture texture;
  if (!texture.loadFromFile("image.png", sf::IntRect(10, 10, 20, 20))) {
    std::cout << "ops" << '\n';
  }
  sf::Sprite sprite{};
  sprite.setTexture(texture);

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    window.clear(sf::Color::White);

    auto const flock_vector = evolve(f, steps_per_evolution, delta_t);

    for (auto& boid : flock_vector) {
      sprite.setPosition((boid.position).x() * scale_x,
                         (boid.position).y() * scale_y);
      window.draw(sprite);
    }

    window.display();
  }
  return 0;
}