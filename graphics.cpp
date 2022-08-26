// compile with:
// g++ -Wall -Wextra -fsanitize=address vector.cpp boids.cpp graphics.cpp -lsfml-graphics -lsfml-window -lsfml-system

//è normale che chiudendo la window dia memory leak?

#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>

#include <fstream>
//#include <iomanip>
#include <iostream>

#include "boids.hpp"
#include "multiflock.hpp"
#include "rules.hpp"
#include "boids_random.hpp"

// simulation including borders:

struct Ambient {  // rectangluar ambient
  Vector top_left_corner{};
  Vector bottom_right_corner{};
};

void avoid_boundaries(Ambient const& ambient, Boid& boid) {
  if ((boid.position).x() > (ambient.bottom_right_corner).x()) {
    Vector v1{(ambient.bottom_right_corner).x(), (boid.position).y()};
    // in realtà anche y non è quella...approssimazione
    Vector v2{-(boid.velocity).x(), (boid.velocity).y()};
    boid.position = v1;
    boid.velocity = v2;
  }
  if ((boid.position).y() > (ambient.bottom_right_corner).y()) {
    Vector v1{(boid.position).x(), (ambient.bottom_right_corner).y()};
    Vector v2{(boid.velocity).x(), -(boid.velocity).y()};
    boid.position = v1;
    boid.velocity = v2;
  }
  if ((boid.position).x() < (ambient.top_left_corner).x()) {
    Vector v1{(ambient.top_left_corner).x(), (boid.position).y()};
    Vector v2{-(boid.velocity).x(), (boid.velocity).y()};
    boid.position = v1;
    boid.velocity = v2;
  }
  if ((boid.position).y() < (ambient.top_left_corner).y()) {
    Vector v1{(boid.position).x(), (ambient.top_left_corner).y()};
    Vector v2{(boid.velocity).x(), -(boid.velocity).y()};
    boid.position = v1;
    boid.velocity = v2;
  }
}

auto closed_ambient_evolve(Ambient const& amb, MultiFlock& more_flock,
                           int steps_per_evolution, sf::Time delta_t) { //<---- !!
  double const dt{delta_t.asSeconds()};

  for (int i{0}; i != steps_per_evolution; ++i) {
    more_flock.evolve(dt);
    //+ correzione ai boundaries da mettere qua
    auto all_boids = more_flock.get_all_boids();
    std::for_each(all_boids.begin(),
                  all_boids.end(), [&](Boid& boid1) { avoid_boundaries(amb, boid1);});
  //ora coi boids evoluti che ci faccio? devo rimetterli dentro al flock perchè si devono evolvere ancora
  //e devono uscire dal loop una volta finito tutto
  }
  return more_flock.get_all_boids();
}

void graphics_simulation(MultiFlock& more_flock,
                         std::vector<std::vector<Vector>> info_position,
                         std::vector<std::vector<Vector>> info_velocity,
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
  // should use texture.getSize() to initialiase d_s (covolume)?

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    window.clear(sf::Color::White);

    auto const flock_vector = closed_ambient_evolve(
        boundaries, more_flock, steps_per_evolution, delta_t);

    for (auto& boid : flock_vector) {
      sprite.setPosition((boid.position).x() * scale_x,
                         (boid.position).y() * scale_y);
      window.draw(sprite);
    }

    window.display();

    info_position.push_back(more_flock.get_all_distance_mean_RMS());
    info_velocity.push_back(more_flock.get_all_speed_mean_RMS());

    time_count += delta_t.asSeconds();
    info_time.push_back(time_count);
  }
}

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
  std::cout << "----Boids simulation: implemented with graphics-----" <<'\n';
  Options simulation{1, 0.5, 0.3, 0.1, 1};
  double view_angle{90};
  MultiFlock random_multif = generate_multiflock(3, 10, simulation, view_angle);

  std::vector<std::vector<Vector>> info_position{};
  std::vector<std::vector<Vector>> info_velocity{};
  std::vector<double> info_time{};

  graphics_simulation(random_multif, info_position, info_velocity, info_time);

  //write to file

}