#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <algorithm>

#include "boids.hpp"
#include "multiflock.hpp"
#include "rules.hpp"


/* void ApplyAmbient(MultiFlock& multiflock, const int width, const int height){
    std::vector<Flock> flock_vec {multiflock.get_flocks()};
    for (size_t flock = 0; flock < flock_vec.size(); flock++)
    {
        std::vector<Boid> boid_vec{flock_vec[flock].get_boids()};
        for (size_t boid = 0; boid < boid_vec.size(); boid++)
        {
            if (boid_vec[boid].position.x() < 0){
                boid_vec[boid].position = Vector(0,boid_vec[boid].position.y());
                boid_vec[boid].velocity = Vector(-boid_vec[boid].velocity.x(),boid_vec[boid].velocity.y());
            }
            if (boid_vec[boid].position.x() > width){
                boid_vec[boid].position = Vector(width,boid_vec[boid].position.y());
                boid_vec[boid].velocity = Vector(-boid_vec[boid].velocity.x(),boid_vec[boid].velocity.y());
            }
            if (boid_vec[boid].position.y() < 0){
                boid_vec[boid].position = Vector(boid_vec[boid].position.x(),0);
                boid_vec[boid].velocity = Vector(boid_vec[boid].velocity.x(),-boid_vec[boid].velocity.y());
            }
            if (boid_vec[boid].position.y() > height){
                boid_vec[boid].position = Vector(boid_vec[boid].position.x(),height);
                boid_vec[boid].velocity = Vector(boid_vec[boid].velocity.x(),-boid_vec[boid].velocity.y());
            }
        }
        flock_vec[flock] = Flock(boid_vec,flock_vec[flock].get_options(),flock_vec[flock].get_alpha());
    }
    multiflock.set(flock_vec);
}
 */
void ApplyFlockAmbient(Flock& flock, const int width, const int height){
    std::vector<Boid> boid_vec{flock.get_boids()};
    for (size_t boid = 0; boid < boid_vec.size(); boid++)
    {
        if (boid_vec[boid].position.x() < 0){
            boid_vec[boid].position = Vector(0,boid_vec[boid].position.y());
            boid_vec[boid].velocity = Vector(-boid_vec[boid].velocity.x(),boid_vec[boid].velocity.y());
        }
        if (boid_vec[boid].position.x() > width){
            boid_vec[boid].position = Vector(width,boid_vec[boid].position.y());
            boid_vec[boid].velocity = Vector(-boid_vec[boid].velocity.x(),boid_vec[boid].velocity.y());
        }
        if (boid_vec[boid].position.y() < 0){
            boid_vec[boid].position = Vector(boid_vec[boid].position.x(),0);
            boid_vec[boid].velocity = Vector(boid_vec[boid].velocity.x(),-boid_vec[boid].velocity.y());
        }
        if (boid_vec[boid].position.y() > height){
            boid_vec[boid].position = Vector(boid_vec[boid].position.x(),height);
            boid_vec[boid].velocity = Vector(boid_vec[boid].velocity.x(),-boid_vec[boid].velocity.y());
        }
    }
    flock = Flock(boid_vec,flock.get_options(),flock.get_alpha());
}

int main(){
    const int width{640};
    const int height{480};

    std::cout << "Insert Distance of neighbours, Distance of collision, separation, alignment, cohesion" << '\n';
    Options opt{};
    std::cin >> opt.distance >> opt.separation_distance >> opt.separation >> opt.alignment >> opt.cohesion;
    std::cout << '\n';

    std::cout << "Insert number of flocks: ";
    int n{};
    std::cin >> n;
    std::cout << '\n';

    MultiFlock multiflock{{}};
    std::default_random_engine gen;
    for (int flock = 0; flock < n; flock++)
    {
        std::cout << "Insert Number of boids and vision angle: ";
        int num_boids{};
        double vision_angle{};
        std::cin >> num_boids >> vision_angle; 
        std::vector<Boid> boid_vector{};
        for (int boid = 0; boid < num_boids; boid++)
        {
            Boid generated_boid {};
            std::normal_distribution<double> Vx(-100, 100);
            std::normal_distribution<double> Vy(-100, 100);
            std::uniform_real_distribution<double> Px(0, width);
            std::uniform_real_distribution<double> Py(0, height);
            Vector Pos(Px(gen),Py(gen));
            Vector Vel(Vx(gen),Vy(gen));
            Pos.print();
            generated_boid.position = Pos;
            generated_boid.velocity = Vel;
            
            boid_vector.push_back(generated_boid);
        }  
        multiflock.add(Flock{boid_vector,opt,vision_angle}); 
    }

    const int boid_radius{5};
    sf::CircleShape drawable_boid(boid_radius,3);
    drawable_boid.setOrigin(boid_radius,boid_radius);

    auto const delta_t{sf::milliseconds(1)};
    int const fps = 60;
    int const steps_per_evolution{1000 / fps};

    sf::RenderWindow window(sf::VideoMode(width,height),"Multiflock");
    window.setFramerateLimit(fps);
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
        }   

        double const dt{delta_t.asSeconds()};
        for (int i = 0; i != steps_per_evolution; i++)
        {
            /* code */
        multiflock.evolve(dt,[width,height](Flock& flock){ApplyFlockAmbient(flock,width,height);});
        //ApplyAmbient(multiflock,width,height);
        }
        
        window.clear();
        for (size_t flock = 0; flock < multiflock.get_flocks().size(); flock++)
        {
            switch (flock)
            {
            case 0:
                drawable_boid.setFillColor(sf::Color(0,255,255));
                break;
            case 1:
                drawable_boid.setFillColor(sf::Color(255,0,255));
                break;
            case 2:
                drawable_boid.setFillColor(sf::Color(255,255,0));
                break;
            
            default:
                break;
            }
            for (size_t boid = 0; boid < multiflock.get_flocks()[flock].get_boids().size(); boid++)
            {
                drawable_boid.setPosition(sf::Vector2f(multiflock.get_flocks()[flock].get_boids()[boid].position.x(),multiflock.get_flocks()[flock].get_boids()[boid].position.y()));
                window.draw(drawable_boid);
            }
            
        }
        
        window.display();   


    }

    

}
