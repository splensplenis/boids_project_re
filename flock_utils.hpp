#ifndef FLOCK_UTILS_HPP
#define FLOCK_UTILS_HPP

#include<vector>
#include<random>
#include<algorithm>
#include<cassert>

#include <SFML/System/Vector2.hpp>
#include "multiflock.hpp"
#include "boids.hpp"
#include "boid_spec.hpp"
#include "io.hpp"


//generate a multiflock from the coordinates and the boid specs
inline MultiFlock GenerateMultiflock(const std::vector<std::vector<sf::Vector2f>>& coord_vector, const std::vector<BoidSpec::boid_specie>& boid_specs,Options opt,double speed_mean, double speed_SD){
    assert(coord_vector.size() == boid_specs.size());
    std::vector<Flock> flock_vec{}; //new empty vector of flocks
    for (size_t flock = 0; flock < coord_vector.size(); ++flock)
    {
        std::vector<Boid> boid_vec{}; //new empty vector of boids 
         //vision angle in boid specs goes from 0 to 360, vision angle in simulation boid goes from 0 to 180 
         //so the angle is split in half
        float vision_angle{(float)boid_specs[flock].GetVisionAngle()/2};
        //for each flock's coordinates
        std::for_each(coord_vector[flock].begin(),coord_vector[flock].end(),[&](const sf::Vector2f& boid_coord){
            Boid boid{}; //new simulation boid
            boid.position = Vector(boid_coord.x, boid_coord.y); //set sim boid position equal to graphic boid coord
            std::default_random_engine eng;
            std::normal_distribution<> velocity_randomizer (speed_mean, speed_SD); //starting velocity is generated randomly following the normal distribution
            boid.velocity = Vector(velocity_randomizer(eng), velocity_randomizer(eng)); //generating starting velocity and assign it to the sim boid
            boid_vec.push_back(boid); 
        });
        assert(boid_vec.size() == coord_vector[flock].size());
        flock_vec.push_back(Flock(boid_vec,opt,vision_angle));
    }
    assert(flock_vec.size() == coord_vector.size());
    return MultiFlock(flock_vec);    
}


/*  Apply ambient constraints to a flock
    When the computed position of a boid isn't in the boundaries the boid bounce back elastically:
    - Its position is brought back on the edge the surpassed border
    - The component of the velocity in the direction of the trespass is inverted of sign */
inline void ApplyAmbient(Flock& flock, const unsigned int width, const unsigned int height){
    std::vector<Boid> boid_vec{flock.get_boids()};
    for (size_t boid = 0; boid < boid_vec.size(); boid++)
    {
        //check left border
        if (boid_vec[boid].position.x() < 0){
            boid_vec[boid].position = Vector(0,boid_vec[boid].position.y());
            boid_vec[boid].velocity = Vector(-boid_vec[boid].velocity.x(),boid_vec[boid].velocity.y());
        }
        //check right border
        if (boid_vec[boid].position.x() > width){
            boid_vec[boid].position = Vector(width,boid_vec[boid].position.y());
            boid_vec[boid].velocity = Vector(-boid_vec[boid].velocity.x(),boid_vec[boid].velocity.y());
        }
        //check upper border
        if (boid_vec[boid].position.y() < 0){
            boid_vec[boid].position = Vector(boid_vec[boid].position.x(),0);
            boid_vec[boid].velocity = Vector(boid_vec[boid].velocity.x(),-boid_vec[boid].velocity.y());
        }
        //check bottom border
        if (boid_vec[boid].position.y() > height){
            boid_vec[boid].position = Vector(boid_vec[boid].position.x(),height);
            boid_vec[boid].velocity = Vector(boid_vec[boid].velocity.x(),-boid_vec[boid].velocity.y());
        }
    }
    //replace the flock with the new values
    flock = Flock(boid_vec,flock.get_options(),flock.get_alpha());
}


//generate a ccoordinates vector from a multiflock
inline std::vector<std::vector<sf::Vector2f>> GenerateCoordVector (const MultiFlock& multiflock){
    std::vector<Flock> flock_vec {multiflock.get_flocks()};
    std::vector<std::vector<sf::Vector2f>> multiflock_coord_vector {};

    //for each flock in multiflock
    std::for_each(flock_vec.begin(),flock_vec.end(),[&](const Flock& flock){
        std::vector<Boid> boid_vec {flock.get_boids()};
        std::vector<sf::Vector2f> flock_coord_vector {};
        //for each boid in single flock
        for (size_t boid{}; boid!=boid_vec.size();++boid){
            sf::Vector2f boid_coord {(float)boid_vec[boid].position.x(),(float)boid_vec[boid].position.y()};
            flock_coord_vector.push_back(boid_coord);
        }
        multiflock_coord_vector.push_back(flock_coord_vector);
    });

    return multiflock_coord_vector;
}

//make graphics option from user-changeable simulation parameters and fix min_distance
inline Options MakeOptions(const SimulationParameters& parameters, const double min_distance){
    Options opt{};
    opt.alignment = parameters.alignment;
    opt.cohesion = parameters.cohesion;
    opt.separation = parameters.separation;
    opt.distance = parameters.vision_distance;
    opt.separation_distance = min_distance;
    return opt;
}

#endif