#ifndef UTILS_HPP
#define  UTILS_HPP

#include<array>
#include<map>
#include<string>
#include<cassert>
#include<cmath>

#include "imgui.h"
#include "boid_spec.hpp"
#include<SFML/System/Vector2.hpp>

inline void TODO(){return;}

//Convert a color std::array to an ImVec4 for ImGui
inline ImVec4 ArrayToImVec4(std::array<float,3> color, float w = 0){
    ImVec4 imgui_color (color[0],color[1],color[2],w);
    return imgui_color;
}

//Convert a color std::array to an ImVec4 for ImGui
inline ImVec4 ArrayToImVec4(std::array<float,4> color){
    ImVec4 imgui_color (color[0],color[1],color[2],color[3]);
    return imgui_color;
}

//get the distance between two coord
float getDistance(const sf::Vector2f& first,const sf::Vector2f& second ){
    return std::sqrt(std::pow((first.x-second.x),2)+std::pow((first.y-second.y),2));
}

//check if a boid is over another drawn boid
bool check_if_isolated(std::vector<std::vector<sf::Vector2f>>& boid_coord_vector, const sf::Vector2f& boid_coord, const int boid_radius){
    bool is_isolated {true};
    std::for_each(boid_coord_vector.begin(),boid_coord_vector.end(),[&is_isolated, &boid_coord, boid_radius](std::vector<sf::Vector2f>& specie_vector){
        is_isolated &= std::none_of(specie_vector.begin(),specie_vector.end(), [&](sf::Vector2f first){
            return(getDistance(first,boid_coord) < (boid_radius)*2);
        });
    });

    return is_isolated;
}

//hold the simulation flags 
struct SimulationFlagHolder
{
    enum class Flag {Modifiable,Running}; //Keep track if the simulation is started or not
    enum class PausedFlag {Paused,Unpaused}; //Keep track if the simulation is paused or not

    Flag flag{Flag::Modifiable};
    PausedFlag paused_flag{PausedFlag::Unpaused};
};

enum class PaintMoveFlag {Move,Draw}; //Flag that keep track if the drawing is enable or disabled -- if drawing is disabled moving is enabled
enum class DrawEraseFlag {Draw,Erase}; //Flag that keep track if the brush or the eraser is selected
enum class LastDeletedFlag {Raised, Unraised}; //Flag that is raised if there is an attempt to erase the last element of a boid_spec vector
enum class InvalidInputFlag {Valid, Invalid}; //Flag raised if an invalid boid file is loaded
enum class NotFoundFlag {Found, NotFound}; //Flag raised if an inexistent file is loaded

namespace boidErr {
    struct lastDeletedError{};
    struct fileNotFound{};
    struct invalidInput{};
}


#endif