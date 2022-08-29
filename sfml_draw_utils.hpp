#ifndef SFML_DRAW_UTILS_HPP
#define SFML_DRAW_UTILS_HPP

#include <SFML/Graphics.hpp>
#include<cmath>

//hold the bools that keep track of the arrow pressed in order to allow arrow movement
struct MovementDirectionHolder
{
    bool move_right{false};
    bool move_left{false};
    bool move_up{false};
    bool move_down{false};
};

void move_view_with_arrows(const bool right,const bool left,const bool up,const bool down, sf::View& view,const float entity, const sf::FloatRect& borders){
    sf::Vector2f movement {sf::Vector2f(0,0)};
    if (right){movement += sf::Vector2f(entity,0);}
    if (left){movement += sf::Vector2f(-entity,0);}
    if (up){movement += sf::Vector2f(0,entity);}
    if (down){movement += sf::Vector2f(0,-entity);}

    if (borders.contains(view.getCenter() + movement)){
        view.move(movement);
    }
}

sf::Color ArrayToColor (const std::array<int,3>& color_array){
    return sf::Color(color_array[0],color_array[1],color_array[2]);
}

/*  boid preview is the disposition of the boids in the brush in order to preview their position on the map if drawn
    the boids are disposed in circles over the center */
std::vector<sf::Vector2f> make_boid_preview(sf::CircleShape& boid_bg, const sf::Vector2f& worldPos, const float boid_brush_border, const int brush_density, const int brush_size){
    std::vector<sf::Vector2f> boid_preview_vector {};
    const double pi = std::acos(-1.0);

    boid_bg.setOrigin(boid_bg.getRadius(),boid_bg.getRadius()); //set the origin of the boid in the center in order to apply all the transformations here
    boid_bg.setPosition(worldPos); //first boid is on mouse position

    boid_preview_vector.push_back(boid_bg.getPosition()); //the first boid (in the center of the brush) is always drawn

    for (int row = 1; row != brush_size; ++row){

        //row_radius represent the distance between the centre of the brush and the considered ring of previewed boids
        //it's computated in this way:
        //the distance between two rows is: (radius of the boid in the inner row) + (separation) + (radius of the boid in the outer row)
        //since the boids have all the same radius: distance between two rows = (2*radius) + (separation)
        //the distance between the center and the n row is n*(distance between two rows)
        double row_radius {row*(2*boid_bg.getRadius()+boid_brush_border)}; 
        //the maximum number of boid in the given row is computed as follow:
        //we draw a regular polygon in the circumference corresponding to the row
        //in each corner of this polygon there is a boid
        //the side of this shape is equal to: 2*(boid radius) + (separation)
        //let alpha the angle vertex1-centre-vertex2 (vertex1 and vertex2 are near): the circumference angle will be (alpha/2)
        //the triangle diameter-vertex1-vertex2 is rectangular so:
        //diameter*sin(circumference angle) = distance between vertex1 and vertex2 (the side of the polygon)
        //circ angle = arcsin(side/diameter) -> alpha = 2*arcsin(side/diameter)
        //but diameter of the n row is 2*row_radius -> 2*n*((2*radius)+separation)
        //side is equal to 2*(radius) + (separation)
        //so alpha = 2*arcsin(1/2n)
        double single_angle {2*std::asin((1.0)/static_cast<double>(2*row))};
        //max number of boid is the number of sides of the poligon:
        //2*pi / alpha
        int max_num_of_boids {static_cast<int>(std::round((2*pi)/single_angle))};
        //selected number of boids is the maximum number of boids multiplied by the brush density and rounded to int
        int selected_num_of_boids {static_cast<int>(std::round(max_num_of_boids*(brush_density/100.0)))};
        //draw angle represents the angle between two drawn boids given the brush density
        double draw_angle {(static_cast<double>(max_num_of_boids)/static_cast<double>(((selected_num_of_boids == 0) ? 1 : selected_num_of_boids)))*single_angle};

        for (int boid_pos = 0; boid_pos != selected_num_of_boids; ++boid_pos){
            boid_bg.setPosition(worldPos);
            boid_bg.move(row_radius*std::cos(boid_pos*draw_angle),row_radius*std::sin(boid_pos*draw_angle));
            //because the math above isn't perfect check if a boid is over an already drawn boid
            //in this case don't push its coordinates in the coord vector
            if (std::none_of(boid_preview_vector.begin(), boid_preview_vector.end(), [&](sf::Vector2f first){return(getDistance(first,boid_bg.getPosition()) < (boid_bg.getRadius())*2);})){
                boid_preview_vector.push_back(boid_bg.getPosition());
            }
        }

    }

    return boid_preview_vector;
}

//draw the boids on the window
inline void draw_boids(const std::vector<std::vector<sf::Vector2f>>& boid_coord_vector, const std::vector<BoidSpec::boid_specie>& boid_spec_vector, sf::CircleShape& boid_shape, sf::RenderWindow& window){
    for (size_t  flock = 0;  flock < boid_coord_vector.size(); ++flock){
        //For each flock set the boid color to the correct color in the flock
        boid_shape.setFillColor(ArrayToColor(boid_spec_vector[flock].GetColorInt()));
        //Draw all boids
        std::for_each(boid_coord_vector[flock].begin(), boid_coord_vector[flock].end(), [&](const sf::Vector2f& boid){
            boid_shape.setPosition(boid);
            window.draw(boid_shape);
        });
    }      
}



#endif