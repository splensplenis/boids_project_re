#ifndef BOID_SPEC_HPP
#define BOID_SPEC_HPP

#include<string>
#include<array>
#include<algorithm>
#include<stdexcept>
#include<cassert>

namespace BoidSpec{

/*  Class which hold the boid as the needs of the graphics
    Store this values:
    - name
    - color
    - vision angle
    Class invariants are:
    - name: none
    - color: between 0 and 1 
    - vision angle: positive */
class boid_specie
{
    std::string m_name;
    std::array<float,3> m_color; //R,G,B every number between 0 and 1
    int m_vision_angle; //angle in degrees between 0 and 360

    public:
    //generic boid constructor
    boid_specie(std::string name, std::array<float,3> color, int vision_angle) : 
        m_name{name}, 
        m_color{color},
        m_vision_angle{vision_angle}
        {
            std::for_each(m_color.begin(),m_color.end(),[](float col){if (col > 1.0f || col < 0.0f){throw std::invalid_argument{"color bigger than 1 or less than 0"};}});
            if (m_vision_angle < 0){throw std::invalid_argument{"vision angle cannot be negative"};}
            if (m_vision_angle > 360){
                m_vision_angle = (m_vision_angle%360 == 0 ? 360 : m_vision_angle%360);
            }

            assert(m_color[0]<=1.0f && m_color[0]>=0.0f);
            assert(m_color[1]<=1.0f && m_color[1]>=0.0f);
            assert(m_color[2]<=1.0f && m_color[2]>=0.0f);
            assert(m_vision_angle>=0 && m_vision_angle<=360);
        }
    
    //default constructor: generate a standard boid
    boid_specie() : m_name{"Std Boid"}, m_color{std::array<float,3>{0,0,0}}, m_vision_angle{360} {}

    void SetName(std::string name){m_name = name;} //set the name of the boid
    std::string GetName() const {return m_name;} //get the name of the boid
    
    //Set boid color
    void SetColor(std::array<float,3> color){
        //check if color inserted is between 0 and 1, if not throw an error
        std::for_each(color.begin(),color.end(),[](float col){if (col > 1.0f || col < 0.0f){throw std::invalid_argument{"color bigger than 1 or less than 0"};}});
        assert(color[1]<=1.0f && color[1]>=0.0f);
        assert(color[2]<=1.0f && color[2]>=0.0f);
        assert(color[0]<=1.0f && color[0]>=0.0f);
        m_color = color;
    }
    std::array<float,3> GetColor() const {return m_color;} //get the boid color

    //set the vision angle
    void SetVisionAngle(int vision_angle){
        if (vision_angle < 0){throw std::invalid_argument{"vision angle cannot be negative"};} //the angle cannot be negative
        //if the angle is bigger than 360 normalize it
        if (vision_angle > 360){
            vision_angle = (vision_angle%360 == 0 ? 360 : vision_angle%360);
        }  
        m_vision_angle = vision_angle;
        assert(m_vision_angle>=0 && m_vision_angle<=360);
    }
    int GetVisionAngle() const {return m_vision_angle;} //get boid vision angle

    //get the boid color in standard 0-255 value
    std::array<int,3> GetColorInt () const {
        std::array<int,3> color_int {};
        std::transform(m_color.begin(), m_color.end(), color_int.begin(), [](float color){return static_cast<int>((255 * color));});
        return color_int;
    }

};
}


#endif