#ifndef IO_HPP
#define IO_HPP

#include<iostream>
#include<iomanip>
#include<stdexcept>
#include<vector>
#include<array>
#include<string>
#include<cassert>
#include<algorithm>
#include<random>
#include<fstream>

#include<SFML/System/Vector2.hpp>

#include "boid_spec.hpp"
#include "utils.hpp"
#include "simulation_utils.hpp"

struct BoidInputData{
    int number{0};
    std::string name{""};
    int vision_angle{0};
};

struct SimulationParameters{
    float cohesion {0};
    float separation {0};
    float alignment {0};
    int vision_distance {0};
};

struct InputData{
    std::vector<BoidInputData> boid_input_data {};
    SimulationParameters simulation_parameters {};
};

inline void Check (bool status){
    if (!status){
        throw std::invalid_argument{"Not a valid input"};
    }
}

//Read input from keyboard
inline InputData KeyboardInput(std::istream& is = std::cin){
    int number_of_flock{0};

    std::cout << "Insert number of flock in the simulation, 0 if you want to access the editor" << '\n';
    is >> number_of_flock;
    std::cout << '\n';

    Check(is.good());
    Check(number_of_flock >= 0); //number of flocks must be positive
    assert (number_of_flock >= 0);

    if (number_of_flock == 0){
        //if number if flock is equal to zero (access the editor) return an empty struct 
        InputData input_data{};
        return input_data;
    }

    assert (number_of_flock > 0);

    std::vector<BoidInputData> input_boid_vector {};
    for (int flock = 0; flock != number_of_flock; ++flock){
        BoidInputData boid_input {};

        //Reading boid's name, its vision angle and the number of boids in the flock
        std::cout << "Insert name, vision angle (in degrees) and number of boid in the flock" << '\n';
        is >> boid_input.name >> boid_input.vision_angle >> boid_input.number;
        std::cout << '\n';
        
        Check(is.good());
        Check(boid_input.name != "");
        Check(boid_input.vision_angle >= 0);
        Check(boid_input.number >= 0);

        input_boid_vector.push_back(boid_input);
    }

    SimulationParameters simulation_parameters {};

    //read simulation parameters
    std::cout << "Insert simulation parameters: alignment, cohesion, separation and vision distance" << '\n';
    is >> simulation_parameters.alignment >> simulation_parameters.cohesion >> simulation_parameters.separation >> simulation_parameters.vision_distance ;
    std::cout << '\n';

    Check(is.good());
    Check(simulation_parameters.alignment >= 0);
    Check(simulation_parameters.cohesion >= 0);
    Check(simulation_parameters.separation >= 0);
    Check(simulation_parameters.vision_distance >= 0);

    assert(simulation_parameters.alignment >= 0  && simulation_parameters.cohesion >= 0 && simulation_parameters.separation >= 0 && simulation_parameters.vision_distance >= 0);
    assert(input_boid_vector.size() == static_cast<unsigned int>(number_of_flock));

    InputData input_data{};
    input_data.simulation_parameters = simulation_parameters;
    input_data.boid_input_data = input_boid_vector;

    return input_data;
    
}

//check if data read in input follows the required constraints
inline void CheckInputCostraints(const InputData& input_data, float sim_low_lim, float sim_up_lim){
    Check(input_data.simulation_parameters.alignment >= sim_low_lim);
    Check(input_data.simulation_parameters.alignment <= sim_up_lim);
    Check(input_data.simulation_parameters.cohesion >= sim_low_lim);
    Check(input_data.simulation_parameters.cohesion <= sim_up_lim);
    Check(input_data.simulation_parameters.separation >= sim_low_lim);
    Check(input_data.simulation_parameters.separation <= sim_up_lim);

    assert(input_data.simulation_parameters.alignment >= sim_low_lim && input_data.simulation_parameters.alignment <= sim_up_lim);
    assert(input_data.simulation_parameters.cohesion >= sim_low_lim && input_data.simulation_parameters.cohesion <= sim_up_lim);
    assert(input_data.simulation_parameters.separation >= sim_low_lim && input_data.simulation_parameters.separation <= sim_up_lim);
}

//assign read input to local variables
inline void AssignInputValues (const InputData& input_data, std::vector<BoidSpec::boid_specie>& boid_spec_vector, std::vector<std::vector<sf::Vector2f>>& coord_vector, const sf::Vector2f& borders, SimulationFlagHolder& holder, SimulationParameters& simulation_parameters, const int boid_radius){
    
    //if 0 flock was added (editor access chosen)
    if (input_data.boid_input_data.empty()){
        boid_spec_vector.push_back(BoidSpec::boid_specie()); //add a std boid to the flock vector
        coord_vector.push_back({}); //add an empty coord to the coord vector
        holder.flag = SimulationFlagHolder::Flag::Modifiable; //set the simulation flag as modifiable

        assert(boid_spec_vector.size() == 1);
        assert(coord_vector.size() == 1);
        assert(boid_spec_vector.size() == coord_vector.size());
        return;
    
    } else {
        //if one or more flocks was added
        std::default_random_engine gen;

        //for each flock added
        std::for_each(input_data.boid_input_data.begin(), input_data.boid_input_data.end(), [&](BoidInputData boid_input){
            
            //Give to the boid a random color
            std::array<float,3> color {0,0,0};
            std::uniform_real_distribution<float> color_distribution(0.0, 1.0);
            std::transform(color.begin(), color.end(), color.begin(), [&](float shade){
                shade = color_distribution(gen);
                return shade;
            });
            boid_spec_vector.push_back(BoidSpec::boid_specie(boid_input.name, color, boid_input.vision_angle));

            //Position randomly the boids on the map following a uniform distribution
            std::uniform_real_distribution<float>  coord_x_distribution(boid_radius, borders.x);
            std::uniform_real_distribution<float>  coord_y_distribution(boid_radius, borders.y);
            
            std::vector<sf::Vector2f> coordinates {};
            //place on the map the selected number of boid
            for (int boid = 0; boid != boid_input.number; ++boid)
            {
                int count {0};
                sf::Vector2f single_coord(coord_x_distribution(gen), coord_y_distribution(gen)); //give the boid a coord  

                //if a boid is over an other boid change its position
                while ((!check_if_isolated(coord_vector,single_coord,1))  || ( (!std::all_of(coordinates.begin(), coordinates.end(), [&](sf::Vector2f coord){return (getDistance(coord, single_coord)>(2*boid_radius));}))))
                {   
                    single_coord = sf::Vector2f(coord_x_distribution(gen), coord_y_distribution(gen));
                    ++count;
                    if (count > 1000){
                        //if too many attempt to find a new position were made raise an exception
                        throw std::runtime_error{"Not enough space for selected number of boids"};
                    }

                }
                 
                coordinates.push_back(single_coord);
            }
            
            coord_vector.push_back(coordinates);
            holder.flag = SimulationFlagHolder::Flag::Running;
            

            assert(boid_spec_vector.size() == coord_vector.size());
        });

        //assign the input values to the local stored simulation parameters
        simulation_parameters.alignment = input_data.simulation_parameters.alignment;
        simulation_parameters.cohesion = input_data.simulation_parameters.cohesion;
        simulation_parameters.separation = input_data.simulation_parameters.separation;
        simulation_parameters.vision_distance = input_data.simulation_parameters.vision_distance;
    }

}

//return the lenght iof the longest boid name in the simulation
inline size_t MaxNameLength (const std::vector<BoidSpec::boid_specie>& species){
    //find the longest boid name
    BoidSpec::boid_specie longest_boid_name {*std::max_element(species.begin(), species.end(), [](BoidSpec::boid_specie a, BoidSpec::boid_specie b){
        return (a.GetName().size() < b.GetName().size() );
    })};

    return longest_boid_name.GetName().size();
}

/* write the horizontal dashed line to box the headers*/
inline void WriteSeparationRow (const long unsigned int columns, const long unsigned int max_lenght){
    //limited to columns-1 because the last drawing must return carriage 
    for (long unsigned int j = 0; j != columns-1; ++j){
        //the dashed line lenght is max_lenght +2 beacause the name must be surrounded by one spaces for side
        std::cout << std::string(max_lenght + 2,'-');
        std::cout << "||"; //column separator
    }
    std::cout << std::string(max_lenght + 2,'-');
    std::cout << "||" << '\n';
}

//write subcolumns data headers for std output table
inline void WriteDataHeader(const size_t columns, const size_t max_lenght, const size_t sub_lenght){
    for (size_t j = 0; j != columns; ++j){
        //every row is aligned to left and (ecxept the last one) occupy a fixed space
        //the last one occupy the remaining space in order to align its end with the end of the name header
        //the computation space for the last row is:
        //max lenght minus space occupied by the three columns behind
        //space occupied by a column: sub_lenght + space at begin + space at end + column separation pipe -> sub_lenght+3
        //space occupied by the three column behind: 3*(space occupied by one column) -> 3*(sub_lenght+3)
        //space occupied by last column: max_lenght -(3*(sub_lenght+3))
        std::cout << ' ' << std::left << std::setw(sub_lenght) << "Mean Dist" << " |";
        std::cout << ' ' << std::left << std::setw(sub_lenght) << "Mean Dist SD" << " |";
        std::cout << ' ' << std::left << std::setw(sub_lenght) << "Mean Vel" << " |";
        std::cout << ' ' << std::left << std::setw((max_lenght -(3*(sub_lenght+3)))) << "Mean Vel SD" << " ||" ;
    }
        std::cout << '\n';
}

//write name headers from std output table
inline void WriteHeaders(const std::vector<BoidSpec::boid_specie>& species, size_t max_lenght, const unsigned int min_row_lenght, const unsigned int sub_lenght){
    
    //if the longest boid name in the simulation is shorter than the minimum row lenght than max lenght became minimum row lenght
    max_lenght = {max_lenght < min_row_lenght ? min_row_lenght : max_lenght }; 
    const size_t n_rows{5};

    for (int i = 0; i != n_rows; ++i){
        
        switch (i)
        {
        case 0:
            WriteSeparationRow(species.size(),max_lenght);
            break;

        case 1:
            //write names
            //every name is aligned to left and occupy the space given by max_lenght
            for (size_t j = 0; j != species.size(); ++j){
                std::cout << ' ' << std::left << std::setw(max_lenght) << species[j].GetName() << " ||";
            }
                std::cout << '\n';
            break;
        
        case 2:
            WriteSeparationRow(species.size(),max_lenght);
            break;

        case 3:
            WriteDataHeader(species.size(), max_lenght, sub_lenght);
            break;
        case 4:
            WriteSeparationRow(species.size(),max_lenght);
            break;

        default:
            break;
        }

    }

}

//write on std output mean distance, distance SD, mean velocity, mean velocity SD
inline void WriteData(const std::array<float,4>& simulation_data,size_t max_lenght, const unsigned int min_row_lenght, const unsigned int sub_lenght, const int cont, const size_t max_cicle_lenght){
    //if the longest boid name in the simulation is shorter than the minimum row lenght than max lenght became minimum row lenght
    max_lenght = {max_lenght < min_row_lenght ? min_row_lenght : max_lenght }; 
    //every row is aligned to left and (ecxept the last one) occupy a fixed space
    //the last one occupy the remaining space in order to align its end with the end of the name header
    //the computation space for the last row is:
    //max lenght minus space occupied by the three columns behind
    //space occupied by a column: sub_lenght + space at begin + space at end + column separation pipe -> sub_lenght+3
    //space occupied by the three column behind: 3*(space occupied by one column) -> 3*(sub_lenght+3)
    //space occupied by last column: max_lenght -(3*(sub_lenght+3))
    std::cout << ' ' << std::right << std::setw(sub_lenght) << simulation_data[0] << " |";
    std::cout << ' ' << std::right << std::setw(sub_lenght) << simulation_data[1] << " |";
    std::cout << ' ' << std::right << std::setw(sub_lenght) << simulation_data[2] << " |";
    std::cout << ' ' << std::right << std::setw((max_lenght -(3*(sub_lenght+3)))) << simulation_data[0] << " ||";
    
    //when the counter reach the max_cicle_lenght return carriage
    if (max_cicle_lenght == static_cast<size_t>(cont)){
        std::cout << '\n';
    }
}

/*  load a new boid from a file
    A valid boid file must have:
    - 5 rows
    - 1st row: boid name -- cannot be empty
    - 2nd row: vision angle -- must be positive
    - 3rd row: red quantity -- must be between 0 and 1 
    - 4th row: green quantity -- must be between 0 and 1 
    - 5th row: blue quantity -- must be between 0 and 1 */
inline BoidSpec::boid_specie LoadBoidFromFile(std::string filename){
    std::fstream fis;
    fis.open(filename);

    //if the filename is inexistent a NotFound error is thrown
    if (!fis){
        throw boidErr::fileNotFound{};
    }

    int n_row{0};
    BoidSpec::boid_specie internal_boid{}; //create a std boid -- will be overwritten from data in input
    std::array<float,3> color {}; //create a blank color array -- will be filled by data in input
    std::string word;
    while (std::getline(fis,word)) //read a line from file
    {
        //if the number of rows are more than 5 throw an error
        if (n_row>4){
            throw boidErr::invalidInput{};
        }
        switch (n_row)
        {
        //1st row -- boid name
        case 0:
            //if the name is empty throw an error
            if (word == ""){
                throw std::invalid_argument{"name can't be empty"};
            }
            internal_boid.SetName(word);//set the read name as boid name
            break;
        
        //2nd row -- vision angle
        case 1:
            //if vision angle is smaller than 0 throw an error
            //if content read is nan stoi will throw an invalid_argument exception
            if (std::stoi(word) < 0){
                throw std::invalid_argument{"angle can't be negative"};
            }
            internal_boid.SetVisionAngle(std::stoi(word)); //set the read vision angle as boid vision angle
            break;

        //row 3-4-5 -- rgb
        default:
            //if color value is smaller than 0 or bigger than 1 throw an error
            //if content read is nan stof will throw an invalid_argument exception
            if (std::stof(word) < 0 || std::stof(word) > 1){
                throw std::invalid_argument{"Color must be between 0 and 1"};
            }
            color[n_row-2] = std::stof(word); //set the read color as boid color
            //n_row-2 bc the switch enters the default in cont eq to 2-3-4 so -2 transform them in 0-1-2 to fill the array
            break;
        }
        ++n_row;
    }

    //if the number of rows is smaller than five (bigger case catched in while loop) throw an error
    if (n_row != 5){
        throw boidErr::invalidInput{};
    }

    internal_boid.SetColor(color);

    return internal_boid;

}

#endif