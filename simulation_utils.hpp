#ifndef SIMULATION_UTILS_HPP
#define SIMULATION_UTILS_HPP

#include<vector>
#include<cassert>

#include "boid_spec.hpp"
#include "utils.hpp"

//Hold the simulation data and keep them synced
struct SimulationData{
    private:
    std::vector<float> mean_distance_data_{0};
    std::vector<float> mean_velocity_data_{0};
    std::vector<float> mean_distance_sd_{0};
    std::vector<float> mean_velocity_sd_{0};

    
    public:
    
    void add (float mean_distance, float mean_velocity, float mean_distance_sd = 0, float mean_velocity_sd = 0){
        mean_distance_data_.push_back(mean_distance);
        mean_velocity_data_.push_back(mean_velocity);
        mean_distance_sd_.push_back(mean_distance_sd);
        mean_velocity_sd_.push_back(mean_velocity_sd);

        assert(mean_distance_data_.size() == mean_velocity_data_.size());
        assert(mean_distance_sd_.size() == mean_velocity_sd_.size());
        assert(mean_distance_data_.size() == mean_distance_sd_.size());
        assert(mean_velocity_data_.size() == mean_velocity_sd_.size());
    }

    void reset (){
        mean_distance_data_.clear();
        mean_velocity_data_.clear();
        mean_distance_sd_.clear();
        mean_velocity_sd_.clear();       

        assert(mean_distance_data_.size() == mean_velocity_data_.size());
        assert(mean_distance_sd_.size() == mean_velocity_sd_.size());
        assert(mean_distance_data_.size() == mean_distance_sd_.size());
        assert(mean_velocity_data_.size() == mean_velocity_sd_.size());
    }

    void erase_first (){
        mean_distance_data_.erase(mean_distance_data_.begin());
        mean_distance_sd_.erase(mean_distance_sd_.begin());
        mean_velocity_data_.erase(mean_velocity_data_.begin());
        mean_velocity_sd_.erase(mean_velocity_sd_.begin());

        assert(mean_distance_data_.size() == mean_velocity_data_.size());
        assert(mean_distance_sd_.size() == mean_velocity_sd_.size());
        assert(mean_distance_data_.size() == mean_distance_sd_.size());
        assert(mean_velocity_data_.size() == mean_velocity_sd_.size());
    }

    std::vector<float> getMeanDistance() const {return mean_distance_data_;}
    std::vector<float> getMeanVelocity() const {return mean_velocity_data_;}
    std::vector<float> getMeanDistanceSD() const {return mean_distance_sd_;}
    std::vector<float> getMeanVelocitySD() const {return mean_velocity_sd_;}
             
};

//Reset the simulation data and time
inline void ResetSimulationData(std::vector<SimulationData>& data, std::vector<float>& time){
    data.clear();
    time.clear();

    assert(time.empty());
}

//Sync the size of data vector with the number of the flocks currently in the simulation
inline void SyncDataVector(std::vector<SimulationData>& data, const std::vector<BoidSpec::boid_specie>& origin){
    for (size_t i = 0; i != origin.size(); ++i){
        data.push_back(SimulationData());
    }
    assert(data.size() == origin.size());
}

/*  Manage the delete of a Flock
    - Erase the selected flock from the species vector
    - Erase the selected flock from the coordinates vector
    - If it's deleted the last flock of the vector all the dropdown menus which pointed to that flock now points to the current last flock  
    - The vector cannot be empty, so if there is an attempt to delete the last element a flag is raised */
inline void delete_boid(std::vector<BoidSpec::boid_specie>& boid_species, int& item_current_idx, int& brush_current_idx, int& sim_current_idx, std::vector<std::vector<sf::Vector2f>>& coord_vector){

    if (boid_species.size() <= 1){
        throw boidErr::lastDeletedError{};
    } else {
        boid_species.erase(boid_species.begin()+item_current_idx);
        coord_vector.erase(coord_vector.begin()+item_current_idx);
        assert(boid_species.size()==coord_vector.size());
        if (item_current_idx >= static_cast<int>(boid_species.size())){
            item_current_idx = boid_species.size() -1;
        }
        if (brush_current_idx >= static_cast<int>(boid_species.size())){
            brush_current_idx = boid_species.size() -1;
        }
        if (sim_current_idx >= static_cast<int>(boid_species.size())){
            sim_current_idx = boid_species.size() -1;
        }

    }

    assert(boid_species.size() >= 1);
    assert(boid_species.size() == coord_vector.size());
    assert(static_cast<size_t>(item_current_idx) < boid_species.size());
    assert(static_cast<size_t>(brush_current_idx) < boid_species.size());
    assert(static_cast<size_t>(sim_current_idx) < boid_species.size());

}


#endif