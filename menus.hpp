#ifndef MENUS_HPP
#define MENUS_HPP

#include<vector>
#include<array>
#include<functional>
#include<cassert>
#include<stdexcept>

#include <SFML/System/Vector2.hpp>
#include "imgui.h"
#include "libraries/implot/implot.h"
#include "libraries/implot/implot_internal.h"

#include "simulation_utils.hpp"
#include "boid_spec.hpp"

namespace Menus{

//Two spaces and a line in between
inline void LineSeparator(){
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
}

//Text with space below
inline void TextSpaced(const std::string& label){
    ImGui::Text("%s",label.data());
    ImGui::Spacing();
}

//Generate a drop down menu in order to display boid names of a boid_species vector
inline void MakeBoidDropDown(const std::string& label, const std::vector<BoidSpec::boid_specie>& boid_vector, int& item_current_idx, float width = 1){
    std::string combo_preview_value = boid_vector[item_current_idx].GetName();
    ImGui::SetNextItemWidth(ImGui::GetWindowWidth()*width);

    if (ImGui::BeginCombo(label.data(), combo_preview_value.data())){

        for(int n = 0; n < static_cast<int>(boid_vector.size()); ++n){ //Display all boids in vector as combo rows
            
            const bool is_selected = (item_current_idx == n);
            if (ImGui::Selectable(boid_vector[n].GetName().data(),is_selected)){ //Change combo preview if new element is selected
                item_current_idx = n;
            }
        }
        
        ImGui::EndCombo();
    }
}

//Draw a table in order to show the characteristics of a boid
inline void DrawBoidSpecsTable (const ImGuiTableFlags& flags, const BoidSpec::boid_specie& boid){
    if(ImGui::BeginTable("Specs Table",1, flags)){
        for (int row = 0; row != 3; ++row){
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            switch (row)
            {
            case 0:
                ImGui::Text("Name:");
                ImGui::SameLine();
                ImGui::Text("%s",boid.GetName().data());
                break;
            
            case 1:  
                ImGui::ColorButton("SpBoidSpec Color", ArrayToImVec4(boid.GetColor()));
                break;

            case 2: 
                ImGui::Text("Vision Angle:");
                ImGui::SameLine();
                ImGui::Text("%d",boid.GetVisionAngle());
                ImGui::SameLine();
                ImGui::Text("degrees");
                break;
            
            default:
                break;
            }
        }

        ImGui::EndTable();
    }
}

//Slider with the control of dimension
inline void ScalableSliderFloat(std::string label, float* p_indicator, float lower_limit, float upper_limit, float width = 1){
    ImGui::SetNextItemWidth(ImGui::GetWindowWidth()*width);
    ImGui::SliderFloat(label.data(),p_indicator,lower_limit,upper_limit);
}   

//Draw a plot which shows mean and SD from a simulation data for each flock passed
inline void Plot(const std::vector<SimulationData>& data, const std::vector<float>& time, const std::function<std::vector<float>(const SimulationData)>& GetMean, const std::function<std::vector<float>(const SimulationData)>& GetSD, const std::vector<BoidSpec::boid_specie>& boid_vector, const std::string& id, const std::string& x_name = "x", const std::string& y_name ="y"){
    //get the absolute maximum of the mean velocity of the flock
    float absolute_maximum{0};
    std::for_each(data.begin(),data.end(),[&](SimulationData specie_data){
        float local_maximum{0};
        for (size_t i = 0; i < GetMean(specie_data).size(); ++i)
        {
            if (GetMean(specie_data)[i]>local_maximum){
                local_maximum = GetMean(specie_data)[i];
            }
        }

        if (absolute_maximum < local_maximum){absolute_maximum = local_maximum;}
        
    }); 
    
    const unsigned int max_time_shift{10}; //display the last 10 values
    const int data_start{0}; //0 is the starting value from the data
    //the visible portion of the graph is between the last (biggest) value of time and max_time_shift values behind
    float upper_time_limit {time[time.size()-1]};  
    float lower_time_limit {upper_time_limit - max_time_shift};    
   
    ImPlot::CreateContext();
    if (ImPlot::BeginPlot(id.data())){

        ImPlot::SetupAxes(x_name.data(), y_name.data());
        ImPlot::SetupAxesLimits(lower_time_limit, upper_time_limit, data_start, absolute_maximum);

        for (size_t i = 0; i != data.size(); ++i){
            
            const std::string label {"##" + std::to_string(i)};
            ImPlot::PlotErrorBars(label.data(),time.data(), GetMean(data[i]).data(), GetSD(data[i]).data(),GetMean(data[i]).size());
            ImPlot::SetNextLineStyle(ArrayToImVec4(boid_vector[i].GetColor(),1));  //set the line color equal to the color of its flock
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle); //set the value indicator as a circle
            ImPlot::PlotLine(label.data(),  time.data(), GetMean(data[i]).data(), GetMean(data[i]).size());
        }

        ImPlot::EndPlot();
    }
    ImPlot::DestroyContext(); 
}

//Table to display data from a selected boid
inline void DataTable(const std::vector<float>& mean, const std::vector<float>& sd, const std::string& id, const int width = 1){
    ImGui::SetNextItemWidth(ImGui::GetWindowWidth()*width);
    if (ImGui::BeginListBox(id.data())){
        for (size_t i = 0; i != mean.size(); ++i){
            //last value first
            const std::string mean_str {std::to_string(mean[mean.size()-1-i])};
            const std::string sd_str {std::to_string(sd[sd.size()-1-i])};
            const std::string result {mean_str + " +- " + sd_str};
            ImGui::Text("%s",result.data());
        }
        ImGui::EndListBox();
        
    }
}
}
#endif