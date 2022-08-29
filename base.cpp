#include <SFML/Graphics.hpp>
#include<vector>
#include<cassert>
#include<cmath>
#include<iostream>
#include<iomanip>

#include "imgui.h"
#include "imgui-SFML.h"

#include "menus.hpp"
#include "boid_spec.hpp"
#include "utils.hpp"
#include "sfml_draw_utils.hpp"
#include "simulation_utils.hpp"
#include "io.hpp"
#include "flock_utils.hpp"

//manage window correct resize
void resize(sf::RenderWindow& window, sf::View& view, float height){
    float ratio{static_cast<float>(window.getSize().x)/static_cast<float>(window.getSize().y)}; //save the original dimensional ratio of the window
    view.setSize(height*ratio,height); //keep the new height, but reset the width in order to peserve the ratio
    return;
}

int main(){

/*     Options dummy{};
    dummy.distance = 100;
    dummy.separation_distance = 7;
    dummy.separation = 10;
    dummy.alignment = 0.01; 
    dummy.cohesion = 0.05; */

    //when a simulation is launched, the velocities of the boids are generated randomly following a normal distribution
    const float generatiom_velocity_mean{100.}; //mean of the velocity distribution
    const float generation_velocity_SD{100.}; //std dev of the velocity distribution

    //window original dimensions
    unsigned int const w_width{1920}; //window original width
    unsigned int const w_height{1080}; //window orginal height

    //loading background texture - if not present throw an error
    sf::Texture world_texture;
    sf::Sprite world_map;
    if (!world_texture.loadFromFile("world_map_texture.png")){
        throw std::runtime_error{"Texture not found -- Make sure to have a file named world_map_texture.png in the program folder"};
    }
    world_map.setTexture(world_texture);

    std::vector<BoidSpec::boid_specie> boid_spec_vector {}; //vector which hold the boid names
    std::vector<std::vector<sf::Vector2f>> boid_coord_vector {}; //vector which hold the boid cooordinates

    /* simulation options variables */

    SimulationParameters simulation_parameters{}; //keep track of the simulation parameters (alignment, cohesion, separation, vision distance)
    float sim_opt_lower_limit{0}; //minimum of the value that can have a sim opt
    float sim_opt_upper_limit{15}; //maximum of the value that can have a sim opt
    
    InputData input_data{KeyboardInput()}; //Take input from cin
    CheckInputCostraints(input_data, sim_opt_lower_limit, sim_opt_upper_limit); //Check if the input parameters are in the correct interval of values
    
    
    SimulationFlagHolder simulation_holder {}; //keep track of the simulation flags (Paused/Unpaused , Running/Modifiable)
    const int boid_radius{5};
    const double minimum_boid_separation{boid_radius+2}; //max distance within the separation is applied
    
    //Assign the values given in input to the local variables
    AssignInputValues(input_data, boid_spec_vector, boid_coord_vector, sf::Vector2f(world_map.getGlobalBounds().width, world_map.getGlobalBounds().height), simulation_holder,simulation_parameters,boid_radius);

    long unsigned int max_boid_name_lenght{0}; //store the lenght of the longest boid's name in the simulation
    const unsigned int min_row_length {57}; //in the console print is the minimum number of characters that a row must have to be correctly displayed
    const unsigned int subset_row_lenght {12}; //in the console print is the lenght of the statistics subcolumn (mean distance ...)

    //create a flock option by merging input parameters and minimum boid separation
    Options sim_opt {MakeOptions(simulation_parameters,minimum_boid_separation)};
    //create a multiflock from coord vector, names vector and options
    MultiFlock multiflock = GenerateMultiflock(boid_coord_vector,boid_spec_vector,sim_opt,generatiom_velocity_mean,generation_velocity_SD);
    
    //if there are flock inserted from cin start the simulation
    if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
        max_boid_name_lenght = MaxNameLength(boid_spec_vector);
        WriteHeaders(boid_spec_vector,max_boid_name_lenght, min_row_length, subset_row_lenght);
    }

    /* simulation data variables */
    
    std::vector<SimulationData> simulation_data {}; //hold the simulation data for each flock in order to draw a plot / display on a table
    std::vector<float> simulation_time {0}; //keep track of the time passed since the simulation has started and store in order to draw a plot

    SyncDataVector(simulation_data, boid_spec_vector); //sync the number of data stored with the flocks currently in the simulation
    
    const sf::Color brush_bg_color (255,255,0,100); //standard color of the brush is transparent yellow
    const sf::Color eraser_bg_color (255,255,255,100); //standard color of the eraser is transparent white
    const float boid_brush_border{0.1}; //in the brush this is the spacing between boids
    const int menu_width_ratio{6}; //ratio window witdth/menu width

    //SFML window setup
    sf::RenderWindow window(sf::VideoMode(w_width,w_height),"Boids");
    sf::View view{window.getDefaultView()};
    window.setFramerateLimit(60);

    //Imgui-SFML window setup
    ImGui::SFML::Init(window);
    sf::Clock deltaClock;

    //correct resize stuff
    sf::Vector2i pixelPos; //mouse position in screen
    sf::Vector2f worldPos; //mouse position in sfml window

    const float menu_witdh{static_cast<float>(window.getSize().x/(menu_width_ratio))}; //witdth of the ImGui menus

    /* indexes tracker initialization */

    int brush_opt_curr_idx{0}; //keep track of the boid selected in the brush drop down menu
    int boid_opt_curr_idx{0}; //keep track of the boid selected in the options drop down menu
    int sim_curr_idx{0}; //keep track of the boid selected in the simulation table drop down menu
   
    /* brush variables initialization */

    int brush_size{1};
    int brush_density{1};
    PaintMoveFlag toggle_brush_flag {PaintMoveFlag::Move}; //toggle/untoggle the brush. When brush disabled movement with mouse is enabled
    DrawEraseFlag toggle_draw_erase {DrawEraseFlag::Draw}; //toggle/untoggle the eraser. Eraser allows user to delete boid
    LastDeletedFlag last_deleted_flag {LastDeletedFlag::Unraised}; //if the last element of the boid species vector this flag is raised
    InvalidInputFlag invalid_input_flag {InvalidInputFlag::Valid}; //if an incorrected boid file is loaded this flag is set to invalid
    NotFoundFlag not_found_flag {NotFoundFlag::Found}; //if an unkown file is given as a boid file name this flag set to NotFound

    //In boid specs visualization draw the border of the table
    const ImGuiTableFlags table_border_flag = ImGuiTableFlags_Borders; 

    MovementDirectionHolder movement_holder; //hold 4 bool for every direction in order to enable movement with arrows

    //brush background is a circle. In that circle will be drown the preview of the boids in the brush
    sf::CircleShape brush_bg (boid_radius + (brush_size-1)*boid_radius + brush_size*boid_brush_border);
    brush_bg.setFillColor(brush_bg_color); 
    
    //boid background is the preview of the position of the boids in the brush
    sf::CircleShape boid_bg (boid_radius,3);
    boid_bg.setOrigin(boid_bg.getRadius(),boid_bg.getRadius()); //set the transformations application point to the centre of the circle

    //boid_to_stable_draw is the circle which will became an effective boid once drawn
    sf::CircleShape boid_to_stable_draw(boid_radius,3);
    boid_to_stable_draw.setOrigin(boid_to_stable_draw.getRadius(),boid_to_stable_draw.getRadius());

    bool is_drawing {false}; //if in drawing mode is true while the left mouse button is pressed, allow to effectevly draw boids on the map
    bool is_moving {false}; //if in moving mode is true while the left mouse button is pressed, allow to move the screen following the mouse
    sf::Vector2f mov_start{sf::Vector2f(0,0)}; //the origin from which the movement is calculated
    sf::Vector2f center_start{sf::Vector2f(0,0)}; //the point where the movement is applied

    unsigned int cicle_count{0}; //keep track of the cicles done since the simulation has started

    const unsigned int max_input_file_lenght{128}; //max lenght of the boid file name that can be uploaded with the editor uploader
    char boid_input_file_name[max_input_file_lenght]{}; //native array that keep track of the input text in the editor uploader -- its management is ImGui responsability

    auto const delta_t{sf::milliseconds(1)};
    int const fps = 60;
    int const steps_per_evolution{1000 / fps};
    double const dt{delta_t.asSeconds()};


    while (window.isOpen())
    {
        //correct resize stuff
        pixelPos = sf::Mouse::getPosition(window);
        worldPos = window.mapPixelToCoords(pixelPos);
        
        sf::Event event;

        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(window, event); //Imgui-SFML process events

            switch (event.type) //SFML process events
            {
                case sf::Event::Closed:
                    window.close();
                    break;
                
                case sf::Event::Resized:
                    resize(window,view,w_height); //correct resizing of the view
                    break;
                
                case sf::Event::KeyPressed:
                    switch (event.key.code)
                    {
                    case sf::Keyboard::Right:
                            movement_holder.move_right = true;
                        break;
                    case sf::Keyboard::Left:
                            movement_holder.move_left = true;
                        break;                    
                    case sf::Keyboard::Up:
                            movement_holder.move_up = true;
                        break;                    
                    case sf::Keyboard::Down:
                            movement_holder.move_down = true;
                        break;    

                    default:
                        break;
                    }
                    break;
                
                case sf::Event::KeyReleased:
                    switch (event.key.code)
                    {
                    case sf::Keyboard::Right:
                            movement_holder.move_right = false;
                        break;
                    case sf::Keyboard::Left:
                            movement_holder.move_left = false;
                        break;                    
                    case sf::Keyboard::Up:
                            movement_holder.move_up = false;
                        break;                    
                    case sf::Keyboard::Down:
                            movement_holder.move_down = false;
                        break;                    
                    default:
                        break;
                    }
                    break;

                case sf::Event::MouseButtonPressed:
                    if (event.mouseButton.button == sf::Mouse::Left){
                        if (toggle_brush_flag == PaintMoveFlag::Draw){
                            if(!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) && world_map.getGlobalBounds().contains(worldPos)){
                                //if in drawing mode check if the mouse is on an ImGui menu or outside the world borders, if not draw the boids 
                                is_drawing = true;
                            }
                        } else {
                            if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)){
                                //if in moving mode check if the mouse is on an Imgui menu, if not enable mouse movement on left click
                                mov_start = worldPos; //the origin from which the movement is calculated is the position of the mouse when clicked
                                center_start = view.getCenter(); //the point where the movement is applied is the position of the center of the view when the mouse is clicked
                                is_moving = true;
                            }
                        }
                    }
                    break;
                
                case sf::Event::MouseButtonReleased:
                    if (event.mouseButton.button == sf::Mouse::Left){
                        if (toggle_brush_flag == PaintMoveFlag::Draw){
                            //on drawing mode stop drawing when left button of the mouse is released
                            is_drawing = false;
                        } else {
                            //on moving mode stop moving when left button of the mouse is released
                            is_moving = false;
                        }
                    }
                    break;
                
                case sf::Event::MouseMoved:
                    if (toggle_brush_flag == PaintMoveFlag::Move){
                        if (is_moving){
                            //in moving mode move the view when mouse is moved
                            const sf::Vector2f new_pos {center_start + (worldPos - mov_start)}; //new position of the center: old center pos + mouse displacement
                            if (world_map.getGlobalBounds().contains(new_pos)){
                                view.setCenter(new_pos); //if the new position is in the bg move the center of the view to the new pos
                            }    
                        }
                    }
                    
                    break;

                default:
                    break;
            }
        }

        ImGui::SFML::Update(window, deltaClock.restart());
        
        //update brush
        assert(brush_size>=1);
        brush_bg.setRadius(boid_radius + 2*(brush_size-1)*boid_radius + brush_size*boid_brush_border); //sync brush size with selected brush size from slider
        brush_bg.setOrigin(brush_bg.getRadius(),brush_bg.getRadius()); //set the origin of the brush to centre in order to apply all the transformations to this point
        brush_bg.setPosition(worldPos.x,worldPos.y); //move brush to mouse

        //this function allow the movement with the arrows
        move_view_with_arrows(movement_holder.move_right, movement_holder.move_left, movement_holder.move_down, movement_holder.move_up, view, 5, world_map.getGlobalBounds());

        /* -------------------------------------------------------------------------- */
        /* -------------- LEFT MENU -- Boid/Brush options --------------------------- */ 
        /* -------------------------------------------------------------------------- */


        ImGui::SetNextWindowSize(ImVec2(menu_witdh,window.getSize().y)); //set left menu dimensions
        ImGui::SetNextWindowPos(ImVec2(0,0)); //set left menu origin to the top left corner
        ImGui::Begin("Boid Options");
            //Disable the modifyng menus if the simulation has started
            if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
                ImGui::BeginDisabled();
            }

            Menus::TextSpaced("Brush Options");

            ImGui::Text("Boid Type");
            Menus::MakeBoidDropDown("##BrushBoidSelector",boid_spec_vector,brush_opt_curr_idx, 0.4); //allow the selection of the boid on the brush

            ImGui::SameLine();

            //Button which allow the selection between brush and eraser
            std::string brush_eraser_button_name {toggle_draw_erase == DrawEraseFlag::Draw ? "Erase" : "Brush"};
            if (ImGui::Button(brush_eraser_button_name.data())){
                toggle_draw_erase = (toggle_draw_erase == DrawEraseFlag::Draw ? DrawEraseFlag::Erase : DrawEraseFlag::Draw);
            }

            ImGui::SameLine();

            //Button which allow the selection between drawing mode and moving mode
            std::string move_draw_button_name {toggle_brush_flag == PaintMoveFlag::Draw ? "Draw" : "Move"};
            if (ImGui::Button(move_draw_button_name.data())){
                toggle_brush_flag = (toggle_brush_flag == PaintMoveFlag::Draw ? PaintMoveFlag::Move : PaintMoveFlag::Draw);
            }

            ImGui::DragInt("Size", &brush_size, 0.5f, 1, __INT_MAX__); //Manage Brush/Eraser dimension
            if (brush_size < 1){brush_size = 1;} //if keybord mess up something fix
            assert(brush_size >= 1);

            ImGui::SliderInt("Density", &brush_density, 1, 100); //Manage Brush density
            if (brush_density < 1){brush_density = 1;}
            if (brush_density > 100){brush_density = 100;}
            assert(brush_density >= 1 && brush_density <= 100);
            
            Menus::LineSeparator();

            Menus::TextSpaced("Boid Options");

            ImGui::Text("Boid Type");
            Menus::MakeBoidDropDown("##OptionsBoidSelector",boid_spec_vector,boid_opt_curr_idx, 0.4); //allow the selection of the boid on the options
            ImGui::SameLine();

            if (ImGui::Button("Delete")){
                //if a boid is deleted remove it from the specie vector and the coord vector.
                //Update all the drop down menus if they are displaying a no more exisiting boid
                try
                {
                    delete_boid(boid_spec_vector,boid_opt_curr_idx,brush_opt_curr_idx,sim_curr_idx,boid_coord_vector);
                }
                catch(const boidErr::lastDeletedError& e)
                {
                    last_deleted_flag = LastDeletedFlag::Raised;
                }
                
            }

            if (last_deleted_flag == LastDeletedFlag::Raised){
                //If there is an attempt to delete the last species left in the vector this message is shown and the delete is stopped
                ImGui::TextColored(ImVec4(1.0,0,0,1), "The vector cannot be empty");
            }

            Menus::DrawBoidSpecsTable(table_border_flag, boid_spec_vector[boid_opt_curr_idx]); //Draw a table with the caracteristics of a boid

            Menus::LineSeparator();
            ImGui::Text("New boid file name");
            if (invalid_input_flag == InvalidInputFlag::Invalid){
                //If there is an attempt to load an invalid boid file this message is shown
                ImGui::TextColored(ImVec4(1.0,0,0,1), "Invalid file");
            }
            if (not_found_flag == NotFoundFlag::NotFound){
                //If there is an attempt to load an unknown boid file this message is shown
                ImGui::TextColored(ImVec4(1.0,0,0,1), "File not found");
            }

            //boid file input text -- here you can upload a boid file by writing the file name in the text input box
            ImGui::InputText("##BoidInput",boid_input_file_name,max_input_file_lenght); 
            
            const std::string str_boid_file_name {boid_input_file_name}; //save the char array as a string

            if (ImGui::Button("Upload")){
                
                try
                {
                    BoidSpec::boid_specie new_boid{LoadBoidFromFile(str_boid_file_name)}; //load new boid from file
                    boid_spec_vector.push_back(new_boid); // insert the new boid to the boid specs vector
                    boid_coord_vector.push_back({}); //insert a empty coord vector to the total coord vector -- this would be the coord vector for the flock just added
                    boid_opt_curr_idx = boid_spec_vector.size()-1; //set the option drop down focus to the boid just added
                    assert(boid_spec_vector.size() == boid_coord_vector.size());                 
                }
                catch(const boidErr::fileNotFound& e)
                {
                    //If the filename doesn't exist raise a NotFound flag
                    not_found_flag = NotFoundFlag::NotFound;
                }
                catch(const boidErr::invalidInput& e){
                    //If the boid file doesn't meet the requirements raise an Invalid flag
                    invalid_input_flag = InvalidInputFlag::Invalid;
                }
                catch(const std::invalid_argument& e){
                    //If some string-to_number conversion goes wrong raise an Invalid flag
                    invalid_input_flag = InvalidInputFlag::Invalid;
                }
                
            }

            if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
                ImGui::EndDisabled();
            }
            Menus::LineSeparator();

            ImGui::Text("Zoom");
            ImGui::SameLine();
            if (ImGui::Button("+")){
                view.zoom(0.3f);
            }
            ImGui::SameLine();
            if(ImGui::Button("-")){
                view.zoom(1.1f);
            }

        ImGui::End();


        /* --------------------------------------------------------------------------- */
        /* -------------- RIGHT MENU -- Simulation options --------------------------- */ 
        /* --------------------------------------------------------------------------- */

        ImGui::SetNextWindowSize(ImVec2(menu_witdh,window.getSize().y)); //Set right menu dimensions
        ImGui::SetNextWindowPos(ImVec2(window.getSize().x-menu_witdh,0)); //Set the origin of the right menu on the top right corner of the window minus the menu width
        ImGui::Begin("Simulation Options");

            if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
                ImGui::BeginDisabled();
            }

            Menus::TextSpaced("Simulation parameters");

            //Vision distance selector
            ImGui::SetNextItemWidth(ImGui::GetWindowWidth()*0.4);
            ImGui::DragInt("vision distance", &simulation_parameters.vision_distance, 0.5f, sim_opt_lower_limit, __INT_MAX__);
            if (simulation_parameters.vision_distance < sim_opt_lower_limit){simulation_parameters.vision_distance = sim_opt_lower_limit;} //if keybord mess up something fix
            assert(simulation_parameters.vision_distance >= sim_opt_lower_limit);

            //separation selector slider
            Menus::ScalableSliderFloat("separation", &simulation_parameters.separation,sim_opt_lower_limit,sim_opt_upper_limit,0.4);
            if (simulation_parameters.separation < sim_opt_lower_limit){simulation_parameters.separation = sim_opt_lower_limit;}
            if (simulation_parameters.separation > sim_opt_upper_limit){simulation_parameters.separation = sim_opt_upper_limit;}
            assert(simulation_parameters.separation >= sim_opt_lower_limit && simulation_parameters.separation <= sim_opt_upper_limit);

            //alignment selector slider
            Menus::ScalableSliderFloat("alignment",&simulation_parameters.alignment,sim_opt_lower_limit,sim_opt_upper_limit,0.4);
            if (simulation_parameters.alignment < sim_opt_lower_limit){simulation_parameters.alignment = sim_opt_lower_limit;}
            if (simulation_parameters.alignment > sim_opt_upper_limit){simulation_parameters.alignment = sim_opt_upper_limit;}
            assert(simulation_parameters.alignment >= sim_opt_lower_limit && simulation_parameters.alignment <= sim_opt_upper_limit);

            //cohesion selector slider
            Menus::ScalableSliderFloat("cohesion",&simulation_parameters.cohesion,sim_opt_lower_limit,sim_opt_upper_limit,0.4);
            if (simulation_parameters.cohesion < sim_opt_lower_limit){simulation_parameters.cohesion = sim_opt_lower_limit;}
            if (simulation_parameters.cohesion > sim_opt_upper_limit){simulation_parameters.cohesion = sim_opt_upper_limit;}
            assert(simulation_parameters.cohesion >= sim_opt_lower_limit && simulation_parameters.cohesion <= sim_opt_upper_limit);

            if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
            ImGui::EndDisabled();
            }

            Menus::LineSeparator();

            //Button which start the simulation and manage all the things needed in order to starts
            if(ImGui::Button("Play")){
                if (simulation_holder.flag == SimulationFlagHolder::Flag::Modifiable){
                    toggle_brush_flag = PaintMoveFlag::Move; //disable drawing
                    ResetSimulationData(simulation_data, simulation_time); //reset data and time
                    SyncDataVector(simulation_data, boid_spec_vector); //sync the number of data tracked with the number of flocks in the simulation
                    simulation_time.push_back(0); //set the simulation time as 0
                    cicle_count = 0; //reset the cicle counter
                    max_boid_name_lenght = MaxNameLength(boid_spec_vector); //update the value of the longest boid name in the simulation
                    WriteHeaders(boid_spec_vector,max_boid_name_lenght,min_row_length,subset_row_lenght); //write table header in std output
                    sim_opt = MakeOptions(simulation_parameters,minimum_boid_separation); //update flock options
                    multiflock = GenerateMultiflock(boid_coord_vector,boid_spec_vector,sim_opt,generatiom_velocity_mean,generation_velocity_SD); //update multiflock
                    simulation_holder.flag = SimulationFlagHolder::Flag::Running; //set the simulation as running
                }
            }

            ImGui::SameLine();

            //Button which stop the simulation
            if(ImGui::Button("Stop")){
                if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
                    simulation_holder.flag = SimulationFlagHolder::Flag::Modifiable; //reset the status as modifiable
                    simulation_holder.paused_flag = SimulationFlagHolder::PausedFlag::Unpaused; //if the simulation was in pause reset the pause flag
                }
            }

            ImGui::SameLine();

            //Button which pause/resume the simulation -- if the simulation is paused the modifying menus aren available, but the simulation doesen't go forward
            //Button doesn't work if the simulation is not started
            std::string pause_resume_button_name {simulation_holder.paused_flag == SimulationFlagHolder::PausedFlag::Paused ? "Resume" : "Pause"};
            if(ImGui::Button(pause_resume_button_name.data())){
                if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
                    simulation_holder.paused_flag = (simulation_holder.paused_flag == SimulationFlagHolder::PausedFlag::Paused ? SimulationFlagHolder::PausedFlag::Unpaused : SimulationFlagHolder::PausedFlag::Paused);
                }
            }

            Menus::LineSeparator();
            
            Menus::TextSpaced("Simulation data");

            /* --------------------------------------------------------------------------- */
            /* ------------------- RIGHT MENU -- Simulation data ------------------------- */ 
            /* --------------------------------------------------------------------------- */

            //tab selector which allows to visualize data from the simulation 

            assert(simulation_data.size() != 0);
            assert(simulation_time.size() != 0);
            assert(simulation_time.size() == simulation_data[0].getMeanDistance().size());

            if (ImGui::BeginTabBar("##MacroSelector")){
                if (ImGui::BeginTabItem("MeanDistance")){  //Select Mean Distance data
                    if (ImGui::BeginTabBar("##MeanDistanceMicroSelector")){ //tab selector which allow to switch between a plot or a table view

                        if (ImGui::BeginTabItem("MeanDistPlot")){ //Select plot
                                //draw the plot
                                Menus::Plot(simulation_data, simulation_time, [](const SimulationData& sim){return sim.getMeanDistance();}, [](const SimulationData& sim){return sim.getMeanDistanceSD();}, boid_spec_vector, "##MeanDist", "time", "Mean distance");
                            ImGui::EndTabItem();
                        }

                        if (ImGui::BeginTabItem("MeanDistTable")){ //Select Table
                                //Show data from selected boid
                                Menus::DataTable(simulation_data[sim_curr_idx].getMeanDistance(),simulation_data[sim_curr_idx].getMeanDistanceSD(),"##MeanDistTab" );
                                //Drop down menu which allow to select the boid in order to visualize its data
                                Menus::MakeBoidDropDown("Boid Type", boid_spec_vector, sim_curr_idx, 0.4);
                            ImGui::EndTabItem();
                        }
                        ImGui::EndTabBar();
                    }
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("MeanVelocity")){ //Select Mean velocity data
                    if (ImGui::BeginTabBar("##MeanVelocityMicroSelector")){ //tab selector which allow to switch between a plot or a table view
                        
                        if (ImGui::BeginTabItem("MeanVelPlot")){ //Select Plot
                                //Draw the plot
                                Menus::Plot(simulation_data, simulation_time, [](const SimulationData& sim){return sim.getMeanVelocity();}, [](const SimulationData& sim){return sim.getMeanVelocitySD();}, boid_spec_vector, "##MeanVel", "time", "Mean velocity");
                            ImGui::EndTabItem();
                        }

                        if (ImGui::BeginTabItem("MeanVelTable")){
                                //Show data from selected boid
                                Menus::DataTable(simulation_data[boid_opt_curr_idx].getMeanVelocity(),simulation_data[boid_opt_curr_idx].getMeanVelocitySD(),"##MeanVelTab" );
                                //Drop down menu which allow to select the boid in order to visualize its data
                                Menus::MakeBoidDropDown("Boid Type", boid_spec_vector, sim_curr_idx, 0.4);
                            ImGui::EndTabItem();
                        }
                        ImGui::EndTabBar();
                    }
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }

            Menus::LineSeparator();

            if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
                ImGui::BeginDisabled();
            }
            if (simulation_holder.flag == SimulationFlagHolder::Flag::Running){
                ImGui::EndDisabled();
            }

        ImGui::End();

        /* --------------------------------------------------------------------------- */
        /* --------------------------- SIMULATION CICLE ------------------------------ */ 
        /* --------------------------------------------------------------------------- */

        //if the simulation is started and is not in pause enter the simulation cicle
        if (simulation_holder.flag == SimulationFlagHolder::Flag::Running && simulation_holder.paused_flag == SimulationFlagHolder::PausedFlag::Unpaused){

            //for each program cicle the simulation goes forward for a number of steps equal to steps_per_evolution
            for (int i = 0; i != steps_per_evolution; ++i)
            {
                //evolve the multiflock applying the ambient modification of bouncing on borders
                const double max_speed{200.};
                const double min_speed{20.};
                multiflock.evolve(dt,max_speed,min_speed,[&](Flock& flock){ApplyAmbient(flock,world_map.getGlobalBounds().width,world_map.getGlobalBounds().height);});
            }

            //get simulation data of this cicle
            const auto mean_distance_and_RMS{multiflock.get_all_distance_mean_RMS()};
            const auto mean_velocity_and_RMS{multiflock.get_all_speed_mean_RMS()};

            //for each flock
            for (size_t i = 0; i != simulation_data.size(); ++i)
            {
                //add data to the simulation data
                const std::array<float,4> cicle_data{(float)std::sqrt(mean_distance_and_RMS[i].x()),(float)std::sqrt(mean_distance_and_RMS[i].y()),(float)std::sqrt(mean_velocity_and_RMS[i].x()),(float)std::sqrt(mean_velocity_and_RMS[i].y())};
                simulation_data[i].add(cicle_data[0],cicle_data[2],cicle_data[1],cicle_data[3]);
                //Write data in std output
                WriteData(cicle_data,max_boid_name_lenght,min_row_length, subset_row_lenght, i, simulation_data.size()-1);

                //in order to avoid problems with the ImGui drawing buffer the size of the data vector is kept costant 
                if (cicle_count > 50) {
                    simulation_data[i].erase_first();
                }
            }         
            simulation_time.push_back(cicle_count); //update time

            if (cicle_count > 50){
                simulation_time.erase(simulation_time.begin()); //in order to keep synced the dimension of the data vector with the dimension of the time vector also the time vector dimension is kept constant
            }
            ++cicle_count;

            std::for_each(simulation_data.begin(), simulation_data.end(), [&](SimulationData data){
                assert(data.getMeanDistance().size() == simulation_time.size());
                assert(data.getMeanDistanceSD().size() == simulation_time.size());
                assert(data.getMeanVelocity().size() == simulation_time.size());
                assert(data.getMeanVelocitySD().size() == simulation_time.size());
            });
        }

        /* --------------------------------------------------------------------------- */
        /* ------------------------------ DRAWING CICLE ------------------------------ */ 
        /* --------------------------------------------------------------------------- */

        window.clear();
        window.setView(view);
        window.draw(world_map);

        if (toggle_brush_flag == PaintMoveFlag::Draw){
            
            if (toggle_draw_erase == DrawEraseFlag::Draw){
                boid_bg.setFillColor(ArrayToColor(boid_spec_vector[brush_opt_curr_idx].GetColorInt())); //set the boid preview color to the selected boid's color
                brush_bg.setFillColor(brush_bg_color); //set the color of the brush to its std
                window.draw(brush_bg); //draw the brush

                auto boid_preview_vector {make_boid_preview(boid_bg, worldPos, boid_brush_border, brush_density, brush_size)}; //vector which contains the position of the preview of the boids in the brush
                std::for_each(boid_preview_vector.begin(), boid_preview_vector.end(), [&](sf::Vector2f boid_preview_coord){
                    boid_bg.setPosition(boid_preview_coord); //set the position of the circle shape which represent the boid to the position set in the vector
                    window.draw(boid_bg); //draw the previewed boid

                    if (is_drawing){
                        //check if the boid which the user wants to draw is over another boid drawn
                        //In this case don't add the boid to the drawing vector
                        if (check_if_isolated(boid_coord_vector,boid_preview_coord,boid_radius) && (world_map.getGlobalBounds().contains(boid_preview_coord))){
                            boid_coord_vector[brush_opt_curr_idx].push_back(boid_bg.getPosition());
                        }
                    } 
                });
            } else {
                brush_bg.setFillColor(eraser_bg_color); //set the color of the brush to the eraser color
                window.draw(brush_bg); //draw the brush

                if (is_drawing){
                    //For each boid in the drawing vector is checked if the distance between the center of the eraser (aka the mouse coord)
                    //and the center of the boid is less than the eraser radius + boid radius
                    //In this case this boid is erased from the drawing vector
                    for (size_t  flock = 0;  flock< boid_coord_vector.size(); ++flock){
                        for (size_t boid = 0; boid < boid_coord_vector[flock].size(); ++boid)
                        {
                            if(getDistance(boid_coord_vector[flock][boid],worldPos)<(brush_bg.getRadius()+boid_radius)){
                                boid_coord_vector[flock].erase(boid_coord_vector[flock].begin()+boid);
                            }        
                        }
                    }
                    
                }
            }
        }

        if (simulation_holder.flag == SimulationFlagHolder::Flag::Modifiable){  
            draw_boids(boid_coord_vector,boid_spec_vector,boid_to_stable_draw,window);
        } else {
            std::vector<std::vector<sf::Vector2f>> sim_boid_coord_vector {GenerateCoordVector(multiflock)};     
            draw_boids(sim_boid_coord_vector,boid_spec_vector,boid_to_stable_draw,window);     
        }
        
        ImGui::SFML::Render(window);
        window.display();
        
    }
    ImGui::SFML::Shutdown();    
}