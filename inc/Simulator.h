/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once

#include <map>
#include <memory>
#include <algorithm>
#include <array>
#include <vector>
#include <optional>

#include <Utils.h>
#include <gbp/GBPCore.h>
#include <Graphics.h>
#include <gbp/Variable.h>
#include <nanoflann.h>

#include <raylib.h>
#include <rlights.h>
#include <nanoflann.h>
#include <KDTreeMapOfVectorsAdaptor.h>
#include <random>

#include <json.hpp>

class Robot;
class Graphics;
class TreeOfRobots;


using nlohmann::json;


using vec2 = std::array<double, 2>;
using vec3 = std::array<double, 3>;

struct TrackedData {
   std::vector<std::array<double, 2>> positions;
   std::vector<std::array<double, 2>> velocities;
   double spawned_at;
   std::optional<double> finished_at;
   std::vector<std::array<double, 2>> waypoints;

   TrackedData() = default;

   TrackedData(std::array<double, 2> initial_position,
               std::array<double, 2> initial_velocity, 
               double spawned_at,
               std::size_t capacity = 1024)
       : spawned_at(spawned_at),
           // finished_at(5.0),
         finished_at(std::nullopt) 
         {
     this->positions.reserve(capacity);
     this->velocities.reserve(capacity);
     this->positions.push_back(initial_position);
     this->velocities.push_back(initial_velocity);
     // this->finished_at = std::nullopt;
   };
 };

 // Define the to_json function for TrackedData
 void to_json(json &j, const TrackedData &data);

/************************************************************************************/
// The main Simulator. This is where the magic happens.
/************************************************************************************/
class Simulator {
public:
    friend class Robot;
    friend class Factor;

    // Constructor
    Simulator();
    ~Simulator();

    // Pointer to Graphics class which hold all the camera, graphics and models for display
    Graphics* graphics;

    // kd-tree to store the positions of the robots at each timestep.
    // This is used for calculating the neighbours of robots blazingly fast.
    typedef KDTreeMapOfVectorsAdaptor<std::map<int,std::vector<double>>> KDTree;
    std::map<int, std::vector<double>> robot_positions_{{0,{0.,0.}}};
    KDTree* treeOfRobots_;

    // Image representing the obstacles in the environment
    Image obstacleImg;

    int next_rid_ = 0;                              // New robots will use this rid. It should be ++ incremented when this happens
    int next_vid_ = 0;                              // New variables will use this vid. It should be ++ incremented when this happens
    int next_fid_ = 0;                              // New factors will use this fid. It should be ++ incremented when this happens
    uint32_t clock_ = 0;                            // Simulation clock (timesteps)                   
    std::map<int, std::shared_ptr<Robot>> robots_;  // Map containing smart pointers to all robots, accessed by their rid.
    bool new_robots_needed_ = true;                 // Whether or not to create new robots. (Some formations are dynamicaly changing)
    bool symmetric_factors = false;                 // If true, when inter-robot factors need to be created between two robots,
                                                    // a pair of factors is created (one belonging to each robot). This becomes a redundancy.


    std::map<int, TrackedData> tracked_data_of_each_robot;
    /*******************************************************************************/
    // Create new robots if needed. Handles deletion of robots out of bounds. 
    // New formations must modify the vectors "robots to create" and optionally "robots_to_delete"
    // by appending (push_back()) a shared pointer to a Robot class.
    /*******************************************************************************/    
    void createOrDeleteRobots();

    /*******************************************************************************/
    // Set a proportion of robots to not perform inter-robot communications
    /*******************************************************************************/
    void setCommsFailure(float failure_rate=globals.COMMS_FAILURE_RATE);

    /*******************************************************************************/
    // Timestep loop of simulator.
    /*******************************************************************************/
    void timestep();

    /*******************************************************************************/
    // Drawing graphics.
    /*******************************************************************************/
    void draw();

    /*******************************************************************************/
    // Use a kd-tree to perform a radius search for neighbours of a robot within comms. range
    // (Updates the neighbours_ of a robot)
    /*******************************************************************************/    
    void calculateRobotNeighbours(std::map<int,std::shared_ptr<Robot>>& robots);

    /*******************************************************************************/
    // Handles keypresses and mouse input, and updates camera.
    /*******************************************************************************/
    void eventHandler();

    /*******************************************************************************/
    // Deletes the robot from the simulator's robots_, as well as any variable/factors associated.
    /*******************************************************************************/
    void deleteRobot(std::shared_ptr<Robot> robot);

    /***************************************************************************************************************/
    // RANDOM NUMBER GENERATOR.
    // Usage: random_number("normal", mean, sigma) or random_number("uniform", lower, upper)
    /***************************************************************************************************************/
    std::mt19937 gen_normal = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform = std::mt19937(globals.SEED);
    std::mt19937 gen_uniform_int = std::mt19937(globals.SEED);
    template<typename T>
    T random_number(std::string distribution, T param1, T param2){
        if (distribution=="normal") return std::normal_distribution<T>(param1, param2)(gen_normal);
        if (distribution=="uniform") return std::uniform_real_distribution<T>(param1, param2)(gen_uniform);
        return (T)0;
    }
    int random_int(int lower, int upper){
        return std::uniform_int_distribution<int>(lower, upper)(gen_uniform_int);
    }

};
