/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Graphics.h>
#include <Robot.h>
#include <Simulator.h>
#include <gbp/GBPCore.h>
#include <iostream>
#include <nanoflann.h>

void to_json(json &j, const TrackedData &data) {
  j = json{
      {"positions", data.positions},
      {"velocities", data.velocities},
      {"started_at", data.spawned_at},
      {"finished_at", *data.finished_at},
      {"mission", json{{"started_at", data.spawned_at},
                       {"finished_at", *data.finished_at},
                       {"waypoints", data.waypoints}}},
  };
}

/*******************************************************************************/
// Raylib setup
/*******************************************************************************/
Simulator::Simulator() {
  SetTraceLogLevel(LOG_ERROR);
  if (globals.DISPLAY) {
    SetTargetFPS(60);
    InitWindow(globals.SCREEN_SZ, globals.SCREEN_SZ, globals.WINDOW_TITLE);
  }

  // Initialise kdtree for storing robot positions (needed for nearest neighbour
  // check)
  treeOfRobots_ = new KDTree(2, robot_positions_, 50);

  // For display only
  // User inputs an obstacle image where the obstacles are BLACK and background
  // is WHITE.
  obstacleImg = LoadImage(globals.OBSTACLE_FILE.c_str());
  if (obstacleImg.width == 0)
    obstacleImg = GenImageColor(globals.WORLD_SZ, globals.WORLD_SZ, WHITE);

  // However for calculation purposes the image needs to be inverted.
  ImageColorInvert(&obstacleImg);
  graphics = new Graphics(obstacleImg);
};

/*******************************************************************************/
// Destructor
/*******************************************************************************/
Simulator::~Simulator() {
  delete treeOfRobots_;
  int n = robots_.size();
  for (int i = 0; i < n; ++i)
    robots_.erase(i);
  if (globals.DISPLAY) {
    delete graphics;
    CloseWindow();
  }

  // TODO: log metrics in a json structure
  // { "makespan": <float>,
  //  "robots": {"robot_id": {"positions": [], "velocities": []}, ...},
  // }
  std::ofstream metrics_file;
  metrics_file.open("metrics.json");
  json json;
  /*json["makespan"] = clock_;*/
  json["clock-ticks"] = this->clock_;
  json["robots"] = {};
  for (auto [rid, tracked_data] : this->tracked_data_of_each_robot) {
    // json["robots"][rid]["positions"] = tracked_data.positions;
    json["robots"][std::to_string(rid)] = tracked_data;
    // json["robots"][rid]["velocities"] = tracked_data.velocities;
  }
  metrics_file << json;
  metrics_file.close();
};

/*******************************************************************************/
// Drawing graphics.
/*******************************************************************************/
void Simulator::draw() {
  if (!globals.DISPLAY)
    return;

  BeginDrawing();
  ClearBackground(RAYWHITE);
  BeginMode3D(graphics->camera3d);
  // Draw Ground
  DrawModel(graphics->groundModel_, graphics->groundModelpos_, 1., WHITE);
  // Draw Robots
  for (auto [rid, robot] : robots_)
    robot->draw();
  EndMode3D();
  draw_info(clock_);
  EndDrawing();
};

/*******************************************************************************/
// Timestep loop of simulator.
/*******************************************************************************/
void Simulator::timestep() {

  if (globals.SIM_MODE != Timestep)
    return;

  // Create and/or destory factors depending on a robot's neighbours
  calculateRobotNeighbours(robots_);
  for (auto [r_id, robot] : robots_) {
    robot->updateInterrobotFactors();
  }

  // If the communications failure rate is non-zero, activate/deactivate robot
  // comms
  setCommsFailure(globals.COMMS_FAILURE_RATE);

  // Perform iterations of GBP. Ideally the internal and external iterations
  // should be interleaved better. Here it is assumed there are an equal number.
  for (int i = 0; i < globals.NUM_ITERS; i++) {
    iterateGBP(1, INTERNAL, robots_);
    iterateGBP(1, EXTERNAL, robots_);
  }

  std::size_t finished = 0;
  // Update the robot current and horizon states by one timestep
  for (auto [r_id, robot] : robots_) {
    robot->updateHorizon();
    robot->updateCurrent();

    auto &tracked_data = this->tracked_data_of_each_robot[r_id];
    std::array<double, 2> position = {robot->position_(0), robot->position_(1)};
    const auto &last_position = tracked_data.positions.back();
    const auto dt = globals.TIMESTEP;
    std::array<double, 2> velocity = {(position[0] - last_position[0]) / dt,
                                      (position[1] - last_position[1]) / dt};
    tracked_data.positions.push_back(position);
    tracked_data.velocities.push_back(velocity);

    // if (robot->waypoints_.empty()) {
    // if (robot->waypoints_.size() == 0  && this->clock_ > 200) {
    auto distance =
        (robot->waypoints_.front()({0, 1}) - robot->position_({0, 1})).norm();
    bool reached_final_waypoint = distance < robot->robot_radius_;

    if (reached_final_waypoint) {
      if (!tracked_data.finished_at.has_value()) {
        tracked_data.finished_at = this->clock_ * globals.TIMESTEP;
      }
      finished++;
    }
  }

  // std::cout << "#robots: " << robots_.size() << " #finished: " << finished <<
  // '\n';
  if (robots_.size() > 0 && finished == robots_.size()) {
    // stop sim if all robots have finished.
    // only makes sense in scenarios where all robots spawn at once.
    globals.RUN = false;
  } else {
    // std::cout << "#robots: " << robots_.size() << " #finished: " << finished
    // << '\n';
  }

  // Increase simulation clock by one timestep
  clock_++;
  if (clock_ >= globals.MAX_TIME)
    globals.RUN = false;
};

/*******************************************************************************/
// Use a kd-tree to perform a radius search for neighbours of a robot within
// comms. range (Updates the neighbours_ of a robot)
/*******************************************************************************/
void Simulator::calculateRobotNeighbours(
    std::map<int, std::shared_ptr<Robot>> &robots) {
  for (auto [rid, robot] : robots) {
    robot_positions_.at(rid) =
        std::vector<double>{robot->position_(0), robot->position_(1)};
  }
  treeOfRobots_->index->buildIndex();

  for (auto [rid, robot] : robots) {
    // Find nearest neighbors in radius
    robot->neighbours_.clear();
    std::vector<double> query_pt = std::vector<double>{
        robots[rid]->position_(0), robots[rid]->position_(1)};
    const float search_radius = pow(globals.COMMUNICATION_RADIUS, 2.);
    std::vector<nanoflann::ResultItem<size_t, double>> matches;
    nanoflann::SearchParameters params;
    params.sorted = true;
    const size_t nMatches = treeOfRobots_->index->radiusSearch(
        &query_pt[0], search_radius, matches, params);
    for (size_t i = 0; i < nMatches; i++) {
      auto it = robots_.begin();
      std::advance(it, matches[i].first);
      if (it->first == rid)
        continue;
      robot->neighbours_.push_back(it->first);
    }
  }
};

/*******************************************************************************/
// Set a proportion of robots to not perform inter-robot communications
/*******************************************************************************/
void Simulator::setCommsFailure(float failure_rate) {
  if (failure_rate == 0)
    return;
  // Get all the robot ids and then shuffle them
  std::vector<int> range{};
  for (auto &[rid, robot] : robots_)
    range.push_back(rid);
  std::shuffle(range.begin(), range.end(), gen_uniform);
  // Set a proportion of the robots as inactive using their
  // interrobot_comms_active_ flag.
  int num_inactive = round(failure_rate * robots_.size());
  for (int i = 0; i < range.size(); i++) {
    robots_.at(range[i])->interrobot_comms_active_ = (i >= num_inactive);
  }
}

/*******************************************************************************/
// Handles keypresses and mouse input, and updates camera.
/*******************************************************************************/
void Simulator::eventHandler() {
  // Deal with Keyboard key press
  int key = GetKeyPressed();
  switch (key) {
  case KEY_ESCAPE:
    globals.RUN = false;
    break;
  case KEY_H:
    globals.LAST_SIM_MODE =
        (globals.SIM_MODE == Help) ? globals.LAST_SIM_MODE : globals.SIM_MODE;
    globals.SIM_MODE =
        (globals.SIM_MODE == Help) ? globals.LAST_SIM_MODE : Help;
    break;
  case KEY_SPACE:
    graphics->camera_transition_ = !graphics->camera_transition_;
    break;
  case KEY_P:
    globals.DRAW_PATH = !globals.DRAW_PATH;
    break;
  case KEY_R:
    globals.DRAW_INTERROBOT = !globals.DRAW_INTERROBOT;
    break;
  case KEY_W:
    globals.DRAW_WAYPOINTS = !globals.DRAW_WAYPOINTS;
    break;
  case KEY_ENTER:
    globals.SIM_MODE = (globals.SIM_MODE == Timestep) ? SimNone : Timestep;
    break;
  default:
    break;
  }

  // Mouse input handling
  Ray ray = GetMouseRay(GetMousePosition(), graphics->camera3d);
  Vector3 mouse_gnd =
      Vector3Add(ray.position, Vector3Scale(ray.direction,
                                            -ray.position.y / ray.direction.y));
  Vector2 mouse_pos{mouse_gnd.x, mouse_gnd.z}; // Position on the ground plane
  // Do stuff with mouse here using mouse_pos .eg:
  // if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)){
  //     do_code
  // }

  // Update the graphics if the camera has moved
  graphics->update_camera();
}

/*******************************************************************************/
// Create new robots if needed. Handles deletion of robots out of bounds.
// New formations must modify the vectors "robots to create" and optionally
// "robots_to_delete" by appending (push_back()) a shared pointer to a Robot
// class.
/*******************************************************************************/
void Simulator::createOrDeleteRobots() {
  if (!new_robots_needed_)
    return;

  std::vector<std::shared_ptr<Robot>> robots_to_create{};
  std::vector<std::shared_ptr<Robot>> robots_to_delete{};
  Eigen::VectorXd starting, turning, ending; // Waypoints : [x,y,xdot,ydot].
  std::cout << "formation: " << globals.FORMATION << '\n';

  if (globals.FORMATION == "varying_network") {
    std::cout << "varying_network\n";
    // Robots must travel to opposite sides of circle
    new_robots_needed_ = false;
    float min_circumference_spacing = 5. * globals.ROBOT_RADIUS;
    double min_radius = 0.25 * globals.WORLD_SZ;
    Eigen::VectorXd centre{{0., 0., 0., 0.}};
    float radius_circle = 50.0;
    for (int i = 0; i < globals.NUM_ROBOTS; i++) {
      // Select radius of large circle to be at least min_radius,
      // Also ensures that robots in the circle are at least
      // min_circumference_spacing away from each other float radius_circle =
      // (globals.NUM_ROBOTS==1) ? min_radius :
      //     std::max(min_radius, sqrt(min_circumference_spacing / (2.
      //     - 2.*cos(2.*PI/(double)globals.NUM_ROBOTS))));

      Eigen::VectorXd offset_from_centre = Eigen::VectorXd{
          {radius_circle * cos(2. * PI * i / (float)globals.NUM_ROBOTS)},
          {radius_circle * sin(2. * PI * i / (float)globals.NUM_ROBOTS)},
          {0.},
          {0.}};
      starting = centre + offset_from_centre;
      ending = centre - offset_from_centre;
      std::deque<Eigen::VectorXd> waypoints{starting, ending};

      // Define robot radius and colour here.
      float robot_radius = globals.ROBOT_RADIUS;
      Color robot_color =
          ColorFromHSV(i * 360. / (float)globals.NUM_ROBOTS, 1., 0.75);
      robots_to_create.push_back(std::make_shared<Robot>(
          this, next_rid_++, waypoints, robot_radius, robot_color));
    }
  } else if (globals.FORMATION == "circle") {
    std::cout << "circle\n";
    // Robots must travel to opposite sides of circle
    new_robots_needed_ = false;
    float min_circumference_spacing = 5. * globals.ROBOT_RADIUS;
    double min_radius = 0.25 * globals.WORLD_SZ;
    const float radius_circle = 50.0;
    Eigen::VectorXd centre{{0., 0., 0., 0.}};
    for (int i = 0; i < globals.NUM_ROBOTS; i++) {
      // Select radius of large circle to be at least min_radius,
      // Also ensures that robots in the circle are at least
      // min_circumference_spacing away from each other float radius_circle =
      // (globals.NUM_ROBOT/ == 1) ? min_radius : std::max(min_radius,
      // sqrt(min_circumference_spacing / (2. - 2. * cos(2. * PI /
      // (double)globals.NUM_ROBOTS))));
      Eigen::VectorXd offset_from_centre = Eigen::VectorXd{
          {radius_circle * cos(2. * PI * i / (float)globals.NUM_ROBOTS)},
          {radius_circle * sin(2. * PI * i / (float)globals.NUM_ROBOTS)},
          {0.},
          {0.}};
      starting = centre + offset_from_centre;
      ending = centre - offset_from_centre;
      std::deque<Eigen::VectorXd> waypoints{starting, ending};

      // Define robot radius and colour here.
      float robot_radius = globals.ROBOT_RADIUS;
      Color robot_color =
          ColorFromHSV(i * 360. / (float)globals.NUM_ROBOTS, 1., 0.75);
      robots_to_create.push_back(std::make_shared<Robot>(
          this, next_rid_++, waypoints, robot_radius, robot_color));
    }    
  } 
   else if (globals.FORMATION == "custom") {
    new_robots_needed_ = false;
    for (int r = 0; r < globals.NUM_ROBOTS; r++) {
      starting = Eigen::VectorXd{
          {1. * random_int(-globals.WORLD_SZ / 2, globals.WORLD_SZ / 2),
           1. * random_int(-globals.WORLD_SZ / 2, globals.WORLD_SZ / 2), 0.,
           0.}};
      ending = Eigen::VectorXd{
          {1. * random_int(-globals.WORLD_SZ / 2, globals.WORLD_SZ / 2),
           1. * random_int(-globals.WORLD_SZ / 2, globals.WORLD_SZ / 2), 0.,
           0.}};

      std::deque<Eigen::VectorXd> waypoints{starting, ending};

      // Define robot radius and colour here.
      float robot_radius = globals.ROBOT_RADIUS;
      Color robot_color =
          ColorFromHSV(r * 360. / (float)globals.NUM_ROBOTS, 1., 0.75);
      robots_to_create.push_back(std::make_shared<Robot>(
          this, next_rid_++, waypoints, robot_radius, robot_color));
    }
    } else if (globals.FORMATION == "junction") {
    // Robots in a cross-roads style junction. There is only one-way traffic,
    // and no turning.
    new_robots_needed_ = true; // This is needed so that more robots can be
                               // created as the simulation progresses.
    int robots_per_second = 5;
    // clock tick is 0.1s, so 5 robots per second
    if (clock_ % (int)(1. / globals.TIMESTEP / robots_per_second) == 0) {
      // int n_roads = 4

      // if (clock_%20==0){              // Arbitrary condition on the
      // simulation time to create new robots
      int n_roads = 2;
      // int road = random_int(0,n_roads-1);
      int road = clock_ % 2 == 0 ? 0 : 1;
      Eigen::Matrix4d rot;
      rot.setZero();
      rot.topLeftCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road),
          sin(PI / 2. * road), cos(PI / 2. * road);
      rot.bottomRightCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road),
          sin(PI / 2. * road), cos(PI / 2. * road);

      int n_lanes = 2;
      int lane = random_int(0, n_lanes - 1);
      double lane_width = 4. * globals.ROBOT_RADIUS;
      double lane_v_offset = (0.5 * (1 - n_lanes) + lane) * lane_width;
      starting = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., lane_v_offset,
                                        globals.MAX_SPEED, 0.}};
      ending = rot * Eigen::VectorXd{
                         {(double)globals.WORLD_SZ, lane_v_offset, 0., 0.}};
      std::deque<Eigen::VectorXd> waypoints{starting, ending};
      float robot_radius = globals.ROBOT_RADIUS;
      Color robot_color = DARKGREEN;
      robots_to_create.push_back(std::make_shared<Robot>(
          this, next_rid_++, waypoints, robot_radius, robot_color));
    }

    // Delete robots if out of bounds
    for (auto [rid, robot] : robots_) {
      if (abs(robot->position_(0)) > globals.WORLD_SZ / 2 ||
          abs(robot->position_(1)) > globals.WORLD_SZ / 2) {
        robots_to_delete.push_back(robot);
      }
    }
  } else if (globals.FORMATION == "junction_twoway") {
    // Robots in a two-way junction, turning LEFT (RED), RIGHT (BLUE) or
    // STRAIGHT (GREEN)
    new_robots_needed_ = true; // This is needed so that more robots can be
                               // created as the simulation progresses.
    if (clock_ % 20 ==
        0) { // Arbitrary condition on the simulation time to create new robots
      int n_roads = 4;
      int road = random_int(0, n_roads - 1);
      // We will define one road (the one going left) and then we can rotate the
      // positions for other roads.
      Eigen::Matrix4d rot;
      rot.setZero();
      rot.topLeftCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road),
          sin(PI / 2. * road), cos(PI / 2. * road);
      rot.bottomRightCorner(2, 2) << cos(PI / 2. * road), -sin(PI / 2. * road),
          sin(PI / 2. * road), cos(PI / 2. * road);

      int n_lanes = 2;
      int lane = random_int(0, n_lanes - 1);
      int turn = random_int(0, 2);
      double lane_width = 4. * globals.ROBOT_RADIUS;
      double lane_v_offset = (0.5 * (1 - 2. * n_lanes) + lane) * lane_width;
      double lane_h_offset = (1 - turn) * (0.5 + lane - n_lanes) * lane_width;
      starting = rot * Eigen::VectorXd{{-globals.WORLD_SZ / 2., lane_v_offset,
                                        globals.MAX_SPEED, 0.}};
      turning = rot * Eigen::VectorXd{{lane_h_offset, lane_v_offset,
                                       (turn % 2) * globals.MAX_SPEED,
                                       (turn - 1) * globals.MAX_SPEED}};
      ending = rot * Eigen::VectorXd{
                         {lane_h_offset + (turn % 2) * globals.WORLD_SZ * 1.,
                          lane_v_offset + (turn - 1) * globals.WORLD_SZ * 1.,
                          0., 0.}};
      std::deque<Eigen::VectorXd> waypoints{starting, turning, ending};
      float robot_radius = globals.ROBOT_RADIUS;
      Color robot_color = ColorFromHSV(turn * 120., 1., 0.75);
      robots_to_create.push_back(std::make_shared<Robot>(
          this, next_rid_++, waypoints, robot_radius, robot_color));
    }

    // Delete robots if out of bounds
    for (auto [rid, robot] : robots_) {
      if (abs(robot->position_(0)) > globals.WORLD_SZ / 2 ||
          abs(robot->position_(1)) > globals.WORLD_SZ / 2) {
        robots_to_delete.push_back(robot);
      }
    }
  } else {
    print("Shouldn't reach here, formation not defined!");
    // Define new formations here!
  }
  // Create and/or delete the robots as necessary.
  for (auto robot : robots_to_create) {
    const auto &wp1 = robot->waypoints_[0];

    // const auto& wp2 = robot->waypoints_[1];

    robot_positions_[robot->rid_] =
        std::vector<double>{robot->waypoints_[0](0), robot->waypoints_[0](1)};
    robots_[robot->rid_] = robot;

    std::array<double, 2> initial_position = {-robot->waypoints_[0](0),
                                              -robot->waypoints_[0](1)};
    // const auto &next_waypoint = robot->waypoints_[1];
    // std::cout << "next_waypoint.size(): " << next_waypoint.size() << '\n';
    const std::array<double, 2> next_waypoint = {-initial_position[0],
                                                 -initial_position[1]};

    const std::array<double, 2> initial_direction = {
        next_waypoint[0] - initial_position[0],
        next_waypoint[1] - initial_position[1]};

    const double length =
        std::sqrt(initial_direction[0] * initial_direction[0] +
                  initial_direction[1] * initial_direction[1]);

    const std::array<double, 2> normalized_direction = {
        initial_direction[0] / length, initial_direction[1] / length};

    const std::array<double, 2> initial_velocity = {
        normalized_direction[0] * globals.MAX_SPEED,
        normalized_direction[1] * globals.MAX_SPEED};

    const double started_at = this->clock_ * globals.TIMESTEP;
    // this->tracked_data_of_each_robot[robot->rid_] =
    this->tracked_data_of_each_robot.emplace(
        robot->rid_,
        TrackedData(initial_position, initial_velocity, started_at, robot->robot_radius_));
    std::vector<std::array<double, 2>> waypoints = {
        {robot->position_(0), robot->position_(1)}};
    for (auto &wp : robot->waypoints_) {
      waypoints.push_back({wp(0), wp(1)});
    }

    assert(waypoints.size() >= 2);

    this->tracked_data_of_each_robot[robot->rid_].waypoints = waypoints;
  };
  for (auto robot : robots_to_delete) {
    deleteRobot(robot);
  };
};

/*******************************************************************************/
// Deletes the robot from the simulator's robots_, as well as any
// variable/factors associated.
/*******************************************************************************/
void Simulator::deleteRobot(std::shared_ptr<Robot> robot) {
  auto connected_rids_copy = robot->connected_r_ids_;
  for (auto r : connected_rids_copy) {
    robot->deleteInterrobotFactors(robots_.at(r));
    robots_.at(r)->deleteInterrobotFactors(robot);
  }
  robots_.erase(robot->rid_);
  robot_positions_.erase(robot->rid_);
}
