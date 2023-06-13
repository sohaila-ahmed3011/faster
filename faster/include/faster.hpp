/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

// #include <iostream>

#include <pcl/kdtree/kdtree.h>
#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdlib.h>
#include <boost/thread.hpp>

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/make_unique.hpp>


#include "timer.hpp"
#include "termcolor.hpp"
#include "faster_types.hpp"
// Solvers includes
//#include "solvers/solvers.hpp" CVXGEN solver interface
#include "solverGurobi.hpp"
#include "jps_manager.hpp"

#define MAP 1          // MAP refers to the occupancy grid
#define UNKNOWN_MAP 2  // UNKNOWN_MAP refers to the unkown grid

#define RETURN_LAST_VERTEX 0
#define RETURN_INTERSECTION 1

// status_ : YAWING-->TRAVELING-->GOAL_SEEN-->GOAL_REACHED-->YAWING-->TRAVELING-->...

enum DroneStatus
{
  YAWING = 0,
  TRAVELING = 1,
  GOAL_SEEN = 2,
  GOAL_REACHED = 3
};

enum PlannerStatus
{
  FIRST_PLAN = 0,
  START_REPLANNING = 1,
  REPLANNED = 2
};

using namespace JPS;
using namespace termcolor;

struct multi_plan_return{
 std::vector<LinearConstraint3D> constraints;
 vec_E<Polyhedron<3>> polys;
};

class Faster
{
public:
  Faster(parameters par);
  // Faster() {};
  bool init();
  void replan(vec_Vecf<3>& JPS_safe_out, vec_Vecf<3>& JPS_whole_out, vec_E<Polyhedron<3>>& poly_safe_out,
              vec_E<Polyhedron<3>>& poly_whole_out, std::vector<state>& X_safe_out, std::vector<state>& X_whole_out);
  void updateState(state data);
  multi_plan_return multi_plan_any_point(state A, state &E, bool &solvedjps, vec_E<Polyhedron<3>> &poly_tmp, 
                            std::vector<LinearConstraint3D> &l_constraints_whole_, JPS_Manager &jps_manager_);
  void multi_plan_E(state A, state &E, state &G, bool &solvedjps, vec_E<Polyhedron<3>> &poly_whole_out, 
                    double ra, vec_Vecf<3> &JPS_whole, vec_Vecf<3> &JPS_in,vec_Vecf<3> &JPSk);

 void push_job(Faster * worker,state A, state &E, bool &solvedjps, vec_E<Polyhedron<3>> &poly_tmp, 
                                  std::vector<LinearConstraint3D> &l_constraints_whole_, JPS_Manager &jps_manager_); 
 
  void updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map, pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk,  
                      nav_msgs::OccupancyGridPtr costmap_msg_ptr, Vec3f _start_velocity_);
  bool getNextGoal(state& next_goal);
  void getState(state& data);
  void getG(state& G);  
  void setTerminalGoal(state& term_goal);
  void resetInitialization();

private:
  std::ofstream timer_logger;
  bool initiate_timer {true};
  int feasibility_counter_{0};
  state M_;
  std::deque<state> plan_;

  double previous_yaw_ = 0.0;

  typedef boost::packaged_task<multi_plan_return> task_t;
  typedef boost::shared_ptr<task_t> ptask_t;
  std::vector<boost::shared_future<multi_plan_return>> pending_data;
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  std::unique_ptr<boost::asio::io_service::work> service_work;

  // 0 for single point, 1 for multi points, 2 for multi thread
  int option {2};
  bool initiate_threads {true}; // flag for the threads
  bool initiate_time {true}; // flag for time files
  bool initiate_volume {true}; // flag for volume files
  double total_volume_single {0};
  double total_volume_multi {0};
  bool showVolume_ {true}; // If true, write free space volume in txt file


  SolverGurobi sg_whole_;  // solver gurobi whole trajectory
  SolverGurobi sg_safe_;   // solver gurobi whole trajectory

  JPS_Manager jps_manager_;  // Manager of JPS
  // JPS_Manager jps_manager_1;  // Manager of JPS 1
  // JPS_Manager jps_manager_2;  // Manager of JPS 2


  void yaw(double diff, state& next_goal);

  void getDesiredYaw(state& next_goal);

  // void yaw(double diff, snapstack_msgs::QuadGoal& quad_goal);
  void createMoreVertexes(vec_Vecf<3>& path, double d);

  int findIndexR(int indexH);

  int findIndexH(bool& needToComputeSafePath);
  bool ARisInFreeSpace(int index);

  void updateInitialCond(int i);

  void changeDroneStatus(int new_status);

  Eigen::Vector3d getPos(int i);
  Eigen::Vector3d getVel(int i);

  Eigen::Vector3d getAccel(int i);
  Eigen::Vector3d getJerk(int i);
  // Returns the first collision of JPS with the map (i.e. with the known obstacles). Note that JPS will collide with a
  // map B if JPS was computed using an older map A
  // If type_return==Intersection, it returns the last point in the JPS path that is at least par_.inflation_jps from
  // map
  Eigen::Vector3d getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map, int type_return);

  bool appendToPlan(int k_end_whole, const std::vector<state>& whole, int k_safe, const std::vector<state>& safe);

  bool initialized();
  bool initializedAllExceptPlanner();

  void print_status();

  parameters par_;

  // SeedDecomp3D seed_decomp_util_;

  bool state_initialized_ = false;
  bool planner_initialized_ = false;

  double current_yaw_ = 0;

  double desired_yaw_old_ = 0;

  double alpha_before_ = 0;
  double desired_yaw_B_ = 0;

  vec_E<Polyhedron<3>> polyhedra_;
  std::vector<LinearConstraint3D> l_constraints_whole_;  // Polytope (Linear) constraints
  std::vector<LinearConstraint3D> l_constraints_safe_;   // Polytope (Linear) constraints
  std::ofstream time_logger;
  std::ofstream volume_logger;

  int deltaT_ = 10;
  int deltaT_min_ = 10;
  int indexR_ = 0;

  Eigen::MatrixXd U_safe_, X_safe_;
  double spinup_time_;
  double z_start_;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_map_;       // kdtree of the point cloud of the occuppancy grid
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_unk_;       // kdtree of the point cloud of the unknown grid
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_frontier_;  // kdtree of the frontier

  bool kdtree_map_initialized_ = 0;
  bool kdtree_unk_initialized_ = 0;

  bool terminal_goal_initialized_ = false;

  int cells_x_;  // Number of cells of the map in X
  int cells_y_;  // Number of cells of the map in Y
  int cells_z_;  // Number of cells of the map in Z

  int n_states_publised_ = 0;  // Number of goals=states published

  int drone_status_ = DroneStatus::TRAVELING;  // status_ can be TRAVELING, GOAL_SEEN, GOAL_REACHED
  int planner_status_ = PlannerStatus::FIRST_PLAN;

  bool force_reset_to_0_ = 1;

  vec_Vecf<3> JPS_old_;

  double dyaw_filtered_ = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk_;

  std::mutex mtx_map;  // mutex of occupied map (kdtree_map_)
  std::mutex mtx_unk;  // mutex of unkonwn map (pclptr_unk_)
  std::mutex mtx_frontier;
  std::mutex mtx_inst;  // mutex of instanteneous data (v_kdtree_new_pcls_)
  std::mutex mtx_goals;

  std::mutex mtx_k;
  std::mutex mtx_X_U_temp;
  std::mutex mtx_X_U_safe;
  std::mutex mtx_X_U;
  std::mutex mtx_planner_status_;
  std::mutex mtx_initial_cond;
  std::mutex mtx_state;
  std::mutex mtx_offsets;
  std::mutex mtx_plan_;
  // std::mutex mtx_factors;

  std::mutex mtx_G;
  std::mutex mtx_G_term;

  Eigen::Vector3d pos_old_;
  Eigen::Vector3d B_;

  bool to_land_ = false;
  bool JPSk_solved_ = false;

  state stateA_;  // It's the initial condition for the solver
  // flightmode flight_mode_;
  state state_;
  state G_;       // This goal is always inside of the map
  state G_term_;  // This goal is the clicked goal
  int recv_success_count{0};
  int recv_fail_count{0};
};
