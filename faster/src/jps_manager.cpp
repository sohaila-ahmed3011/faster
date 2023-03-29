/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "jps_manager.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/StdVector>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <assert.h>
#include <stdlib.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include "termcolor.hpp"

// using namespace JPS;

JPS_Manager::JPS_Manager()
{
  map_util_ = std::make_shared<JPS::VoxelMapUtil>();
  planner_ptr_ = std::unique_ptr<JPSPlanner3D>(new JPSPlanner3D(false));
  hybrid_a_star_ptr_ = std::make_shared<Planner>();
}

void JPS_Manager::setNumCells(int cells_x, int cells_y, int cells_z)
{
  cells_x_ = cells_x;
  cells_y_ = cells_y;
  cells_z_ = cells_z;
}

void JPS_Manager::setFactorJPS(double factor_jps)
{
  factor_jps_ = factor_jps;
}

void JPS_Manager::setResolution(double res)
{
  res_ = res;
}

void JPS_Manager::setInflationJPS(double inflation_jps)
{
  inflation_jps_ = inflation_jps;
}

void JPS_Manager::setZGroundAndZMax(double z_ground, double z_max)
{
  z_ground_ = z_ground;
  z_max_ = z_max;
}

void JPS_Manager::setVisual(bool visual)
{
  visual_ = visual;
}

void JPS_Manager::setDroneRadius(double drone_radius)
{
  drone_radius_ = drone_radius;
}

void JPS_Manager::cvxEllipsoidDecomp(vec_Vecf<3>& path, int type_space, std::vector<LinearConstraint3D>& l_constraints,
                                     vec_E<Polyhedron<3>>& poly_out)
{
  /*  if (takeoff_done_ == false)
    {
      // std::cout << bold << green << "No takeoff_done_ done yet" << std::endl;
      vec_Vec3f empty_obs;
      ellip_decomp_util_uo2_.set_obs(empty_obs);  // No unkown space when taking off
    }
    else
    {*/
  if (type_space == UNKOWN_AND_OCCUPIED_SPACE)
  {
    ellip_decomp_util_.set_obs(vec_uo_);
  }
  else
  {
    ellip_decomp_util_.set_obs(vec_o_);
  }
  //}
  ellip_decomp_util_.set_local_bbox(Vec3f(2, 2, 1));  // Only try to find cvx decomp in the Mikowsski sum of JPS and
                                                      // this box (I think) par_.drone_radius
  ellip_decomp_util_.set_inflate_distance(drone_radius_);  // The obstacles are inflated by this distance
  ellip_decomp_util_.dilate(path);                         // Find convex polyhedra
  // decomp_util.shrink_polyhedrons(par_.drone_radius);  // Shrink polyhedra by the drone radius. NOT RECOMMENDED (leads
  // to lack of continuity in path sometimes)

  // Convert to inequality constraints Ax < b
  // std::vector<polytope> polytopes;
  auto polys = ellip_decomp_util_.get_polyhedrons();

  l_constraints.clear();

  for (size_t i = 0; i < path.size() - 1; i++)
  {
    const auto pt_inside = (path[i] + path[i + 1]) / 2;
    LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());

    // Now add the constraint "Above the ground:"
    cs.A_.conservativeResize(cs.A_.rows() + 1, cs.A_.cols());
    cs.A_.row(cs.A_.rows() - 1) = -Eigen::Vector3d::UnitZ();
    cs.b_.conservativeResize(cs.b_.rows() + 1, cs.b_.cols());
    cs.b_[cs.b_.rows() - 1] = -z_ground_;

    l_constraints.push_back(cs);
  }
  poly_out = ellip_decomp_util_.get_polyhedrons();
}

void JPS_Manager::updateJPSMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, Eigen::Vector3d& center, nav_msgs::OccupancyGridPtr ocubptr, Vec3f _start_velocity_)
{
  Vec3f center_map = center;  // state_.pos;
  
  hybridVel_ =  _start_velocity_;
  
  mtx_jps_map_util.lock();

  if {use_hybrid_a_}
  {
    hybrid_a_star_ptr_->setmap(ocubptr, cells_x_, cells_y_, cells_z_, factor_jps_ * res_ ); //map to hybrid a star
  }else
  {
    map_util_->readMap(pclptr, cells_x_, cells_y_, cells_z_, factor_jps_ * res_, center_map, z_ground_, z_max_,
                     inflation_jps_);  // Map read to jps
  }
 

  

  mtx_jps_map_util.unlock();
}

vec_Vecf<3> JPS_Manager::solveJPS3D(Vec3f& start_sent, Vec3f& goal_sent, bool* solved, int i)
{
  Eigen::Vector3d start(start_sent(0), start_sent(1), std::max(start_sent(2), 0.0));
  Eigen::Vector3d goal(goal_sent(0), goal_sent(1), std::max(goal_sent(2), 0.0));

  Vec3f originalStart = start;

  pcl::PointXYZ pcl_start = eigenPoint2pclPoint(start);
  pcl::PointXYZ pcl_goal = eigenPoint2pclPoint(goal);

  ///////////////////////////////////////////////////////////////
  /////////////////////////// RUN JPS ///////////////////////////
  ///////////////////////////////////////////////////////////////

  mtx_jps_map_util.lock();

  vec_Vecf<3> path;
  path.clear();

  if {use_hybrid_a_}
  {
    bool valid_HAS = hybrid_a_star_ptr_->plan(start, goal, hybridVel_); //using hybrid a star planner

    if (valid_HAS == true)  // There is a solution
    {
      path = hybrid_a_star_ptr_->getPath();  // getpar_.RawPath() if you want the path with more corners (not "cleaned")
      if (path.size() > 1)
      {
        path[0] = start;
        path[path.size() - 1] = goal;  // force to start and end in the start and goal (and not somewhere in the voxel)
      }
      else
      {  // happens when start and goal are very near (--> same cell)
        vec_Vecf<3> tmp;
        tmp.push_back(start);
        tmp.push_back(goal);
        path = tmp;
      }
      
      if (initiate_path)
      {
        path_logger.open("/home/ros/ros_ws/src/faster/faster/src/path_hybrid_a_star.txt", std::ofstream::out | std::ofstream::trunc);
        path_logger.close();
        initiate_path = false;
      }

      path_logger.open("/home/ros/ros_ws/src/faster/faster/src/path_hybrid_a_star.txt", std::ios_base::app);
      path_logger << path << "\n";
      path_logger.close();

    }
    else
    {
      std::cout << "Hybrid A* didn't find a solution from " << start.transpose() << " to " << goal.transpose() << std::endl;
    }
    mtx_jps_map_util.unlock();

    *solved = valid_HAS;
    return path;
  }
  else
  {
    // Set start and goal free
    const Veci<3> start_int = map_util_->floatToInt(start);
    const Veci<3> goal_int = map_util_->floatToInt(goal);

    map_util_->setFreeVoxelAndSurroundings(start_int, inflation_jps_);
    map_util_->setFreeVoxelAndSurroundings(goal_int, inflation_jps_);

    planner_ptr_->setMapUtil(map_util_);  // Set collision checking function
    bool valid_jps = planner_ptr_->plan(start, goal, 1, true);  // Plan from start to goal with heuristic weight=1, and
                                                                using JPS (if false --> use A*)
    
    if (valid_jps == true)  // There is a solution
    {
      path = planner_ptr_->getPath();  // getpar_.RawPath() if you want the path with more corners (not "cleaned")
      if (path.size() > 1)
      {
        path[0] = start;
        path[path.size() - 1] = goal;  // force to start and end in the start and goal (and not somewhere in the voxel)
      }
      else
      {  // happens when start and goal are very near (--> same cell)
        vec_Vecf<3> tmp;
        tmp.push_back(start);
        tmp.push_back(goal);
        path = tmp;
      }

      if (initiate_path)
      {
        path_logger.open("/home/ros/ros_ws/src/faster/faster/src/path_jps.txt", std::ofstream::out | std::ofstream::trunc);
        path_logger.close();
        initiate_path = false;
      }

      path_logger.open("/home/ros/ros_ws/src/faster/faster/src/path_jps.txt", std::ios_base::app);
      path_logger << path << "\n";
      path_logger.close();
    
    }
    else
    {
      std::cout << "JPS didn't find a solution from" << start.transpose() << " to " << goal.transpose() << std::endl;
    }
    mtx_jps_map_util.unlock();

    *solved = valid_jps;
    return path;
  }
}
