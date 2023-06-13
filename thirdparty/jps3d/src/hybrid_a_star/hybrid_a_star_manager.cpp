#include <hybrid_a_star/hybrid_a_star_manager.h>

void Planner::costMapCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr){
    // std::cout<< "getting cost map" << std::endl;
    _cost_map_ = costmap_msg_ptr;
}

void Planner::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom = *msg;
}


void Planner::setmap(nav_msgs::OccupancyGridPtr costMapPtr, int _x_size, int _y_size, int _z_size, double res){
    // initialize cost map
    costMapCallBack(costMapPtr);
    //////////////////////////////////////HARD CODED///////////////////////
    _x_size = 10;
    _y_size = 10;
    
    //res = 0.2; // enforce higher resolution for faster computation
    _map_resolution = res;
    //initialize grid graph parameters with dimension 10 by 10 by 4
    // std::cout << " _x_size " << _x_size << std::endl;
    // std::cout << " _y_size " << _y_size << std::endl;
    //_z_size = _z_size * res ;
    _z_size = 1; //Hard coded
    // try to change x and y size to match the incoming occ grid

    _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
    _map_upper << _x_size / 2.0, _y_size / 2.0, _z_size;

    _inv_map_resolution = 1.0 / res;

    _max_x_id = (int) (_x_size * _inv_map_resolution);
    _max_y_id = (int) (_y_size * _inv_map_resolution);
    _max_z_id = (int) (_z_size * _inv_map_resolution);
    
    // std::cout<< "xyz_lower: " << _map_lower << std::endl;
    // std::cout<< "xyz_upper: " << _map_upper << std::endl;
    // std::cout<< "grid size: " <<  Vector3i(_max_x_id, _max_y_id, _max_z_id) << std::endl;
    // std::cout<< "resolution: " << res << std::endl;

    //run only once
    if (initBool_){
      graph_ = std::make_shared<GridGraph3D>();
      _hybrid_a_star = std::make_shared<HybridAStar3D>();
    //   std::cout<<"res "<<res << " | _max_x_id: "<<_max_x_id<<" | _max_y_id: "<<_max_y_id << std::endl;
      initBool_ = false;
    }

    graph_->InitGridMap(_map_lower, _map_upper, Vector3i(_max_x_id, _max_y_id, _max_z_id), res);
    // graph_->InitGridMap(res, _max_x_id, _max_y_id, params);
    // std::cout<< "setmap" << std::endl;
    setObstacles();
    _hybrid_a_star->setGraph(graph_);

}


bool Planner::plan(Vec3f &start,  Vec3f &goal, Vec3f _start_velocity_) {
    Vec3f target_pt = goal;
    start[2] = 0;
    target_pt[2] = 0;
    _start_velocity_ = Vec3f(0,0,0);
    std::function<huristics_cost_t(Vec3f, Vec3f, Vec3f)> heuristic_optimal_bvp = &optimal_boundary_value_problem<Vec3f>;
    // TrajectoryStatePtr*** trajectory_lib = _hybrid_a_star->trajectoryLibrary(start, _start_velocity_, target_pt, heuristic_optimal_bvp);
    bool solved_bool = _hybrid_a_star->searchPath(start, target_pt, heuristic_optimal_bvp); 
    std::function<huristics_cost_t(RobotNode::Ptr,  RobotNode::Ptr)> heuristic = &calculate_euclidean_dis<RobotNode::Ptr>;
    


    // graph_->InitGridMap(_map_lower, _map_upper, Vector3i(_max_x_id, _max_y_id, _max_z_id), _map_resolution);
    // graph_->InitGridMap(_map_resolution, _max_x_id, _max_y_id, params);  // result in unknown console output
  
    return solved_bool;
}

vec_E<Vec3f> Planner::getPath(){
    auto finalPath = _hybrid_a_star->getPath();
    if(graph_ != nullptr){
        graph_->reset(); 
    }
    return finalPath;
}


void Planner::setObstacles(){
    unsigned int map_w = std::floor(_cost_map_->info.width / _map_resolution);
    unsigned int map_h = std::floor(_cost_map_->info.height / _map_resolution);
    for (unsigned int w = 0; w < map_w; ++w) {
        for (unsigned int h = 0; h < map_h; ++h) {
            auto x = static_cast<unsigned int> ((w + 0.5) * _map_resolution/ _cost_map_->info.resolution);
            auto y = static_cast<unsigned int> ((h + 0.5) * _map_resolution/ _cost_map_->info.resolution);
            if (_cost_map_->data[y * _cost_map_->info.width + x] == 100) {
                graph_->SetObstacle(std::make_unsigned_t<int>(w), std::make_unsigned_t<int>(h));
            }
        }
    }
    // std::cout << "_map_resolution: " << _map_resolution << "  ||| _cost_map_->info.resolution: " << _cost_map_->info.resolution << std::endl;
    // std::cout << "_cost_map_->info.width: " << _cost_map_->info.width << " ||| _cost_map_->info.height: " << _cost_map_->info.height  << std::endl;
    // std::cout << "map_w: " << map_w << " map_h: " << map_h << std::endl;
    // std::cout << "_cost_map_->data.size() is "<< _cost_map_->data.size() << std::endl;
    // std::cout << "No of Obstacles is " << obsCount << "  &&&  " << "No of free is " <<freeCount << std::endl;
}




