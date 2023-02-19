//
// Created by meng on 2021/9/3.
//
#include <hybrid_a_star/kino_a_star.h>
#include <cmath>
#include <unordered_set>

using namespace std;


template<typename Graph, typename State>
void KinoAStar<Graph, State>::setGraph(std::shared_ptr<Graph>& graph){
    graph_ = graph;
}


template<typename Graph, typename State>
bool KinoAStar<Graph, State>::searchPath(const Vec3f &start_pt, const Vec3f &end_pt, std::function<huristics_cost_t(Vec3f a, Vec3f start_velocity, Vec3f b)> calculate_huristics) {
    ros::Time start_time = ros::Time::now();
    Vec3i start_idx = graph_->coord2gridIndex(start_pt);
    Vec3f start_velocity(0.0, 0.0, 0.0);

    typename State::Ptr current_node_ptr = nullptr;
    typename State::Ptr neighbor_node_ptr = nullptr;
    open_set_.clear();


    typename State::Ptr start_node_ptr = new State(start_idx, start_pt);
    start_node_ptr->g_score_ = 0.0;
    start_node_ptr->f_score_ = calculate_huristics(start_pt, start_velocity, end_pt);
    start_node_ptr->id_ = State::WOULD_LIKE; // neither in open list nor in closed list
    open_set_.insert(std::make_pair(start_node_ptr->f_score_, start_node_ptr));

    std::vector< typename State::Ptr> neighbors_ptr;
    std::vector<TrajectoryStatePtr> neighbors_traj_state;

    while (!open_set_.empty()) {
        current_node_ptr = open_set_.begin()->second;
        current_node_ptr->id_ = State::WAS_THERE;  // in closed list
        open_set_.erase(open_set_.begin());

        double dist = (current_node_ptr->robot_state_ - end_pt).norm();
        // terminate if dist to goal is small
        if (dist < graph_->resolution_) {
            terminate_ptr_ = current_node_ptr;
            ros::Duration use_time = ros::Time::now() - start_time;
            ROS_INFO("\033[1;32m --> hybrid A* use time: %f (ms)\033[0m", use_time.toSec() * 1000);
            return true;
        }
        

        TrajectoryStatePtr ***trajectory_state_ptr;
        if (current_node_ptr->trajectory_ == nullptr) {
            trajectory_state_ptr = trajectoryLibrary(current_node_ptr->robot_state_, start_velocity, end_pt, calculate_huristics);
        } else {
            trajectory_state_ptr = trajectoryLibrary(current_node_ptr->robot_state_, current_node_ptr->trajectory_->Velocity.back(), end_pt, calculate_huristics);
        }

        graph_->getNeighbors(trajectory_state_ptr, neighbors_ptr, neighbors_traj_state, discretize_step_);
        // std::cout << "neighbors_ptr SIZE " << neighbors_ptr.size() << std::endl;
        for (unsigned int i = 0; i < neighbors_ptr.size(); ++i) {
            neighbor_node_ptr = neighbors_ptr[i];

            ///TODO: G score 
            double delta_score = (neighbor_node_ptr->robot_state_ - current_node_ptr->robot_state_).norm();
            if (neighbor_node_ptr->id_ == State::WOULD_LIKE) {
                neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + delta_score;
                neighbor_node_ptr->f_score_ = neighbor_node_ptr->g_score_ + neighbors_traj_state[i]->Trajctory_Cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->trajectory_ = neighbors_traj_state[i];
                open_set_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));

                neighbor_node_ptr->id_ = State::WILL_BE; // in open list
                // std::cout << "Debug 1" << std::endl;
                continue;
            } else if (neighbor_node_ptr->id_ == State::WILL_BE) {
                if (neighbor_node_ptr->g_score_ > current_node_ptr->g_score_ + delta_score) {

                    neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + delta_score;
                    neighbor_node_ptr->f_score_ = neighbor_node_ptr->g_score_ + neighbors_traj_state[i]->Trajctory_Cost;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    delete neighbor_node_ptr->trajectory_;
                    neighbor_node_ptr->trajectory_ = neighbors_traj_state[i];

                    typename std::multimap<double,  typename State::Ptr>::iterator map_iter = open_set_.begin();
                    for (; map_iter != open_set_.end(); map_iter++) {
                        if (map_iter->second->robot_grid_index_ == neighbor_node_ptr->robot_grid_index_) {
                            open_set_.erase(map_iter);
                            open_set_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                            break;
                        }
                    }
                    // std::cout << "Debug 2" << std::endl;
                }
                continue;
            } else {
                continue;
            }
        }
    }
    ROS_WARN_STREAM("Hybrid A* failed to search path!");
    return false;
}

template<typename Graph, typename State>
TrajectoryStatePtr ***KinoAStar<Graph, State>::trajectoryLibrary(const Vec3f &start_pt, const Vec3f &start_velocity, const Vec3f &target_pt
                                                    , std::function<huristics_cost_t(Vec3f a, Vec3f start_velocity, Vec3f b)> calculate_huristics_) {
    Vector3d acc_input;
    Vector3d pos, vel;
    int a = 0;
    int b = 0;
    int c = 0;

    double min_Cost = 100000.0;
    double Trajctory_Cost;
    TrajectoryStatePtr ***TraLibrary;
    TraLibrary = new TrajectoryStatePtr **[discretize_step_ + 1];//recored all trajectories after input

    for (int i = 0; i <= discretize_step_; i++) {//acc_input_ax
        TraLibrary[i] = new TrajectoryStatePtr *[discretize_step_ + 1];
        for (int j = 0; j <= discretize_step_; j++) {//acc_input_ay
            TraLibrary[i][j] = new TrajectoryStatePtr[discretize_step_ + 1];
            for (int k = 0; k <= discretize_step_; k++) {//acc_input_az
                vector<Vector3d> Position;
                vector<Vector3d> Velocity;
                acc_input(0) = double(-max_input_acc_ + i * (2 * max_input_acc_ / double(discretize_step_)));
                acc_input(1) = double(-max_input_acc_ + j * (2 * max_input_acc_ / double(discretize_step_)));
                acc_input(2) = double(k * (2 * max_input_acc_ / double(discretize_step_)));

                pos(0) = start_pt(0);
                pos(1) = start_pt(1);
                pos(2) = start_pt(2);
                vel(0) = start_velocity(0);
                vel(1) = start_velocity(1);
                vel(2) = start_velocity(2);
                Position.push_back(pos);
                Velocity.push_back(vel);

                bool collision = false;
                double delta_time;
                delta_time = time_interval_ / double(time_step_);

                ///TODO: Added the function of dynamically planning the trajectory length, when the trajectory is near the target, the trajectory length should be reduced
                for (int step = 0; step <= time_step_; step++) {
                    pos = pos + vel * delta_time + 0.5 * acc_input * delta_time * delta_time;
                    vel = vel + acc_input * delta_time;

                    Position.push_back(pos);
                    Velocity.push_back(vel);
                    double coord_x = pos(0);
                    double coord_y = pos(1);
                    double coord_z = pos(2);
                    if (graph_->isObsFree(coord_x, coord_y, coord_z) != 1) {
                        collision = true;
                    }
                }

                Trajctory_Cost = calculate_huristics_(pos, vel, target_pt);

                TraLibrary[i][j][k] = new TrajectoryState(Position, Velocity, Trajctory_Cost);

                if (collision)
                    TraLibrary[i][j][k]->setCollisionfree();

                if (Trajctory_Cost < min_Cost && !TraLibrary[i][j][k]->collision_check) {
                    a = i;
                    b = j;
                    c = k;
                    min_Cost = Trajctory_Cost;
                }
            }
        }
    }
    TraLibrary[a][b][c]->setOptimal();

    return TraLibrary;
}


template<typename Graph, typename State>
vec_E<Vec3f> KinoAStar<Graph, State>::getPath() {
    vec_E<Vec3f> path;
    std::vector< typename State::Ptr> grid_path;

     typename State::Ptr grid_node_ptr = terminate_ptr_;
    while (grid_node_ptr != nullptr) {
        grid_path.emplace_back(grid_node_ptr);
        grid_node_ptr = grid_node_ptr->parent_node_;
    }

    for (auto &ptr: grid_path) {
        if (ptr->trajectory_ == nullptr) {
            continue;
        }
        for (auto iter = ptr->trajectory_->Position.end() - 1; iter != ptr->trajectory_->Position.begin(); iter--) {
            path.emplace_back(*iter);
        }
    }

    reverse(path.begin(), path.end());

    return path;
}




template class KinoAStar<GridGraph3D, RobotNode>;