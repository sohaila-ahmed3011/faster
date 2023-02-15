#ifndef GRID_PATH_SEARCHER_HYBRID_A_STAR_H
#define GRID_PATH_SEARCHER_HYBRID_A_STAR_H

#include "robot_state.h"
#include "grid_graph.h"

using namespace Eigen;

template<typename Graph, typename State>
class KinoAStar {
public:
    KinoAStar() = default;

    ~KinoAStar() = default;

    bool searchPath(const Vec3f &start_pt, const Vec3f &end_pt, std::function<huristics_cost_t(Vec3f a, Vec3f start_velocity, Vec3f b)> calculate_huristics);
    void setGraph(std::shared_ptr<Graph>& graph);
    vec_E<Vec3f> getPath();
    TrajectoryStatePtr ***trajectoryLibrary(const Vec3f &start_pt, const Vec3f &start_velocity,
                                            const Vec3f &target_pt, std::function<huristics_cost_t(Vec3f a, Vec3f start_velocity, Vec3f b)> calculate_huristics);
private:

    std::shared_ptr<Graph> graph_;
    typename State::Ptr terminate_ptr_;
    std::multimap<double,  typename State::Ptr> open_set_;

    int discretize_step_ = 3;
    double max_input_acc_ = 1.0;
    double time_interval_ = 1.0;
    int time_step_ = 50;
};


typedef KinoAStar<GridGraph3D, RobotNode> HybridAStar3D;
#endif //GRID_PATH_SEARCHER_HYBRID_A_STAR_H
