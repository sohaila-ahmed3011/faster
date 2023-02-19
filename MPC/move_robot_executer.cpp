#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_husky_robot/ExecuteControlAction.h>
#include <nav_husky_robot/ExecuteControlFeedback.h>
#include <nav_husky_robot/ExecuteControlResult.h>
#include "mpc_control.h"
#include <cmath>

class MissionActionServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<nav_husky_robot::ExecuteControlAction> as_;
    std::string action_name_;
    nav_husky_robot::ExecuteControlFeedback feedback_;
    nav_husky_robot::ExecuteControlResult result_;
    MPCControl controller_;
public:
    MissionActionServer(std::string name, MPCControl controller)
        : as_(nh_, name, boost::bind(&MissionActionServer::executeCB, this, _1), false)
        , action_name_(name)
        , controller_(controller)
    {
        as_.start();
    }

    void executeCB(const nav_husky_robot::ExecuteControlGoalConstPtr &goal)
    {
        ros::Rate r(int(1/controller_.Ts_));
        feedback_.inter_error = 0.0;
        double roll, pitch, yaw;
        controller_.euler_from_quaternion(goal->target_pose.pose.orientation, roll, pitch, yaw);
        Eigen::Vector3d target_pose(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y, yaw);
        ROS_INFO("%s: Executing, moving to a target location %f,%f with current error %f", action_name_.c_str(), target_pose(0), target_pose(1), feedback_.inter_error);
        controller_.mpc_planner_init(target_pose);
        while (ros::ok())
        {
            controller_.move_one_step();
            if (controller_.error_ != 0.0)
            {
                feedback_.inter_error = controller_.error_;
            }
            as_.publishFeedback(feedback_);
            if (controller_.success_)
            {
                result_.final_error = controller_.error_;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
                controller_.success_ = false;
                controller_.init_reg_ = false;
                break;
            }
            r.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_planner");
    MPCControl controller(0.05, 0.2);
    MissionActionServer server(ros::this_node::getName(), controller);
    ros::spin();
    return 0;
}
