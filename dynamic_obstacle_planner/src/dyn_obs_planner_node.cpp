#include <ros/ros.h>
#include <dynamic_obstacle_planner/dynamic_obstacle_planner.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "dyn_obs_planner_node");
    ros::NodeHandle nh;
 
    dynamic_obstacle_planner::dyn_obs_planner n (nh);
    n.run();
    ros::spin();
    return 0;
}