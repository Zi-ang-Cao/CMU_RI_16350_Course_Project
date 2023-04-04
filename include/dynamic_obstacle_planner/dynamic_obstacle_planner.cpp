#include <dynamic_obstacle_planner/dynamic_obstacle_planner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

namespace dynamic_obstacle_planner{
	dyn_obs_planner::dyn_obs_planner(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "dyn_obs_planner";
		// this->hint_ = "[planner]";	
		// this->initModules();
		// this->initParam();
		// this->registerPub();
	}
    void dyn_obs_planner::initParam(){
    }
    void dyn_obs_planner::initModules(){
        //this->map_.reset(new mapManager::occMap (this->nh_));
    }
    void dyn_obs_planner::registerPub(){

    }
    void dyn_obs_planner::registerCallback(){
        this->odomSub_ = this->nh_.subscribe("/odom", 1000, &dyn_obs_planner::odomCB, this);
        this->clickedPointSub_ = this->nh_.subscribe("/move_base_simple/goal", 1000, &dyn_obs_planner::clickedPointCB, this);
        this->map_ = this->nh_.subscribe("/map",1000, &dyn_obs_planner::mapCallback, this);
        
        // rrt timer
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dyn_obs_planner::plannerCB, this);

		// visualization
		//this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dyn_obs_planner::visCB, this);
    }
    void dyn_obs_planner::plannerCB(const ros::TimerEvent&){
		if (not this->receiveOdom_) return;
		if (not this->receiveClickedPoint_) return;		
		cout << "planner cb" << endl;
    }
    void dyn_obs_planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        std_msgs::Header header = msg->header;
        nav_msgs::MapMetaData info = msg->info;
        //int8_t data = msg->data;
    }
    void dyn_obs_planner::clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
		this->goal_.pose.position.x = cp->pose.position.x;
		this->goal_.pose.position.y  = cp->pose.position.y;
		this->goal_.pose.position.z  = 0.5; // set height to be 0.3 m
		this->receiveClickedPoint_ = true;
	}
    void dyn_obs_planner::odomCB(const nav_msgs::Odometry::ConstPtr& msg){
	    this->odom_ = *msg;
	    this->odom_.pose.pose.position.z = 0.5;
	    this->receiveOdom_ = true;
	}
    void dyn_obs_planner::run(){
        // register timer callback
        this->registerCallback();
	}

}