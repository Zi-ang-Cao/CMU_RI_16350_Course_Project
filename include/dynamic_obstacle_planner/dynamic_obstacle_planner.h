/*
	FILE: dynamic_obstacle_planner.h
	------------------------
	planner header file for dynamic obs avoidance
*/

#ifndef DYNAMIC_OBSTACLE_PLANNER_H
#define DYNAMIC_OBSTACLE_PLANNER_H

#include <ros/ros.h>
using namespace std;

namespace dynamic_obstacle_planner{
	class dyn_obs_planner{
		private:
		std::string ns_;
		std::string hint_;

		ros::NodeHandle nh_;

		ros::Timer rrtTimer_;
		ros::Timer visTimer_;
		ros::Timer pidTimer_;

		ros::Publisher rrtPathPub_;
		ros::Publisher cmdvelPub_;
		ros::Subscriber odomSub_;
		ros::Subscriber clickedPointSub_; 

		//nav_msgs::Odometry odom_;
		//geometry_msgs::PoseStamped goal_;
		//nav_msgs::Path rrtPathMsg_;



		double desiredVel_;
		double desired_angVel_;
		double Kp_a_;
		double Kp_d_;
		double Ki_a_;
		double Ki_d_;
		double Kd_a_;
		double Kd_d_;
		bool receiveOdom_ = false;
		bool receiveClickedPoint_ = false;
		bool rrtPathUpdated_ = false;
		int i = 0;
		int cmd = 1; // 1 Turning 2 Linear Vel


		public:
		dyn_obs_planner(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void run();

		//void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
		//void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp);
		void rrtCB(const ros::TimerEvent&);
		void visCB(const ros::TimerEvent&);
		void PID(const ros::TimerEvent&);
	};
}

#endif
