/*
	FILE: dynamic_obstacle_planner.h
	------------------------
	planner header file for dynamic obs avoidance
*/

#ifndef DYNAMIC_OBSTACLE_PLANNER_H
#define DYNAMIC_OBSTACLE_PLANNER_H

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <random>
using namespace std;

// === Map ===
#include <vector>
#include <map>
//access to the map is shifted to account for 0-based indexing in the map
#define BOUNDARY 10.0 
#define STEP_SIZE 0.05
// #define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)
#define GETMAPINDEX(X, Y, XSIZE) ( (int) (floor((Y + BOUNDARY)/STEP_SIZE)*XSIZE + floor((X + BOUNDARY)/STEP_SIZE)))

#define OBSTACLE_VALUE 50
#define INF 0x7FFFFF

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif


// === A Star BEGIN ===
#define NUMOFDIRS 8
#define eps 3



namespace dynamic_obstacle_planner{
	class dyn_obs_planner{
		private:
		std::string ns_;
		std::string hint_;

		ros::NodeHandle nh_;

		ros::Timer plannerTimer_;
		ros::Timer visDynObsTimer_;
		ros::Timer pidTimer_;

		ros::Publisher visPathPub_;
		ros::Publisher cmdvelPub_;
		ros::Publisher ObsPub_;
		ros::Subscriber odomSub_;
		ros::Subscriber clickedPointSub_;
		ros::Subscriber map_;

		nav_msgs::Odometry odom_;
		geometry_msgs::PoseStamped goal_;
		nav_msgs::Path path_msg_;
		
		// visualize dynamic obstacle
		visualization_msgs::Marker marker_;
		uint32_t shape = visualization_msgs::Marker::CUBE;
		int obs_loop = 0;
		double obs_radius = 0.2;
		double obs_initial_x = -1.2;
		// double obs_initial_x = 3.0;
		double obs_initial_y = 1.0;
        // double obs_initial_y = 2.0;

		double obs_movable_range = 2.0;

		double obs_x_upperBound = obs_movable_range + obs_initial_x;
		double obs_y_upperBound = obs_movable_range + obs_initial_y;
		double obs_x_lowerBound = -1*obs_movable_range + obs_initial_x;
		double obs_y_lowerBound = -1*obs_movable_range + obs_initial_y;


		double obs_pos_x;
		double obs_pos_y;
		double move[3] = {-0.05, 0.0, 0.05};

		// COST MAP INFLATION
		double rob_radius = 0.3;
		double offset_dyn_obs = obs_radius+rob_radius;
		double offset_static_obs = rob_radius;

		// visualize path
		std::vector<visualization_msgs::Marker> pathVisVec_;
		visualization_msgs::MarkerArray pathVisMsg_;

		bool gen_map_flag = true;

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
		// path_ref_index
		int path_ref_index = 1;
		int cmd = 1; // 1 Turning 2 Linear Vel

		public:
		dyn_obs_planner(const ros::NodeHandle& nh);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();
		void run();

		void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
		void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp);
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void plannerCB(const ros::TimerEvent&);
		void dynObsCB(const ros::TimerEvent&);
		void createmap();
		void PID(const ros::TimerEvent&);
		int randomInt( int LB, int UB ){
  			//std::uniform_real_distribution<double> unif(LB,UB);
  			//std::default_random_engine re;
			//return round(unif(re));
			int randNum = rand()%(UB-LB + 1) + LB;
			return randNum;	
		}
		void updatePathVisVec(const std::vector<pair<double,double>> &plan){
			this->pathVisVec_.clear();
			visualization_msgs::Marker waypoint;
			visualization_msgs::Marker line;
			geometry_msgs::Point p1, p2;
			std::vector<geometry_msgs::Point> lineVec;
			for (size_t i=0; i < plan.size(); ++i){
				auto currentPoint = plan[i];
				if (i != plan.size() - 1){
					auto nextPoint = plan[i+1];
					p1.x = currentPoint.first;
					p1.y = currentPoint.second;
					p1.z = 0;
					p2.x = nextPoint.first;
					p2.y = nextPoint.second;
					p2.z = 0; 
					lineVec.push_back(p1);
					lineVec.push_back(p2);
				}
				// waypoint
				waypoint.header.frame_id = "map";
				waypoint.id = 1+i;
				waypoint.ns = "rrt_path";
				waypoint.type = visualization_msgs::Marker::SPHERE;
				waypoint.pose.position.x = currentPoint.first;
				waypoint.pose.position.y = currentPoint.second;
				waypoint.pose.position.z = 0;
				waypoint.scale.x = 0.2;
				waypoint.scale.y = 0.2;
				waypoint.scale.z = 0.2;
				waypoint.color.a = 0.8;
				waypoint.color.r = 0.3;
				waypoint.color.g = 1;
				waypoint.color.b = 0.5;
				//pathVisVec_.push_back(waypoint);
			}
			line.header.frame_id = "map";
			line.points = lineVec;
			line.ns = "rrt_path";
			line.id = 0;
			line.type = visualization_msgs::Marker::LINE_LIST;
			line.scale.x = 0.05;
			line.scale.y = 0.05;
			line.scale.z = 0.05;
			line.color.a = 1.0;
			line.color.r = 0.5;
			line.color.g = 0.1;
			line.color.b = 1;
			pathVisVec_.push_back(line);
		}
		void pathMsgConverter(const std::vector<pair<double,double>> &plan, nav_msgs::Path& path){
			std::vector<geometry_msgs::PoseStamped> pathVec;
			for (auto p: plan){
				geometry_msgs::PoseStamped ps;
				ps.header.stamp = ros::Time();
				ps.header.frame_id = "map";
				ps.pose.position.x = p.first;
				ps.pose.position.y = p.second;
				ps.pose.position.z = 0;
				pathVec.push_back(ps);
			}
			path.poses = pathVec;
			path.header.stamp = ros::Time();
			path.header.frame_id = "map";
		}

		// === Planner Control Bool ===
		bool has_plan_to_execute = false;
		bool replan_for_newGoal = false;
		bool replan_for_dynamicObs = false;
		bool do_rewiring = false;

		bool update_new_plan = false;
		vector<pair<double,double>> replan_pair_vec;

		bool achieve_temp_goal();

		void refresh_NodeTable();

		int REPLAN_STRATEG_Indicator = 1;




		
		// === Incremental Search BEGIN ===
		double factor_of_safeZone = 1.6;
		// double factor_of_safeZone = 1.6;
		bool near_dyn_obs = false;
		bool need_rewiring = true;


        bool do_incremental_search = false;

		void Lable_avoidZone();
		void UN_Lable_avoidZone();

		// === Incremental Search END ===
		
		
		// === A Star BEGIN ===
		void AStarBaseline(vector<pair<double,double>>* plan_vec, pair<double,double> curPosi, pair<double,double> goalPosi);

		struct Node {
			double X;
  			double Y;

			// For Robot
			bool is_Static_Obs = false;
			bool is_Dynamic_Obs = false;
			
			// For dynamic obstacle
			bool isTraversable_for_DynObs = true;

			float g=INF;
			float h=INF;
			float f=INF;

			bool isOpen=false;
			bool isClosed=false;
			Node* prevNodePtr;
		};
		struct DecendingSortPtr {
			bool operator()(Node* const& n1, Node* const& n2)
			{
				// return "true" if "n1" is ordered before "n2", for example:
				// return (n1->g + n1->h) > (n2->g + n2->h);
				return (n1->f) > (n2->f);
			}
		};
		vector<Node*> OPEN;
		vector<Node*> CLOSED;

		// MODIFY!!!
		// vector<Node*> NodeTable; // -- Cause bug
		//map<int, Node*> NodeTable;

		vector<int8_t> cost_map;
		int x_size;
		//8-connected grid
		double dX[NUMOFDIRS] = 	{-1, -1, -1,  0,  0,  1, 1, 1};    
		double dY[NUMOFDIRS] = 	{-1,  0,  1, -1,  1, -1, 0, 1};
		float cost[NUMOFDIRS] = {1.4, 1, 1.4, 1, 1, 1.4, 1, 1.4};

		// === A Star END ===
	};
}

#endif