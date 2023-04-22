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
		// Functions for harvey.cpp
		double distance(const pair<double,double>& p1, const pair<double,double>& p2) 
		{
			double dx = p1.first - p2.first;
			double dy = p1.second - p2.second;
			return sqrt(dx*dx + dy*dy);
		}

		// Define the function to find the closest safe point on the path

		vector<pair<double,double>> genCircleBarrier(const pair<double,double> obPos, const double robRad, const pair<double,double> curPosi, const double obs_radius)
		{
			const int barRes = 50;
			vector<pair<double,double>> barPoints;
			// Determine the Orientation of the circle
			double barOrient = 0;
			// Generate points along the circle
			for (int i = 0; i <= barRes; i++)
			{
				double angle = i * 2.0 * M_PI / barRes;
				double x = obPos.first() + obs_radius * cos(angle + barOrient);
				double y = obPos.second() + obs_radius * sin(angle + barOrient);
				barPoints.push_back(make_pair(x,y));
			}
			return barPoints;
		}

		pair<double, double> findClosestSafePointOnPath(const vector<pair<double, double>>& path, const pair<double, double>& obstaclePos, const double& dist_near_dyn_obs, const double& offset_dyn_obs, const double& factor_of_safeZone, const pair<double, double>& goalPosi) {
			double safeZone = factor_of_safeZone * offset_dyn_obs;

			// Initialize variables for the closest safe point and its distance to the goal
			pair<double, double> closestPoint;
			double minDistToGoal = INFINITY;

			// Initialize a vector to store all the safe points on the path
			vector<pair<double, double>> safePoints;

			for (int i = 0; i < path.size(); i++) {
				pair<double, double> point = path[i];
				double distToObstacle = distance(point, obstaclePos);

				// Check if the point is within the safe zone
				if (distToObstacle < safeZone) {
					continue;
				}

				// Check if the point is closer to the obstacle than the nearest safe point found so far
				if (distToObstacle > dist_near_dyn_obs) {
					continue;
				}

				// Calculate the distance from the point to the goal
				double distToGoal = distance(point, goalPosi);

				// If the point is closer to the goal than the current closest safe point, update the closest safe point
				if (distToGoal < minDistToGoal) {
					closestPoint = point;
					minDistToGoal = distToGoal;
				}

				// Add the safe point to the vector of safe points
				safePoints.push_back(point);
			}

			// Sort the vector of safe points in increasing order of their distance to the goal
			sort(safePoints.begin(), safePoints.end(), [&](const pair<double, double>& a, const pair<double, double>& b) {
				double distToGoalA = distance(a, goalPosi);
				double distToGoalB = distance(b, goalPosi);
				return distToGoalA < distToGoalB;
			});

			// If there are any safe points, update the closest safe point to be the closest one to the goal
			if (!safePoints.empty()) {
				closestPoint = safePoints[0];
			}

			return closestPoint;
		}

		vector<pair<double,double>> genNonTraversableBarrier(const pair<double,double> obPos, const double robRad, const pair<double,double> curPosi, const pair<double,double> goalPos)
		{
			// Generate circular barrier points
			vector<pair<double,double>> barrierPoints = genCircleBarrier(obPos, robRad, curPosi);
			// Find closest point on original path with respect to obstacle that is "dist_near_dyn_obs" away
			pair<double, double> tempGoal = findClosestPointOnPath(curPosi, path, dist_near_dyn_obs);
			// Generate non-traversable barrier path
			vector<pair<double,double>> barrierPath;
			for (int i = 0; i < barrierPoints.size() - 1; i++)
			{
				pair<double, double> p1 = barrierPoints[i];
				pair<double, double> p2 = barrierPoints[i+1];
				double cost = distance(p1, p2);
				barrierPath.push_back(make_pair(p1, p2, cost));
			}
			// Append line segment from start position to closest point on barrier path
			barrierPath.insert(barrierPath.begin(), make_pair(curPosi, tempGoal, distance(curPosi, tempGoal)));
			// Append line segment from closest point on barrier path to goal position
			barrierPath.push_back(make_pair(tempGoal, goalPos, distance(tempGoal, goalPos)));
			return barrierPath;
		}

		double findClosestObstacle(const pair<double,double>& robPos, const vector<pair<double,double>>& obstacles, const double robotRadius)
		{
			double minDist = numeric_limits<double>::max();
			for (const auto& obstacle : obstacles) {
				double dist = distance(point, obstacle) - robotRadius;
				if (dist < minDist) {
					minDist = dist;
				}
			}
			return minDist;
		}

		vector<pair<double,double>> interpolatePath(const vector<pair<double,double>>& path) {
			// Output path with at least two points
			vector<pair<double,double>> output;
			if (path.size() < 2) {
				return output;
			}

			// Interpolate between adjacent points in the input path
			for (int i = 0; i < path.size() - 1; i++) {
				// Start and end points of the current segment
				pair<double,double> p1 = path[i];
				pair<double,double> p2 = path[i+1];
				
				// Calculate the distance and angle between the start and end points
				double dist = distance(p1, p2);
				double angle = atan2(p2.second - p1.second, p2.first - p1.first);
				
				// Interpolate between the start and end points using a step size of 0.1
				for (double t = 0.0; t <= 1.0; t += 0.1) {
					double x = p1.first + t * dist * cos(angle);
					double y = p1.second + t * dist * sin(angle);
					pair<double,double> point(x, y);
					output.push_back(point);
				}
			}
			
			// Add the final point in the input path to the output path
			output.push_back(path.back());
			
			return output;
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