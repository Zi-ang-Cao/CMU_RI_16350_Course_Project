#include <dynamic_obstacle_planner/dynamic_obstacle_planner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cmath>

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

// Define a function that linearly interpolates from the start condition to goal condition
vector<double> linearInterpolation(double startX, double startY, double goalX, double goalY, double stepSize) {
    vector<double> interpolated;
    int numSteps = static_cast<int>(1.0 / stepSize);

    for (int i = 0; i <= numSteps; i++) {
        double t = static_cast<double>(i) / numSteps;
        vector<double> temp;
        double x = startX + t * (goalX - startX);
        double y = startY + t * (goalY - startY);
        temp.push_back(x);
        temp.push_back(y);
        interpolated.insert(interpolated.end(), temp.begin(), temp.end());
    }

    return interpolated;
}

void harveyplannerCB(const vector<double>& startCond, const vector<double>& goalCond, const double& stepSize) 
{
    vector<double> interpolated = linearInterpolation(startX,  startY,  goalX,  goalY,  stepSize);

    // Print the interpolated values
    cout << "Interpolated values:" << endl;
    for (int i = 0; i < interpolated.size(); i++) {
        cout << interpolated[i] << " ";
    }
    cout << endl;
}

void incrementalPlanner(const vector<double>& startCond, const vector<double>& goalCond, const double& stepSize, const double robotDim)
{
    vector<pair<double,double>> plan;
    vector<double> goal = goalCond;
    vector<double> currCond = startCond;
    // if there is an obstacle with in the safety margin (3*the robot size)
        // plan around the obstacle to the nearest increment that does not violate the safety margin


}
////////////////////////////////////////////////////////////////////////////////////////////////// ALL USED FUNCTIONS START HERE ///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////// ANYTHING BEFORE THIS LINE IS NOT USED //////////////////////////////////////////////////////////////////////////////////////////////////////
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

case 2: {
    // Do Partial A* Replanning
    vector<pair<double,double>> remaining_path_vec;
    AStarBaseline(&replan_pair_vec, curPosi, goalPosi);
    this->near_dyn_obs = false;
    double dist_near_dyn_obs = offset_dyn_obs*factor_of_safeZone;
    while (curPosi != goalPosi)
    {
        double obPos = findClosestObstacle(curPosi, obstacles, robRad);
        if (this->near_dyn_obs)
        {
            // ********DON'T FIND CLOSEST POINT ON BARRIER, FIND CLOSEST POINT TO NEAR_DYN_OBS FIELD*******************
            pair<double,double> tempGoal = findClosestSafePointOnPath(&replan_pair_vec, ObPos, dist_near_dyn_obs, offset_dyn_obs, factor_of_safeZone)
            // Generate a non-traversable barrier
            vector<pair<double,double>> barrierPath = genNonTraversableBarrier(obPos, robRad, curPosi, tempGoal);
            // Plan new temporary path with AStarBaseline around barrier
            vector<pair<double,double>> temp_path_vec;
            AStarBaseline(&temp_path_vec, curPosi, tempGoal);
            // Append remaining path from original path to new path
            for (int i = 0; i < temp_path_vec.size(); i++)
            {
                replan_pair_vec.push_back(temp_path_vec[i]);
            }
            // Interpolate through the path to generate a smooth trajectory
            vector<pair<double,double>> smoothPath = interpolatePath(replan_pair_vec);
            // Update remaining_path_vec with new smooth path
            remaining_path_vec.clear();
            for (int i = 0; i < smoothPath.size() - 1; i++)
            {
                pair<double, double> p1 = smoothPath[i];
                pair<double, double> p2 = smoothPath[i+1];
                double cost = distance(p1, p2);
                remaining_path_vec.push_back(make_pair(p1, p2, cost));
            }
        }
        else
        {
            // Traverse original path
            for (int i = 0; i < replan_pair_vec.size(); i++)
            {
                pair<double, double> p1 = replan_pair_vec[i].first;
                pair<double, double> p2 = replan_pair_vec[i].second;
                double cost = replan_pair_vec[i].cost;
                // Check if robot is close enough to the goal position and break out of loop if true
                if (distance(curPosi, goalPosi) < goal_tolerance)
                {
                    break;
                }
                // Update current position to the end of the path
                curPosi = p2;
                // Append remaining path to remaining_path_vec
                remaining_path_vec.push_back(make_pair(p1, p2, cost));
                // Check if robot is near an obstacle and break out of loop if true
                double newObPos = findClosestObstacle(curPosi, obstacles, robRad);
                if (newObPos < INFINITY)
                {
                    break;
                }
            }
        }
    }
    // Append remaining path to remaining_path_vec
    for (int i = 0; i < replan_pair_vec.size(); i++)
    {
        pair<double, double> p1 = replan_pair_vec[i].first;
        pair<double, double> p2 = replan_pair_vec[i].second;
        double cost = replan_pair_vec[i].cost;
        remaining_path_vec.push_back(make_pair(p1, p2, cost));
    }
}



// Reduce the number of function calls
// Prune the replan pair vector
// Consider a heuristic to estimate distance to goal
// Use multi-threading or parallel processing