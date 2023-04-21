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

vector<pair<double,double>> genSemiCircleBarrier(const pair<double,double> obPos, const double robRad, const pair<double,double> robPos)
{
    // Initialize Things
    const double barRad = 3*robRad;
    const int barRes = 50;
    vector<pair<double,double>> barPoints;
    // Determine Direction of the Robot
    double robDir = atan2(robPos.second() - obPos.second(),robPos.first() - obPos.first());
    // Determine the Orientation of the semi-circle
    double barOrient = robDir + M_PI_2; // Make sure it's perpendicular
    // Generate points along the semi-circle
    for (int i = 0; i <= barRes; i++)
    {
        double angle = i*M_PI / barRes;
        double x = obPos.first() + barRad*cos(angle + barOrient);
        double y = obPos.second() + barRad*sin(angle + barOrient);
        barPoints.push_back(make_pair(x,y));
    }
    return barPoints;
}

pair<double,double> closestObstacle(const pair<double,double>& robPos, const vector<pair<double,double>>& nodeTable)
{
    double minDist = numeric_limits<double>::max();
    pair<double,double> closestPoint;

    for (const auto& obstacle : nodeTable)
    {
        double dist = sqrt(pow(obstacle.first - point.first, 2) + pow(obstacle.second - point.second, 2));
        if (dist < minDist)
        {
            minDist = dist;
            closestPoint = obstacle;
        }
    }

    return closestPoint;
}

bool withinBarrier(pair<double,double> robPos, vector<pair<double,double>> barrier) 
{
    // Winding Number Algorithm
    
    int i, j, nvert = barrier.size();
    bool c = false;
    for (i = 0, j = nvert - 1;
    i < nvert; j = i++)
    {
        if (((barrier[i].second > robPos.second) != (barrier[j].second > robPos.second)) && 
        (robPos.first < (barrier[j].first - barrier[i].first) * (robPos.second - barrier[i].second) / (barrier[j].second - barrier[i].second) + barrier[i].first))
        {
            c = !c;
        }
    }
    return c;
}

vector<pair<double,double>> modifyPathToAvoidBarrier(const vector<pair<double,double>>& plannedPath, const vector<pair<double,double>>& semiCircleBarrier, const double robRad)
{
    const double barRad = 3*robRad;
    vector<pair<double,double>> modifiedPath;
    // Check each point in the planned path
    for (auto& point : plannedPath)
    {
        // Check if point lies inside the barrier
        bool insideBarrier = false;
        for (auto& barrierPoint : semiCircleBarrier)
        {
            double dist = sqrt(pow(point.first - barrierPoint.first, 2) + pow(point.second - barrierPoint.second, 2));
            if (dist < robRad)
            {
                insideBarrier = true;
                break;
            }
        }
        // Modify the point if it lies inside the barrier
        if (insideBarrier)
        {
            // Move the point away from the barrier along the tangent
            double pointDir = atan2(point.second - obPos.second(), point.first - obPos.first());
            double newPointX = obPos.first() + (barRad + robRad) * cos(pointDir);
            double newPointY = obPos.second() + (barRad + robRad) * sin(pointDir);
            modifiedPath.push_back(make_pair(newPointX, newPointY));
        }
        else
        {
            // Keep the original point if it lies outside the barrier
            modifiedPath.push_back(point);
        }
    }
    return modifiedPath;
}

double distance(const pair<double,double>& p1, const pair<double,double>& p2) 
{
    double dx = p1.first - p2.first;
    double dy = p1.second - p2.second;
    return sqrt(dx*dx + dy*dy);
}

// Define the function to find the closest safe point on the path
pair<double, double> findClosestSafePoint(const pair<double,double>& currentPos, const vector<pair<double,double>>& path, const vector<pair<double,double>>& obstacles, const double robotRadius) 
{
    // Find the closest obstacle to the current position
    pair<double,double> closestObstacle;
    double minDist = numeric_limits<double>::max();
    for (const auto& obstacle : obstacles) {
        double dist = sqrt(pow(obstacle.first - currentPos.first, 2) + pow(obstacle.second - currentPos.second, 2));
        if (dist < minDist) {
            minDist = dist;
            closestObstacle = obstacle;
        }
    }

    // If the closest obstacle is more than 3 times the robot radius away, return the current position
    if (minDist > 3 * robotRadius) {
        return currentPos;
    }

    // Find the closest point on the path to the closest obstacle that is safe
    pair<double,double> closestSafePoint;
    double closestSafeDist = numeric_limits<double>::max();
    for (const auto& pathPoint : path) {
        double dist = sqrt(pow(pathPoint.first - closestObstacle.first, 2) + pow(pathPoint.second - closestObstacle.second, 2));
        if (dist > 3 * robotRadius && dist < closestSafeDist) {
            closestSafeDist = dist;
            closestSafePoint = pathPoint;
        }
    }

    return closestSafePoint;
}

pair<double,double> findClosestPointOnPath(const pair<double,double>& point, const vector<pair<double,double>>& path)
{
    double minDistance = numeric_limits<double>::max();
    pair<double,double> closestPoint;
    for (int i = 0; i < path.size() - 1; i++) {
        pair<double,double> p1 = path[i];
        pair<double,double> p2 = path[i+1];
        pair<double,double> projection = projectPointOnLine(point, p1, p2);
        double distance = distance(point, projection);
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = projection;
        }
    }
    return closestPoint;
}


vector<pair<double,double>> genCircleBarrier(const pair<double,double> obPos, const double robRad, const pair<double,double> robPos)
{
    // Initialize Things
    const double barRad = 3*robRad;
    const int barRes = 50;
    vector<pair<double,double>> barPoints;
    // Determine the Orientation of the circle
    double barOrient = 0;
    // Generate points along the circle
    for (int i = 0; i <= barRes; i++)
    {
        double angle = i * 2.0 * M_PI / barRes;
        double x = obPos.first() + barRad * cos(angle + barOrient);
        double y = obPos.second() + barRad * sin(angle + barOrient);
        barPoints.push_back(make_pair(x,y));
    }
    return barPoints;
}

pair<double, double> findClosestPointOnBarrier(const pair<double, double>& goal, const vector<pair<double, double>>& barrierPoints) 
{
    double minDist = numeric_limits<double>::max();
    pair<double, double> closestPoint;
    
    for (int i = 0; i < barrierPoints.size(); i++) {
        pair<double, double> point = barrierPoints[i];
        double dist = distance(point, goal);
        
        if (dist < minDist) {
            minDist = dist;
            closestPoint = point;
        }
    }
    
    return closestPoint;
}


vector<pair<double,double>> genBarrierPath(const pair<double,double> obPos, const double robRad, const pair<double,double> robPos, const pair<double,double> tempGoal, const vector<pair<double,double>> barrierPoints)
{
    // Find the closest safe point on the barrier path from the current position
    pair<double,double> closestSafePoint = findClosestSafePoint(robPos, barrierPoints, obstacles, robRad);
    
    // Find the closest point on the barrier path from the temporary goal
    pair<double,double> closestPointOnPath = findClosestPointOnPath(tempGoal, barrierPoints);
    
    // Generate a path that connects the closest safe point and the closest point on the barrier path
    vector<pair<double,double>> pathToBarrier;
    pathToBarrier.push_back(robPos);
    pathToBarrier.push_back(closestSafePoint);
    double pathLength = distance(closestSafePoint, closestPointOnPath);
    for (int i = 1; i < 10; i++) {
        double t = i / 10.0;
        double x = closestSafePoint.first + t * (closestPointOnPath.first - closestSafePoint.first);
        double y = closestSafePoint.second + t * (closestPointOnPath.second - closestSafePoint.second);
        pair<double,double> point(x, y);
        pathToBarrier.push_back(point);
    }
    
    // Concatenate the generated path with the barrier path to get the final path
    vector<pair<double,double>> barrierPath;
    for (int i = 0; i < barrierPoints.size() - 1; i++) {
        barrierPath.push_back(barrierPoints[i]);
        barrierPath.push_back(barrierPoints[i+1]);
    }
    barrierPath.push_back(barrierPoints.back());
    barrierPath.push_back(barrierPoints.front());
    
    vector<pair<double,double>> finalPath;
    finalPath.insert(finalPath.end(), pathToBarrier.begin(), pathToBarrier.end());
    finalPath.insert(finalPath.end(), barrierPath.begin(), barrierPath.end());
    
    return finalPath;
}
