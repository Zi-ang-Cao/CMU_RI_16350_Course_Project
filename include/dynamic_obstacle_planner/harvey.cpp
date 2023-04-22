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

using namespace std;

vector<dynamic_obstacle_planner::dyn_obs_planner::Node> NodeTable(500*500);

namespace dynamic_obstacle_planner{
	dyn_obs_planner::dyn_obs_planner(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "dyn_obs_planner";
		// this->hint_ = "[planner]";	
		this->initModules();
		this->initParam();
		this->registerPub();
	}
    void dyn_obs_planner::initParam(){
        if (not this->nh_.getParam(this->ns_ + "/UGV_velocity", this->desiredVel_)){
			this->desiredVel_ = 0.5;
			cout << "[planner2UGV]: No UGV velocity param. Use default 0.5m/s." << endl;
		}
		else{
			cout << "[planner2UGV]: UGV velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/UGV_angular_velocity", this->desired_angVel_)){
			this->desired_angVel_ = 0.1;
			cout << "[planner2UGV]: No UGV angular velocity param. Use default 0.1m/s." << endl;
		}
		else{
			cout << "[planner2UGV]: UGV velocity is set to: " << this->desired_angVel_ << "m/s." << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Kp_angle", this->Kp_a_)){
			this->Kp_a_ = 1;
			cout << this->hint_ << ": No Angle P Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << ": Angle P Gain " << this->Kp_a_ << endl;
		}	
		if (not this->nh_.getParam(this->ns_ + "/Kp_distance", this->Kp_d_)){
			this->Kp_d_ = 1;
			cout << this->hint_ << ": No Distance P Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << ": Distance P Gain " << this->Kp_d_ << endl;
		}					
		if (not this->nh_.getParam(this->ns_ + "/Ki_angle", this->Ki_a_)){
			this->Ki_a_ = 1;
			cout << this->hint_ << ": No Angle I Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << ": Angle I Gain " << this->Ki_a_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Ki_distance", this->Ki_d_)){
			this->Ki_d_ = 1;
			cout << this->hint_ << ": No Distance I Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << ": Distance I Gain " << this->Ki_d_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Kd_angle", this->Kd_a_)){
			this->Kd_a_ = 1;
			cout << this->hint_ << ": No Angle D Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << ": Angle D Gain " << this->Kd_a_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Kd_distance", this->Kd_d_)){
			this->Kd_d_ = 1;
			cout << this->hint_ << ": No Distance D Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << ": Distance D Gain " << this->Kd_d_ << endl;
		}
    }
    void dyn_obs_planner::initModules(){
        //this->map_.reset(new mapManager::occMap (this->nh_));
    }
    void dyn_obs_planner::registerPub(){
        this->ObsPub_ = this->nh_.advertise<visualization_msgs::Marker>("/dyn_obs", 10);
        this->visPathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/vis_path", 10);
        this->cmdvelPub_ = this->nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }
    void dyn_obs_planner::registerCallback(){
        this->odomSub_ = this->nh_.subscribe("/odom", 1000, &dyn_obs_planner::odomCB, this);
        this->clickedPointSub_ = this->nh_.subscribe("/move_base_simple/goal", 1000, &dyn_obs_planner::clickedPointCB, this);
        this->map_ = this->nh_.subscribe("/map",1000, &dyn_obs_planner::mapCallback, this);
        
        // rrt timer
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &dyn_obs_planner::plannerCB, this);

		// visualization
		this->visDynObsTimer_ = this->nh_.createTimer(ros::Duration(1), &dyn_obs_planner::dynObsCB, this);

        // PID
        this->pidTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dyn_obs_planner::PID, this);
    }

    bool dyn_obs_planner::achieve_temp_goal() {
        return false;
    }

    void dyn_obs_planner::refresh_NodeTable() {
        for (int idx=0; idx<NodeTable.size(); idx++) {
            NodeTable[idx].g = INF;
            NodeTable[idx].isClosed = false;
            NodeTable[idx].isOpen = false;
        }
    }

    void dyn_obs_planner::plannerCB(const ros::TimerEvent&){
        // Only Enter the Planner After Reviceving an Odom update
        if (!this->receiveOdom_) return;

        if (this->receiveClickedPoint_) {
            // reset flag
            this->replan_for_newGoal=true;
            // reset h, g and f in NodeTable
            refresh_NodeTable();
        }

        // Only check dynamic obs with a generated has_plan_to_execute
        if (this->has_plan_to_execute) {
            // this->near_dyn_obs may become true in dynObsCB() function!! 
            if (this->near_dyn_obs) {
                if (this->do_rewiring) this->replan_for_dynamicObs=true;
                else {
                    printf("Even the dynamic obstacle is close, no need to replan! Just follow the previous rewiring!\n");
                    this->replan_for_dynamicObs=false;
                }
            }
            // this->near_dyn_obs may back to false once achieve the temporary goal.
            if (achieve_temp_goal()) {
                this->do_rewiring = true;

                // un-Lable one-time un-traversable zone!!
                UN_Lable_avoidZone();
                
                this->near_dyn_obs = false;
            }
        }

        if ((!this->replan_for_newGoal) && (!this->replan_for_dynamicObs)) return;
        
        // Replaning is required -- no matter for newGoal or to avoid DynObs
        update_new_plan = true;

        if (this->replan_for_newGoal) {
            refresh_NodeTable();
            // Replan for a new goal
            replan_pair_vec.clear();
            printf("Received a new goal AND generating path via A*");
            pair<double,double> curPosi = make_pair(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y);
            pair<double,double> goalPosi = make_pair(this->goal_.pose.position.x, this->goal_.pose.position.y);
        
            AStarBaseline(&replan_pair_vec, curPosi, goalPosi);
            replan_for_newGoal = false;
            do_rewiring = true;
        }
        else if (this->replan_for_dynamicObs) {
            switch (this->REPLAN_STRATEG_Indicator)
            {
            case 1: {
                // Do Partial A* Replanning
                pair<double,double> curPosi = make_pair(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y);
                pair<double,double> goalPosi = make_pair(this->goal_.pose.position.x, this->goal_.pose.position.y);
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
            }
            case 2: {
                // do_incremental_search
                pair<double,double> curPosi = make_pair(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y);
                pair<double,double> goalPosi = make_pair(this->goal_.pose.position.x, this->goal_.pose.position.y);
                // We want to find nearest obstacle
                // insert function here
                while (curPosi != goalPosi)
                {
                    AStarBaseline(&replan_pair_vec, curPosi, goalPosi);
                    if (this->near_dyn_obs)
                    {
                        // Generate circular barrier points
                        vector<pair<double,double>> barrierPoints = genCircleBarrier(obPos, robRad, robPos);
                        // Generate temporary goal
                        pair<double, double> tempGoal = findClosestPointOnBarrier(const pair<double, double>& goal, const vector<pair<double, double>>& barrierPoints) 
                        // Generate path
                        vector<pair<double,double>> barrierPath = genBarrierPath(obPos, robRad, robPos, tempGoal, barrierPoints)
                        // Interpoolte through that path
                    }
                }
            }
            default: {
                // do_Entire_AStar_Replan

                refresh_NodeTable();
                // Replan for a new goal
                replan_pair_vec.clear();
                printf("Received a new goal AND generating path via A*");
                pair<double,double> curPosi = make_pair(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y);
                pair<double,double> goalPosi = make_pair(this->goal_.pose.position.x, this->goal_.pose.position.y);
            
                AStarBaseline(&replan_pair_vec, curPosi, goalPosi);
                replan_for_newGoal = false;
                do_rewiring = true;
                
                break;
            }
            }
            
            // Reset Flag
            replan_for_dynamicObs = false;
            // [POTENTIAL BUG] NEED TO CHANGE THE PIPELINE FOR MULTIPLE DYNAMiC OBSTACLE
            do_rewiring = false;    
        }

        if (update_new_plan) {
            this->has_plan_to_execute = true;
            this->path_ref_index = 1;   // Essential reset!!!!
            updatePathVisVec(replan_pair_vec);
            this->pathMsgConverter(replan_pair_vec, this->path_msg_);
            this->pathVisMsg_.markers = this->pathVisVec_;
            this->visPathPub_.publish(this->pathVisMsg_);
            cout << "Published Path via AStarBaseline" << endl;
            // Reset Flag
            update_new_plan = false;
        }
        
        // Only when the path being published, set the receiveClickedPoint_=false;
        this->receiveClickedPoint_ = false;        
    }

    void dyn_obs_planner::Lable_avoidZone() {
        printf("Lable_avoidZone before calling AStar\n");
    }
    void dyn_obs_planner::UN_Lable_avoidZone() {
        printf("REMOVE Labled avoidZone for previous rewiring!!!\n");
    }



    void dyn_obs_planner::AStarBaseline(vector<pair<double,double>>* replan_pair_vec,
                                        pair<double,double> curPosi, pair<double,double> goalPosi) {
        // === A Star Begin ===
        Node* nodePtr;
        Node* curNodePtr;
        Node* nextNodePtr;
        Node* goalNodePtr;
        Node* startNodePtr;
        double newx;
        double newy;
        int minDiff;
        int maxDiff;

        startNodePtr = & NodeTable[GETMAPINDEX(curPosi.first,curPosi.second,this->x_size)];
        goalNodePtr = & NodeTable[GETMAPINDEX(goalPosi.first,goalPosi.second,this->x_size)];
        //cout<< startNodePtr->X <<" " <<startNodePtr->Y<<endl;
        if (OPEN.size() == 0) {
            startNodePtr->isOpen = true;
            startNodePtr->g = 0;
            startNodePtr->h = -1;
            startNodePtr->f = 0;
            OPEN.push_back(startNodePtr);
        }
        while ((!OPEN.empty()) & (!goalNodePtr->isClosed)) {
            // [1] Remove s with the smallest [f(s) = g(s) + h(s)] from OPEN;
            sort(OPEN.begin(), OPEN.end(), DecendingSortPtr());
            curNodePtr = OPEN.back();
            OPEN.pop_back();

            // [2] Insert s into CLOSED;
            CLOSED.push_back(curNodePtr);
            curNodePtr->isClosed = true;

            // [3] For Every successor s' of s such that s' not in CLOSE
            for(int dir = 0; dir < NUMOFDIRS; dir++) {
                newx = curNodePtr->X + STEP_SIZE*dX[dir];
                newy = curNodePtr->Y + STEP_SIZE*dY[dir];

                // Check it whether within the map
                if (newx >= -1*BOUNDARY && newx <= BOUNDARY && newy >= -1*BOUNDARY && newy <= BOUNDARY) {
                    // Prepare nodePtr for new Position
                    nodePtr = & NodeTable[GETMAPINDEX(newx,newy,this->x_size)];
                    
                    // Skip for Static_Obs
                    if (nodePtr->is_Static_Obs) continue;
                    // Skip for Dynamic_Obs
                    if (nodePtr->is_Dynamic_Obs) continue;

                    // Skip for s' is in CLOSED
                    if (nodePtr->isClosed) continue;
                    // Updates h and g -- Allow chasing moving target
                    minDiff = (int) (MIN(abs(newx-goalPosi.first), abs(newy-goalPosi.second)) / STEP_SIZE);
                    maxDiff = (int) (MAX(abs(newx-goalPosi.first), abs(newy-goalPosi.second)) / STEP_SIZE);
                    nodePtr->h= (maxDiff - minDiff) + 1.4 * (minDiff);
                    if (nodePtr->g > curNodePtr->g + cost[dir]) {
                        nodePtr->g = curNodePtr->g + cost[dir]; // Update the optimal cost
                        nodePtr->prevNodePtr = curNodePtr;  // Record the prevNodePtr!!!!
                        // Add unVisited Nodes to Open
                        if (! nodePtr->isOpen) {
                            nodePtr->isOpen = true;
                            OPEN.push_back(nodePtr);
                        }
                    }
                    nodePtr->f = nodePtr->g + eps * nodePtr->h;
                }
            }
        }
        OPEN.clear();
        CLOSED.clear();

        nodePtr = goalNodePtr;
        // startNodePtr = &NodeTable[GETMAPINDEX(curPosi.first,curPosi.second,x_size)];
        list<pair<double,double>> plan;
        while (nodePtr->prevNodePtr != startNodePtr) {
            plan.push_back(make_pair(nodePtr->X, nodePtr->Y));
            nodePtr = nodePtr->prevNodePtr;
        }
        plan.push_back(make_pair(nodePtr->X, nodePtr->Y));
        plan.reverse();

        // vector<pair<double,double>> replan_pair_vec;
        replan_pair_vec->clear();

        for (auto pair: plan) {
            replan_pair_vec->push_back(pair);
            // Print out the plan
            //printf("Move to x=%g, y=%g\n", pair.first, pair.second);
        }
        
        // === A Star END ===
    }

    void dyn_obs_planner::dynObsCB(const ros::TimerEvent&){
        if (this->obs_loop !=0){

            for (double x= (this->obs_pos_x - this->offset_dyn_obs); x<(this->obs_pos_x + this->offset_dyn_obs); x+=STEP_SIZE) {
                for (double y= (this->obs_pos_y - this->offset_dyn_obs); y<(this->obs_pos_y + this->offset_dyn_obs); y+=STEP_SIZE) {
                    // Remove obstacle from NodeTable
                    NodeTable[GETMAPINDEX(x,y, x_size)].is_Dynamic_Obs = false;
                }
            }

            // Update obs position
            this->obs_pos_x = (this->obs_pos_x)+this->move[randomInt(0,2)];
            this->obs_pos_y = (this->obs_pos_y)+this->move[randomInt(0,2)];
        }
        else{
            this->obs_pos_x = this->obs_initial_x;
            this->obs_pos_y = this->obs_initial_y;

            // One-Time Initialization
            this->marker_.type = this->shape;
            this->marker_.header.frame_id = "map";
            this->marker_.scale.x = 2*this->obs_radius; // 0.3
            this->marker_.scale.y = 2*this->obs_radius; // 0.3
            this->marker_.scale.z = 1.0;
            this->marker_.color.r = 0.0f;
            this->marker_.color.g = 1.0f;
            this->marker_.color.b = 0.0f;
            this->marker_.color.a = 1.0;            
        }
        this->obs_loop++;
        if (this->obs_pos_x > obs_x_upperBound){
            this->obs_pos_x = obs_x_upperBound;
        }
        if (this->obs_pos_x < obs_x_lowerBound){
            this->obs_pos_x = obs_x_lowerBound;
        }
        if (this->obs_pos_y > obs_y_upperBound){
            this->obs_pos_y = obs_y_upperBound;
        }
        if (this->obs_pos_y < obs_y_lowerBound){
            this->obs_pos_y = obs_y_lowerBound;
        }



        // Update flags "near_dyn_obs"
        double length = sqrt(pow(obs_pos_x - this->odom_.pose.pose.position.x, 2) + pow(obs_pos_y - this->odom_.pose.pose.position.y, 2));
        if (length <= factor_of_safeZone*offset_dyn_obs) {
            this->near_dyn_obs = true;
        }else this->near_dyn_obs = false;

        // Update NodeTable
        for (double x= (this->obs_pos_x - this->offset_dyn_obs); x<(this->obs_pos_x + this->offset_dyn_obs); x+=STEP_SIZE) {
            for (double y= (this->obs_pos_y - this->offset_dyn_obs); y<(this->obs_pos_y + this->offset_dyn_obs); y+=STEP_SIZE) {
                // Add obstacle to NodeTable
                NodeTable[GETMAPINDEX(x,y, x_size)].is_Dynamic_Obs = true;
            }
        }

        this->marker_.pose.position.x = this->obs_pos_x;
        this->marker_.pose.position.y = this->obs_pos_y;
        this->marker_.pose.position.z = 0.5;
        this->ObsPub_.publish(this->marker_);
		//cout << "dynamic obstacle cb" << endl;
    }

    void dyn_obs_planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        std_msgs::Header header = msg->header;
        nav_msgs::MapMetaData info = msg->info;
        this->cost_map = msg->data;
        int width = info.width;
        int height = info.height;
        this->x_size = height;
        ROS_INFO("Got map %d %d", info.width, info.height);

        if (this->gen_map_flag == true){
            createmap();
            this->gen_map_flag = false;
            ROS_INFO("Build the NodeTable Done");
        }
    }

    void dyn_obs_planner::createmap(){
        bool is_static_obs = false;
        bool is_unexplored = false;
        double inflate_x;
        double inflate_y;
        double x;
        double y;


        for ( x= -1*BOUNDARY; x<=((this->x_size*STEP_SIZE)-BOUNDARY); x+=STEP_SIZE) {
            for ( y= -1*BOUNDARY; y<=((this->x_size*STEP_SIZE)-BOUNDARY); y+=STEP_SIZE) {
                is_static_obs = false;
                is_unexplored = false;
                // cout<<"Y Value: "<<y <<endl;
                // cout<<"X Value: "<<x <<endl;
                // NodeTable[GETMAPINDEX(x,y, x_size)] = new Node();
                NodeTable[GETMAPINDEX(x,y, x_size)].X = x;
                NodeTable[GETMAPINDEX(x,y, x_size)].Y = y;

                // Low Efficient Stategy (But Easiest)
                is_static_obs = ((int)(this->cost_map[GETMAPINDEX(x,y,this->x_size)]) > OBSTACLE_VALUE);
                is_unexplored = (int)(this->cost_map[GETMAPINDEX(x,y,this->x_size)]) < 0;
                // Add obstacle to NodeTable
                if (is_static_obs | is_unexplored) {
                    for (inflate_x= (x - this->offset_static_obs); inflate_x<(x + this->offset_static_obs); inflate_x+=STEP_SIZE) {
                        for (inflate_y= (y - this->offset_static_obs); inflate_y<(y + this->offset_static_obs); inflate_y+=STEP_SIZE) {
                            if (inflate_x < -1*BOUNDARY) continue;
                            if (inflate_x >= ((this->x_size*STEP_SIZE)-BOUNDARY)) continue;
                            if (inflate_y < -1*BOUNDARY) continue;
                            if (inflate_y >= ((this->x_size*STEP_SIZE)-BOUNDARY)) continue;

                            NodeTable[GETMAPINDEX(inflate_x,inflate_y, x_size)].is_Static_Obs = true;
                        }
                    }
                }
                if (is_static_obs | is_unexplored) {
                    for ( inflate_x= (x - this->obs_radius); inflate_x<(x + this->obs_radius); inflate_x+=STEP_SIZE) {
                        for ( inflate_y= (y - this->obs_radius); inflate_y<(y + this->obs_radius); inflate_y+=STEP_SIZE) {
                            
                            if (inflate_x < -1*BOUNDARY) inflate_x = -1*BOUNDARY;
                            if (inflate_x >= BOUNDARY) inflate_x = BOUNDARY;
                            if (inflate_y < -1*BOUNDARY) inflate_y = -1*BOUNDARY;
                            if (inflate_y >= BOUNDARY) inflate_y = BOUNDARY;

                            NodeTable[GETMAPINDEX(inflate_x,inflate_y, x_size)].isTraversable_for_DynObs = false;
                        }
                    }
                }
                // Checker Done == ALl Static Obstacle would be remembered here
            }
        }
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
    inline double rpy_from_quaternion(const geometry_msgs::Quaternion& quat){
	    // return is [0, 2pi]
	    tf2::Quaternion tf_quat;
	    tf2::convert(quat, tf_quat);
	    double roll, pitch, yaw;
	    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
	    return yaw;
	}
    void dyn_obs_planner::PID(const ros::TimerEvent&){
        // Only Enter the Planner After Reviceving an Odom update
        if (not this->receiveOdom_) return;
        if (not this->has_plan_to_execute) return;

		//cout<<"PID CB" <<endl;
        //double fixed_time=;
		int endIdx=path_msg_.poses.size();
        //cout<<endIdx<<endl;

        printf("path_msg_.poses.size() = %d, with path_ref_index=%d, cmd=%d\n", endIdx, path_ref_index, cmd);


		if (this->path_msg_.poses.size() != 0){
		// if (endIdx != 0) {
			// cmd 1 = angular turning, 2 = linear forward
			// collect data
            if (this->path_ref_index > endIdx){
                this->path_ref_index = endIdx;
            }
            //cout<<this->path_ref_index<<endl;
			double target_x = path_msg_.poses[this->path_ref_index].pose.position.x;
			double target_y = path_msg_.poses[this->path_ref_index].pose.position.y;
			double current_x = odom_.pose.pose.position.x;
			double current_y = odom_.pose.pose.position.y;
			double current_vel_x = odom_.twist.twist.linear.x;
			double current_vel_ang = odom_.twist.twist.angular.z;
            //cout<<"current x: "<<current_x<<"target x: "<<target_x<<endl;
			double x_diff = (target_x-current_x);
			// convert data to angle
			double target_angle = atan2((target_y-current_y),x_diff);
			if (x_diff == 0){
				target_angle = 0;
			}
			
			double current_angle = rpy_from_quaternion(odom_.pose.pose.orientation);
			double error_angle = target_angle - current_angle;
			double error_dis = sqrt(pow((target_x - current_x),2) + pow((target_y-current_y),2));

            // cout<<error_angle<<endl;
			// double derivative_angle = (error_angle-prev_error_angle)/fixed_time;
			// double derivative_dis   = (error_dis-prev_error_dis)/fixed_time;

			// wrap to 2PI
			if (error_angle >= M_PI){
				error_angle -= 2*M_PI;
			}
			if (error_angle <= -M_PI){
				error_angle += 2*M_PI; 
			}
			
            // [POTENTIAL BUG] Add 
			if (abs(error_dis) <= 0.1){
				this->path_ref_index += 10;
				this->path_ref_index =std::min(this->path_ref_index, int(path_msg_.poses.size())-1);
				cmd = 1;
			}
			else if (abs(error_angle) <= 0.001){
				cmd  = 2;
                // [Que] No need to add this line???
                // this->path_ref_index +=1;
			}

            // printf("path_msg_.poses.size() = %d, with path_ref_index=%d, cmd=%d, error_dis=%f, error_angle=%f\n", endIdx, path_ref_index, cmd, error_dis, error_angle);


			// apply control

			// prev_error_angle=error_angle;
			// prev_error_dis  =error_dis;
			double output_angle_vel = (this->Kp_a_*error_angle) + (this->Kd_a_*current_vel_ang);
			double output_linear_vel = (this->Kp_d_*error_dis) + (this->Kd_d_*current_vel_x);

			cout<< "output_angle_vel = " << output_angle_vel << endl;

			
			//set max and min speed
			if (output_angle_vel >= this->desired_angVel_){
				output_angle_vel = this->desired_angVel_;
			}
			if (output_angle_vel <= -this->desired_angVel_){
				output_angle_vel = -this->desired_angVel_;
			}
			if (output_linear_vel >= this->desiredVel_){
				output_linear_vel = this->desiredVel_;
			}
			//cout << output_angle_vel <<endl;
			//publish to UGV
			geometry_msgs::Twist Twist;
			if (cmd ==1){
				Twist.angular.z = output_angle_vel;
                Twist.linear.x=0;
				this->cmdvelPub_.publish(Twist);
			}
			if (cmd ==2){
                Twist.angular.z = 0;
				Twist.linear.x = output_linear_vel;
				this->cmdvelPub_.publish(Twist);
			}	

			// Termination condition
			double noice, noice1, noice2;
			noice  = abs((path_msg_.poses[endIdx-1].pose.orientation.w) - (odom_.pose.pose.orientation.w));
			// std::cout << path_msg_poses[endIdx-1]<< endl;
			noice1 =abs(path_msg_.poses[endIdx-1].pose.position.x - odom_.pose.pose.position.x);
			noice2 =abs(path_msg_.poses[endIdx-1].pose.position.y - odom_.pose.pose.position.y);

			//std::cout << noice  <<":   orientation"<<endl;
			//std::cout << noice1 <<":   pose x"<< endl;
			//std::cout << noice2 <<":   pose y"<< endl;
			//std::cout << this->path_ref_index <<": waypoint"<< endl;
			//std::cout << this->path_msg_poses.size()  <<": waypoint size"<< endl;

			// noice <=0.1
			if (noice2 <=0.25 && noice1 <=0.25 && path_ref_index == endIdx-1 )
			{
				std::cout << "goal achieved " << std::endl;
				cmd = 1;
				// this->receiveClickedPoint_ = false;
				Twist.angular.z=0;
				Twist.linear.x=0;
				this->cmdvelPub_.publish(Twist);
				nav_msgs::Path emptyPath;
				this->path_msg_ = emptyPath;

                this->has_plan_to_execute = false;
			}

		}
	}
    void dyn_obs_planner::run(){
        // register timer callback
        this->registerCallback();
	}

}