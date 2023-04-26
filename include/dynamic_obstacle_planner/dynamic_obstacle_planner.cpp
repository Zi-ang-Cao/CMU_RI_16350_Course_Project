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
        // === Additional Variables ===
        if (not this->nh_.getParam(this->ns_ + "/factor_of_safeZone", this->factor_of_safeZone)){
			this->factor_of_safeZone = 3.0;
			cout << "[dyn_obs_planner]: factor_of_safeZone. Use default 3.0." << endl;
		}else{
			cout << "[dyn_obs_planner]: factor_of_safeZone is set to: " << this->factor_of_safeZone << endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/REPLAN_STRATEG_Indicator", this->REPLAN_STRATEG_Indicator)){
			this->REPLAN_STRATEG_Indicator = 1;
			cout << "[dyn_obs_planner]: No REPLAN_STRATEG_Indicator param. Use default 1." << endl;
		}else{
			cout << "[dyn_obs_planner]: REPLAN_STRATEG_Indicator is set to: " << this->REPLAN_STRATEG_Indicator << endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/Barrier_Resolution", this->Barrier_Resolution)){
			this->Barrier_Resolution = 400;
			cout << "[dyn_obs_planner]: No Barrier_Resolution param. Use default 400." << endl;
		}else{
			cout << "[dyn_obs_planner]: Barrier_Resolution is set to: " << this->Barrier_Resolution << endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/Barrier_Radius_Factor", this->Barrier_Radius_Factor)){
			this->Barrier_Radius_Factor = 2.0;
			cout << "[dyn_obs_planner]: No Barrier_Radius_Factor param. Use default 2.0." << endl;
		}else{
			cout << "[dyn_obs_planner]: Barrier_Radius_Factor is set to: " << this->Barrier_Radius_Factor << endl;
		}

        if (not this->nh_.getParam(this->ns_ + "/obs_marker_factor", this->obs_marker_factor)){
			this->obs_marker_factor = 1.5;
			cout << "[dyn_obs_planner]: No obs_marker_factor param. Use default 1.5." << endl;
		}else{
			cout << "[dyn_obs_planner]: obs_marker_factor is set to: " << this->obs_marker_factor << endl;
        }

        // === Baseline Variables ===
        if (not this->nh_.getParam(this->ns_ + "/Debug", this->Debug)){
			this->Debug = false;
			cout << "[dyn_obs_planner]: No debugger mode param. Use default false." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Debugger Mode is set to: " << this->Debug << endl;
		}
        if (not this->nh_.getParam(this->ns_ + "/pseudo_click", this->pseudo_click)){
			this->pseudo_click = false;
			cout << "[dyn_obs_planner]: No pseudo click param. Use default false." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Pseudo Click is set to: " << this->pseudo_click << endl;
		}
        if (not this->nh_.getParam(this->ns_ + "/DynObs_rate", this->DynObs_rate)){
			this->DynObs_rate = 0.1;
			cout << "[dyn_obs_planner]: No obstacle publish rate param. Use default 0.1 sec." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Dynamic Obstacle Publish Rate is set to: " << this->DynObs_rate<< "sec." << endl;
		}
        if (not this->nh_.getParam(this->ns_ + "/PseudoRob_rate", this->PseudoRob_rate)){
			this->PseudoRob_rate = 0.1;
			cout << "[dyn_obs_planner]: No pseudo robot publish rate param. Use default 0.1 sec." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Pseudo Robot Publish Rate is set to: " << this->PseudoRob_rate<< "sec." << endl;
		}
        if (not this->nh_.getParam(this->ns_ + "/obs_radius", this->obs_radius)){
			this->obs_radius = 1;
			cout << "[dyn_obs_planner]: No obstacle radius param. Use default 1 m." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Dynamic Obstacle Radius is set to: " << this->obs_radius<< "m." << endl;
		}
        if (not this->nh_.getParam(this->ns_ + "/obs_initial_x", this->obs_initial_x)){
			this->obs_initial_x = 0;
			cout << "[dyn_obs_planner]: No obstacle initial position param. Use default 0 m." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Dynamic Obstacle Position is set to: " << this->obs_initial_x<< "m." << endl;
		}
        if (not this->nh_.getParam(this->ns_ + "/obs_initial_y", this->obs_initial_y)){
			this->obs_initial_y = 0;
			cout << "[dyn_obs_planner]: No obstacle initial position param. Use default 1 m." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Dynamic Obstacle Position is set to: " << this->obs_initial_y<< "m." << endl;
		}
        if (not this->nh_.getParam(this->ns_ + "/obs_movable_range", this->obs_movable_range)){
			this->obs_movable_range = 1;
			cout << "[dyn_obs_planner]: No obstacle movable range param. Use default 1 m." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Dynamic Obstacle Movable Range is set to: " << this->obs_movable_range<< "m." << endl;
        }
        if (not this->nh_.getParam(this->ns_ + "/Robot_velocity", this->desiredVel_)){
			this->desiredVel_ = 0.5;
			cout << "[dyn_obs_planner]: No robot velocity param. Use default 0.5m/s." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Robot velocity is set to: " << this->desiredVel_ << "m/s." << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Robot_angular_velocity", this->desired_angVel_)){
			this->desired_angVel_ = 0.1;
			cout << "[dyn_obs_planner]: No robot angular velocity param. Use default 0.1m/s." << endl;
		}
		else{
			cout << "[dyn_obs_planner]: Robot velocity is set to: " << this->desired_angVel_ << "m/s." << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Kp_angle", this->Kp_a_)){
			this->Kp_a_ = 1;
			cout << this->hint_ << ": No Angle P Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << "[dyn_obs_planner]: Angle P Gain " << this->Kp_a_ << endl;
		}	
		if (not this->nh_.getParam(this->ns_ + "/Kp_distance", this->Kp_d_)){
			this->Kp_d_ = 1;
			cout << this->hint_ << "[dyn_obs_planner]: No Distance P Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << "[dyn_obs_planner]: Distance P Gain " << this->Kp_d_ << endl;
		}					
		if (not this->nh_.getParam(this->ns_ + "/Ki_angle", this->Ki_a_)){
			this->Ki_a_ = 1;
			cout << this->hint_ << "[dyn_obs_planner]: No Angle I Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << "[dyn_obs_planner]: Angle I Gain " << this->Ki_a_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Ki_distance", this->Ki_d_)){
			this->Ki_d_ = 1;
			cout << this->hint_ << "[dyn_obs_planner]: No Distance I Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << "[dyn_obs_planner]: Distance I Gain " << this->Ki_d_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Kd_angle", this->Kd_a_)){
			this->Kd_a_ = 1;
			cout << this->hint_ << "[dyn_obs_planner]: No Angle D Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << "[dyn_obs_planner]: Angle D Gain " << this->Kd_a_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/Kd_distance", this->Kd_d_)){
			this->Kd_d_ = 1;
			cout << this->hint_ << "[dyn_obs_planner]: No Distance D Gain entered. Default set to 1" << endl;
		}
		else{
			cout << this->hint_ << "[dyn_obs_planner]: Distance D Gain " << this->Kd_d_ << endl;
		}
        this->param_flag = true;
    }
    void dyn_obs_planner::initModules(){
        //this->map_.reset(new mapManager::occMap (this->nh_));
    }
    void dyn_obs_planner::registerPub(){
        this->ObsPub_ = this->nh_.advertise<visualization_msgs::Marker>("/dyn_obs", 10);
        this->visPathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/vis_path", 10);
        this->visReplanPathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/vis_repath", 10);
        this->cmdvelPub_ = this->nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        this->PseudoRobotPub_ = this->nh_.advertise<visualization_msgs::Marker>("/pseudo_rob", 10);
    }
    void dyn_obs_planner::registerCallback(){
        this->odomSub_ = this->nh_.subscribe("/odom", 1000, &dyn_obs_planner::odomCB, this);
        this->clickedPointSub_ = this->nh_.subscribe("/move_base_simple/goal", 1000, &dyn_obs_planner::clickedPointCB, this);
        this->map_ = this->nh_.subscribe("/map",1000, &dyn_obs_planner::mapCallback, this);
        
        // rrt timer
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.02), &dyn_obs_planner::plannerCB, this);

		// visualization
		this->visDynObsTimer_ = this->nh_.createTimer(ros::Duration(this->DynObs_rate), &dyn_obs_planner::dynObsCB, this);
        
        if (this->Debug){
            this->visPseudoRobTimer_ = this->nh_.createTimer(ros::Duration(this->PseudoRob_rate), &dyn_obs_planner::pseudoRobCB, this);
        }
        else{
            // PID
            this->pidTimer_ = this->nh_.createTimer(ros::Duration(0.01), &dyn_obs_planner::PID, this);
        }

        
    }

    bool dyn_obs_planner::achieve_temp_goal() {

        // double goal_do_distance = sqrt(pow(this->goalPosi.first - this->pseudo_rob_pos_x, 2) + pow(this->goalPosi.second - this->pseudo_rob_pos_y, 2));
        // obs_distance_to_Rob + 
        return false;
    }

    void dyn_obs_planner::refresh_NodeTable() {
        for (int idx=0; idx<NodeTable.size(); idx++) {
            NodeTable[idx].g = INF;
            NodeTable[idx].h = INF;
            NodeTable[idx].f = INF;
            NodeTable[idx].isClosed = false;
            NodeTable[idx].isOpen = false;
        }
    }

    void dyn_obs_planner::plannerCB(const ros::TimerEvent&){
        // Only Enter the Planner After Reviceving an Odom update
        if (not this->receiveOdom_) return;


        if (this->sync_pseudoRob_flag == true){
            this->pseudo_rob_pos_x = this->odom_.pose.pose.position.x;
            this->pseudo_rob_pos_y = this->odom_.pose.pose.position.y;
            this->sync_pseudoRob_flag = false;
        }

        // ==== Reset Flags ====
        this->offset_dyn_obs = obs_radius+rob_radius;
        this->replan_range = factor_of_safeZone * offset_dyn_obs;
        this->Barrier_Radius = Barrier_Radius_Factor * offset_dyn_obs;

        // Update Flag for "this->replan_for_newGoal"
        if (this->receiveClickedPoint_) {
            this->replan_for_newGoal=true;
            // generate_newPath2Publish=true;
            refresh_NodeTable();        // reset h, g and f in NodeTable
            plan2Publish_vec_ptr = new vector<pair<double,double>>();
            residualPlan_vec_ptr = new vector<pair<double,double>>();
            rewirePlan_vec_ptr = new vector<pair<double,double>>();

            this->near_dyn_obs = false;
            this->tempGoal_is_On = false;
            this->pseudo_goal = false;

        }
        
        // Check whether I should swap to "residualPlan_vec_ptr"

        if (this->path_msg_.poses.size() == 0){
        // if (not this->has_plan_to_execute) {


            // Means: Already achieved the tempGoal via PID or pesudoRob
            if (this->tempGoal_is_On) {
                if (residualPlan_vec_ptr->size()>1) {
                    if(this->pseudo_goal){
                        // Swtich to the saved plan!

                        // updatePathVisVec(*residualPlan_vec_ptr);                        
                        this->pathMsgConverter(*residualPlan_vec_ptr, this->path_msg_);


                        // this->pathVisMsg_.markers = this->pathVisVec_;
                        // this->visPathPub_.publish(this->pathVisMsg_);
                        // this->has_plan_to_execute = true;
                        
                        this->tempGoal_is_On = false;

                        // USEFUL TRICK!!!
                        int bias = MIN(rewirePlan_vec_ptr->size(), this->path_msg_.poses.size() -1);

                        this->pseudo_loop = 0 + bias;
                        this->PID_path_ref_index = 1 + bias;
                        // DO NOT RESET RESIDUALPLAN

                        this->pseudo_goal = false;
                    }
                }
                else printf("THERE MIGHT BE AN ISSUE OR NOT READY FOR INCREMENTAL SEARCH!!!");
                
                this->tempGoal_is_On = false;  // reset flag

            }
        }

        if (this->replan_for_newGoal | this->near_dyn_obs) printf("Good TO Continue\n");
        else return;

        // Prepare "curPosi", "goalPosi"
        if (this->Debug) curPosi = make_pair(this->pseudo_rob_pos_x, this->pseudo_rob_pos_y);
        else curPosi = make_pair(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y);
        goalPosi = make_pair(this->goal_.pose.position.x, this->goal_.pose.position.y);


        if (this->replan_for_newGoal) {
            AStarBaseline(plan2Publish_vec_ptr, curPosi, goalPosi);
            // this->has_plan_to_execute = true;
            updatePathVisVec(*plan2Publish_vec_ptr);
            this->pathMsgConverter(*plan2Publish_vec_ptr, this->path_msg_);
            this->pathVisMsg_.markers = this->pathVisVec_;
            this->visPathPub_.publish(this->pathVisMsg_);
            this->pseudo_loop = 0;
            this->PID_path_ref_index = 1; 
            this->replan_for_newGoal = false;     // Update flags
        }
        else if (this->near_dyn_obs) {

            refresh_NodeTable();

            if (this->Debug) this->obs_distance_to_Rob = sqrt(pow(this->obs_pos_x - this->pseudo_rob_pos_x, 2) + pow(this->obs_pos_y - this->pseudo_rob_pos_y, 2));
            else this->obs_distance_to_Rob = sqrt(pow(this->obs_pos_x - this->odom_.pose.pose.position.x, 2) + pow(this->obs_pos_y - this->odom_.pose.pose.position.y, 2));
            printf("obs_distance_to_Rob-Barrier_Radius=%g\n", obs_distance_to_Rob-Barrier_Radius);
            
            switch (REPLAN_STRATEG_Indicator)
            {
                case do_ASTAR_Total_Replan: {
                    // ONLY setup a knowledged Barrier, then back to baseline AStar!
                    Lable_avoidZone();

                    tempGoalPosi = goalPosi;
                    AStarBaseline(rewirePlan_vec_ptr, curPosi, tempGoalPosi);
                    UN_Lable_avoidZone();

                    printf("Publish Rewire Path!\n");
                    // this->has_plan_to_execute = true;
                    updateReplanPathVisVec(*rewirePlan_vec_ptr);
                    this->pathMsgConverter(*rewirePlan_vec_ptr, this->path_msg_);
                    this->pathReplanVisMsg_.markers = this->ReplanpathVisVec_;
                    this->visReplanPathPub_.publish(this->pathReplanVisMsg_);
                    this->pseudo_loop = 0;
                    this->PID_path_ref_index = 1;
                    break;
                }
                case do_ASTAR_Partial_Replan: {
                    this->tempGoal_is_On = true;
                    // ONLY setup a knowledged Barrier, then back to baseline AStar!
                    Lable_avoidZone();
                    residualPlan_vec_ptr->clear();
                    find_a_TempGoal();  // Assigned tempGoalPosi inside of the function!!!
                    AStarBaseline(rewirePlan_vec_ptr, curPosi, tempGoalPosi);
                    UN_Lable_avoidZone();

                    printf("Publish Rewire Path!\n");
                    // this->has_plan_to_execute = true;
                    updateReplanPathVisVec(*rewirePlan_vec_ptr);
                    this->pathMsgConverter(*rewirePlan_vec_ptr, this->path_msg_);
                    this->pathReplanVisMsg_.markers = this->ReplanpathVisVec_;
                    this->visReplanPathPub_.publish(this->pathReplanVisMsg_);
                    this->pseudo_loop = 0;
                    this->PID_path_ref_index = 1;

                    break;
                }
                case do_Incremental_Search: {
                    printf("Will Handle Incremental_Search later.");
                    break;
                }
                default: {
                    printf("Should Never Be There!");
                    break;
                }
            }

            // === Reset Flag ===
            this->near_dyn_obs = false;
            printf("==== near_dyn_obs is OFF by Rewiring ====\n");
        }
        
        this->receiveClickedPoint_ = false;     // Reset Flag
    }

    void dyn_obs_planner::find_a_TempGoal() {
        // plan2Publish_vec_ptr
        int last_index = -1;
        int interested_IDX = -1;

        pair<double,double> bufferPair;

        // Find the last pair that satisfies (pair.first+pair.second<1)
        for (int i = plan2Publish_vec_ptr->size() - 1; i >= 0; i--) {
            bufferPair = (*plan2Publish_vec_ptr)[i];
            if (NodeTable[GETMAPINDEX(bufferPair.first, bufferPair.second,this->x_size)].is_Barrier_Obs) {
                last_index = i;
                break;
            }
        }

        // If a valid last index was found, assign the value to tempGoalPosi
        if (last_index >= 0) {
            interested_IDX = MIN(last_index+5, (plan2Publish_vec_ptr->size() - 1));

            bufferPair = (*plan2Publish_vec_ptr)[interested_IDX];
            if (get_distance(bufferPair, goalPosi) < get_distance(tempGoalPosi, goalPosi)) {
                tempGoalPosi = bufferPair;
            }
            
        } else {
            printf("No Valid tempGoalPosi being found!! Use Ultimate Goal Directly\n");
            tempGoalPosi = goalPosi;
            return;
        }

        // Create a new vector to store all items from tempGoalPosi to the end of plan2Publish_vec_ptr
        if (interested_IDX >= 0) {
            for (int i = interested_IDX; i < plan2Publish_vec_ptr->size(); i++) {
                residualPlan_vec_ptr->push_back((*plan2Publish_vec_ptr)[i]);
            }
        }
        printf("Size of residualPlan_vec_ptr=%d\n", (int)residualPlan_vec_ptr->size());
    }

    void dyn_obs_planner::Lable_avoidZone() {
        // Determine the Orientation -- useful for semiCircle
        Barrier_Orientation = 0;    // No matter for a circle

        printf("Barrier_Radius=%g\n", Barrier_Radius);

        printf("obs_distance_to_Rob-Barrier_Radius=%g\n", obs_distance_to_Rob-Barrier_Radius);

        // Generate points along the circle
        double temp_x, temp_y;
        double dist_1, dist_2;

        if (this->Debug) curPosi = make_pair(this->pseudo_rob_pos_x, this->pseudo_rob_pos_y);
        else curPosi = make_pair(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y);

        dist_2 = get_distance(make_pair(obs_pos_x, obs_pos_y), tempGoalPosi);

        for (double r=0; r<=Barrier_Radius; r=r+0.01) {
            for (int i = 0; i<=Barrier_Resolution; i++) {
                Barrier_Angle = i * 2.0 * M_PI / Barrier_Resolution;
                temp_x = obs_pos_x + r * cos(Barrier_Angle + Barrier_Orientation);
                temp_y = obs_pos_y + r * sin(Barrier_Angle + Barrier_Orientation);

                dist_1 = get_distance(make_pair(temp_x, temp_y), tempGoalPosi);

                if (dist_2 < dist_1) {
                    this->Barrier_pair_vec.push_back(make_pair(temp_x, temp_y));
                }

                // this->Barrier_pair_vec.push_back(make_pair(temp_x+STEP_SIZE, temp_y+STEP_SIZE));
                // this->Barrier_pair_vec.push_back(make_pair(temp_x-STEP_SIZE, temp_y-STEP_SIZE));
                // this->Barrier_pair_vec.push_back(make_pair(temp_x-STEP_SIZE, temp_y+STEP_SIZE));
                // this->Barrier_pair_vec.push_back(make_pair(temp_x+STEP_SIZE, temp_y-STEP_SIZE));
            }
        }

        // for (double x= (this->obs_pos_x - 0.5); x<(this->obs_pos_x + 0.5); x=x+0.01) {
        //     for (double y= (this->obs_pos_y - 0.5); y<(this->obs_pos_y + 0.5); y=y+0.01) {
        //         this->Barrier_pair_vec.push_back(make_pair(x, y));
        //     }
        // }
        

        // Label all
        for (auto p: this->Barrier_pair_vec) {
            NodeTable[GETMAPINDEX(p.first, p.second,this->x_size)].is_Barrier_Obs = true;
        }

    }

    double dyn_obs_planner::get_distance(pair<double, double> p1, pair<double, double> p2) {
        return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
    }


    void dyn_obs_planner::UN_Lable_avoidZone() {
        // UN Label all
        for (auto p: this->Barrier_pair_vec) {
            NodeTable[GETMAPINDEX(p.first, p.second,this->x_size)].is_Barrier_Obs = false;
        }
        this->Barrier_pair_vec.clear();
    }



    void dyn_obs_planner::AStarBaseline(vector<pair<double,double>>* plan2Publish_pair_vec,
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
                    // Skip for Barrier_Obs
                    if (nodePtr->is_Barrier_Obs) continue;

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
        // Clear the 
        OPEN.clear();
        CLOSED.clear();

        // No Valid Path can be found
        if (!goalNodePtr->isClosed) {
            printf("[No Valid Path can be found] Goal is within a nonTraversable Zone!\n");
            printf("Wait for next valid Path to be found! \n");
            plan2Publish_pair_vec->clear();
            return;
        }

        // Have Valid Path
        nodePtr = goalNodePtr;
        // startNodePtr = &NodeTable[GETMAPINDEX(curPosi.first,curPosi.second,x_size)];
        list<pair<double,double>> plan;
        while (nodePtr->prevNodePtr != startNodePtr) {
            plan.push_back(make_pair(nodePtr->X, nodePtr->Y));
            nodePtr = nodePtr->prevNodePtr;
        }
        plan.push_back(make_pair(nodePtr->X, nodePtr->Y));
        plan.reverse();

        // vector<pair<double,double>> plan2Publish_pair_vec;
        plan2Publish_pair_vec->clear();

        for (auto pair: plan) {
            plan2Publish_pair_vec->push_back(pair);
            // Print out the plan
            //printf("Move to x=%g, y=%g\n", pair.first, pair.second);
        }
        
        // === A Star END ===
    }
    void dyn_obs_planner::pseudoRobCB(const ros::TimerEvent&){
        // Only Enter the Planner After Reviceving an Odom update
        if (not this->receiveOdom_) return;
        // if (not this->has_plan_to_execute) return;
        if (this->replan_for_newGoal | this->near_dyn_obs) return;

        if (this->Debug) curPosi = make_pair(this->pseudo_rob_pos_x, this->pseudo_rob_pos_y);
        else curPosi = make_pair(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y);

        this->obs_distance_to_Rob = get_distance(make_pair(obs_pos_x, obs_pos_y), curPosi);

        if (this->obs_distance_to_Rob <= this->replan_range) {
            double rob_2_goal = get_distance(curPosi, goalPosi);
            double obs_2_goal = get_distance(make_pair(obs_pos_x, obs_pos_y), goalPosi);

            if (rob_2_goal > obs_2_goal) {
                this->near_dyn_obs = true;
            }            
        }

        // Update pseudo_rob_ positoin and loop_indicator.
		if (this->path_msg_.poses.size() != 0){
            if (this->pseudo_loop == 0){
                this->pseudoRobMarker_.type = this->shape;
                this->pseudoRobMarker_.header.frame_id = "map";
                this->pseudoRobMarker_.scale.x = 0.3; 
                this->pseudoRobMarker_.scale.y = 0.3; 
                this->pseudoRobMarker_.scale.z = 0.3;
                this->pseudoRobMarker_.color.r = 1.0f;
                this->pseudoRobMarker_.color.g = 0.0f;
                this->pseudoRobMarker_.color.b = 0.0f;
                this->pseudoRobMarker_.color.a = 1.0;
            }
            // get current (pseudo_rob_pos_x, pseudo_rob_pos_y)
            this->pseudo_rob_pos_x = path_msg_.poses[this->pseudo_loop].pose.position.x;
            this->pseudo_rob_pos_y = path_msg_.poses[this->pseudo_loop].pose.position.y;
            // Increment the pseudo_loop by 1
            this->pseudo_loop++;
            // Change the position of pseudoRobMarker_ to follow the pseudo_rob_
            this->pseudoRobMarker_.pose.position.x = this->pseudo_rob_pos_x;
            this->pseudoRobMarker_.pose.position.y = this->pseudo_rob_pos_y;
            this->pseudoRobMarker_.pose.position.z = 0.15;
        }

        // Termination check for pseudo_rob_

        if (this->path_msg_.poses.size() ==  this->pseudo_loop){
            nav_msgs::Path emptyPath;
			this->path_msg_ = emptyPath;
            this->pseudo_loop = 0;
            // this->has_plan_to_execute = false;
            this->pseudo_goal = true;
        }
        this->PseudoRobotPub_.publish(this->pseudoRobMarker_);
    }


    void dyn_obs_planner::dynObsCB(const ros::TimerEvent&){
        // Only enter dynObsCB after param_flag=true -- After initialization!
        if (! this->param_flag) return;

        // Based on the "obs_loop", either update or init dynObs.
        if (this->obs_loop !=0 ) {
            // // remove its history effect from NodeTable
            // for (double x= (this->obs_pos_x - this->offset_dyn_obs); x<(this->obs_pos_x + this->offset_dyn_obs); x+=STEP_SIZE) {
            //     for (double y= (this->obs_pos_y - this->offset_dyn_obs); y<(this->obs_pos_y + this->offset_dyn_obs); y+=STEP_SIZE) {
            //         // Remove obstacle from NodeTable
            //         NodeTable[GETMAPINDEX(x,y, x_size)].is_Dynamic_Obs = false;
            //     }
            // }

            // Update obs position
            //this->obs_pos_x = (this->obs_pos_x)+this->move[randomInt(0,2)];
            //this->obs_pos_y = (this->obs_pos_y)+this->move[randomInt(0,2)];
            if (this->obs_pos_y > 5.0){
                this->move_direction = true;
            }
            if (this->obs_pos_y < 0.5){
                this->move_direction = false;
            }
            if(this->move_direction){
                this->obs_pos_y = (this->obs_pos_y)-0.1;
            }
            if(!this->move_direction){
                this->obs_pos_y = (this->obs_pos_y)+0.1;
            }
        }
        else {
            // Initialize the dynObs
            this->obs_pos_x = this->obs_initial_x;
            this->obs_pos_y = this->obs_initial_y;
            this->obs_x_upperBound = this->obs_movable_range + this->obs_initial_x;
            this->obs_y_upperBound = this->obs_movable_range + this->obs_initial_y;
            this->obs_x_lowerBound = -1*this->obs_movable_range + this->obs_initial_x;
            this->obs_y_lowerBound = -1*this->obs_movable_range + this->obs_initial_y;
            
            // One-Time Initialization
            this->marker_.type = this->shape;
            this->marker_.header.frame_id = "map";
            this->marker_.scale.x = this->obs_marker_factor*this->obs_radius; // 0.3
            this->marker_.scale.y = this->obs_marker_factor*this->obs_radius; // 0.3
            this->marker_.scale.z = 1.0;
            this->marker_.color.r = 0.0f;
            this->marker_.color.g = 1.0f;
            this->marker_.color.b = 0.0f;
            this->marker_.color.a = 1.0;      
            
        }
    
        // Increment the obs_loop indicator by 1
        this->obs_loop++;

        // Bound the obs_pos
        if (this->obs_pos_x > this->obs_x_upperBound){
            this->obs_pos_x = this->obs_x_upperBound;
        }
        if (this->obs_pos_x < this->obs_x_lowerBound){
            this->obs_pos_x = this->obs_x_lowerBound;
        }
        if (this->obs_pos_y > this->obs_y_upperBound){
            this->obs_pos_y = this->obs_y_upperBound;
        }
        if (this->obs_pos_y < this->obs_y_lowerBound){
            this->obs_pos_y = this->obs_y_lowerBound;
        }



        // Publish obs_pos to marker_
        this->marker_.pose.position.x = this->obs_pos_x;
        this->marker_.pose.position.y = this->obs_pos_y;
        this->marker_.pose.position.z = 0.5;
        this->ObsPub_.publish(this->marker_);

        if(this->obs_pos_y > 2.9 && this->pseudo_click){
            this->goal_.pose.position.x = -1;
            this->goal_.pose.position.y  = 4;
            this->goal_.pose.position.z  = 0.2; // set height to be 0.3 m
            this->receiveClickedPoint_ = true;
        }

        // // Update NodeTable
        // for (double x= (this->obs_pos_x - this->offset_dyn_obs); x<(this->obs_pos_x + this->offset_dyn_obs); x+=STEP_SIZE) {
        //     for (double y= (this->obs_pos_y - this->offset_dyn_obs); y<(this->obs_pos_y + this->offset_dyn_obs); y+=STEP_SIZE) {
        //         // Add obstacle to NodeTable
        //         NodeTable[GETMAPINDEX(x,y, x_size)].is_Dynamic_Obs = true;
        //     }
        // }
        

    }
    void dyn_obs_planner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        std_msgs::Header header = msg->header;
        nav_msgs::MapMetaData info = msg->info;
        this->cost_map = msg->data;
        int width = info.width;
        int height = info.height;
        this->x_size = height;
        ROS_INFO("Got map %d %d", info.width, info.height);

        init_NoteTable();
        ROS_INFO("Build the NodeTable Done");

        
    }

    void dyn_obs_planner::init_NoteTable(){
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
                if (is_static_obs or is_unexplored) {
                    NodeTable[GETMAPINDEX(x,y, x_size)].is_Static_Obs = true;
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
        // if (not this->has_plan_to_execute) return;
        if (this->replan_for_newGoal | this->near_dyn_obs) return;

        if (this->Debug) curPosi = make_pair(this->pseudo_rob_pos_x, this->pseudo_rob_pos_y);
        else curPosi = make_pair(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y);

        this->obs_distance_to_Rob = get_distance(make_pair(obs_pos_x, obs_pos_y), curPosi);

        if (this->obs_distance_to_Rob <= this->replan_range) {
            double rob_2_goal = get_distance(curPosi, goalPosi);
            double obs_2_goal = get_distance(make_pair(obs_pos_x, obs_pos_y), goalPosi);

            if (rob_2_goal > obs_2_goal) {
                this->near_dyn_obs = true;
            }            
        }


		//cout<<"PID CB" <<endl;
        //double fixed_time=;
		int endIdx=path_msg_.poses.size();
        //cout<<endIdx<<endl;

        //printf("path_msg_.poses.size() = %d, with PID_path_ref_index=%d, cmd=%d\n", endIdx, PID_path_ref_index, cmd);

        this->obs_distance_to_Rob = sqrt(pow(this->obs_pos_x - this->odom_.pose.pose.position.x, 2) + pow(this->obs_pos_y - this->odom_.pose.pose.position.y, 2));
        if (this->obs_distance_to_Rob <= this->replan_range) {
            this->near_dyn_obs = true;
        }
		if (this->path_msg_.poses.size() != 0){
		// if (endIdx != 0) {
			// cmd 1 = angular turning, 2 = linear forward
			// collect data
            if (this->PID_path_ref_index > endIdx){
                this->PID_path_ref_index = endIdx;
            }
            //cout<<this->PID_path_ref_index<<endl;
			double target_x = path_msg_.poses[this->PID_path_ref_index].pose.position.x;
			double target_y = path_msg_.poses[this->PID_path_ref_index].pose.position.y;
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
			cmd = 1;
            // [POTENTIAL BUG] Add 
			if (abs(error_dis) <= 0.1){
				this->PID_path_ref_index += 1;
				this->PID_path_ref_index =std::min(this->PID_path_ref_index, int(path_msg_.poses.size())-1);
				cmd = 1;
			}
			else if (abs(error_angle) <= 0.08){
				cmd  = 2;
                // [Que] No need to add this line???
                // this->PID_path_ref_index +=1;
			}

            // printf("path_msg_.poses.size() = %d, with PID_path_ref_index=%d, cmd=%d, error_dis=%f, error_angle=%f\n", endIdx, PID_path_ref_index, cmd, error_dis, error_angle);


			// apply control

			// prev_error_angle=error_angle;
			// prev_error_dis  =error_dis;
			double output_angle_vel = (this->Kp_a_*error_angle) + (this->Kd_a_*current_vel_ang);
			double output_linear_vel = (this->Kp_d_*error_dis) + (this->Kd_d_*current_vel_x);

			//cout<< "output_angle_vel = " << output_angle_vel << endl;

			
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
			//std::cout << this->PID_path_ref_index <<": waypoint"<< endl;
			//std::cout << this->path_msg_poses.size()  <<": waypoint size"<< endl;

			// noice <=0.1
			if (noice2 <=0.2 && noice1 <=0.2 && PID_path_ref_index == endIdx-1 )
			{
				std::cout << "goal achieved " << std::endl;
				cmd = 1;
				// this->receiveClickedPoint_ = false;
				Twist.angular.z=0;
				Twist.linear.x=0;
				this->cmdvelPub_.publish(Twist);
				nav_msgs::Path emptyPath;
				this->path_msg_ = emptyPath;

                this->PID_path_ref_index = 1;
                // this->has_plan_to_execute = false;
                this->pseudo_goal = true;

                // this->has_plan_to_execute = false;
			}

		}
	}

    
    void dyn_obs_planner::run(){
        // register timer callback
        this->registerCallback();
	}

}