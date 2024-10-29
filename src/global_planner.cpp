#include <planners_pkg/global_planner.hpp>


GlobalPlanner::GlobalPlanner(std::string node_name_)
{
    //The tf buffer is used to lookup the base link position(tf from world frame to robot base frame)
    node_name = node_name_;
    nh.reset(new ros::NodeHandle("~"));

    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));

    std::string node_name_grid_ = "grid3D_node";
	grid_3D = new Grid3d(node_name_grid_);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "Initialazing Trilinear Interpolation (grid3D) in Global Planner");
	grid_3D->computeTrilinearInterpolation();
    ROS_INFO_COND(showConfig, PRINTF_GREEN "Finished Trilinear Interpolation (grid3D) in Global Planner");
	rrt_star_planner = new rrtStarPlanner();

    configParams();
    configTopics();
    configServices();
    configRRTStar();
}

//This function gets parameter from param server at startup if they exists, if not it passes default values
void GlobalPlanner::configParams()
{
    //At startup, no goal and no costmap received yet
    seq = 0;
    //Get params from param server. If they dont exist give variables default values
    nh->param("show_config", showConfig, (bool)0);
    nh->param("planner_type", planner_type, (std::string)"rrt_star");

    nh->param("ws_x_max", ws_x_max, (double)30);
    nh->param("ws_y_max", ws_y_max, (double)30);
    nh->param("ws_z_max", ws_z_max, (double)30);
    nh->param("ws_x_min", ws_x_min, (double)0);
    nh->param("ws_y_min", ws_y_min, (double)0);
    nh->param("ws_z_min", ws_z_min, (double)0);

    nh->param("n_iter", n_iter, (int)100);
    nh->param("n_loop", n_loop, (int)1);
    nh->param("world_frame", world_frame, (std::string) "/map");
    nh->param("robot_frame", robot_frame, (std::string) "ugv_base_link");
    nh->param("radius_near_nodes", radius_near_nodes, (double)1.0);
    nh->param("step_steer", step_steer, (double)0.5);
    nh->param("goal_gap_m", goal_gap_m, (double)0.2);

    nh->param("distance_obstacle_robot", distance_obstacle_robot, (double)1.0);

	nh->param("w_nearest_ugv", w_nearest_ugv, (double)1.0);
	nh->param("w_nearest_smooth", w_nearest_smooth, (double)1.0);

	nh->param("path", path, (std::string) "~/");
	nh->param("map_file", map_file, (std::string) "my_map");

    nh->param("sample_mode", sample_mode, (int)1); // value 1 or 2
    nh->param("samp_goal_rate", samp_goal_rate, (int)10);
    
    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D Node Configuration:");
    ROS_INFO_COND(showConfig, PRINTF_GREEN "   Workspace = X: [%.2f, %.2f]\t Y: [%.2f, %.2f]\t Z: [%.2f, %.2f]  ", ws_x_max, ws_x_min, ws_y_max, ws_y_min, ws_z_max, ws_z_min);
    ROS_INFO_COND(showConfig, PRINTF_GREEN "   World frame: %s, UGV base frame: %s ", world_frame.c_str(), robot_frame.c_str());
}

void GlobalPlanner::configRRTStar()
{
    rrt_star_planner->init(ws_x_max, ws_y_max, ws_z_max, ws_x_min, ws_y_min, ws_z_min, world_frame, goal_gap_m, 
                         distance_obstacle_robot, grid_3D, map_file, path, nh);
    configRandomPlanner();
}

void GlobalPlanner::configTopics()
{
    point_cloud_map_trav_sub_ = nh->subscribe( "/region_growing_traversability_pc_map", 1,  &GlobalPlanner::readPointCloudTraversabilityMapCallback, this);
    point_cloud_map_ugv_sub_ = nh->subscribe( "/region_growing_obstacles_pc_map", 1,  &GlobalPlanner::readPointCloudUGVObstaclesMapCallback, this);
    planner_server_ = lnh_.advertiseService("get_algorithm", &GlobalPlanner::requestPathService, this);

    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D Topics and Subscriber Configurated");
}

void GlobalPlanner::configServices()
{
    ROS_INFO_COND(showConfig, PRINTF_GREEN "Global Planner 3D: Action client from global planner ready");
}

void GlobalPlanner::readPointCloudTraversabilityMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    rrt_star_planner->readPointCloudTraversabilityMapUGV(msg);
    ROS_INFO_COND(true, PRINTF_GREEN "Global Planner: UGV Traversability Map Navigation Received");
}

void GlobalPlanner::readPointCloudUGVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    rrt_star_planner->readPointCloudMapForUGV(msg);
    pc_obs_ugv = msg;
    ROS_INFO_COND(true, PRINTF_GREEN "Global Planner: UGV Obstacles Map Navigation Received");
}

bool GlobalPlanner::requestPathService(planners_pkg::SetPathRequest &_req, planners_pkg::SetPathResponse &_rep)
{
    bool ret = false;
    ROS_INFO_COND(true, PRINTF_GREEN "Global Planner: Service Received");

    if (isPointFeasible(_req.start,"Start") && isPointFeasible(_req.goal, "Goal"))
    {
        ROS_INFO_COND(true, PRINTF_GREEN "Global Planner: Goal and start successfull set:");
        //Print succes start and goal points
        start_point = _req.start;
        goal_point = _req.goal;
        ROS_INFO_COND(true, PRINTF_GREEN "\t\t StartPoint[%f %f %f ] GoalPoint[%f %f %f]",
                    start_point.x,start_point.y, start_point.z, goal_point.x, goal_point.y, goal_point.z);
        rrt_star_planner->setStartAndGoal(start_point, goal_point);
        // Path calculation
        ROS_INFO(PRINTF_YELLOW "Global Planner: Time Spent in Global Path Calculation: %.1f ms", milliseconds + seconds * 1000);
        number_of_points = rrt_star_planner->computePath();
        if (number_of_points > 0){ 
            ROS_INFO(PRINTF_GREEN "Global Planner: Number of points in path: %d", number_of_points);
            // std::list<RRTNode*> rrt_path = rrt_star_planner->getPath();
            ret = true;
        }
        else
            ROS_INFO(PRINTF_YELLOW "Global Planner: Number of points in path: %d", number_of_points);

    }
    return ret;
}

bool GlobalPlanner::isPointFeasible(geometry_msgs::Point _p, std::string _s)
{
	statusPointAnalisys status_point;
    status_point.initGrid(grid_3D);
    if(ws_x_max > _p.x && ws_x_min < _p.x && ws_y_max > _p.y && ws_y_min < _p.y && ws_z_max > _p.z && ws_z_min < _p.z){
        if (distance_obstacle_robot < status_point.distanceToObstacles(_p)){
            return true;
        }
        else{
            ROS_ERROR("Point %s : close to obstacle d=%f", _s.c_str(), status_point.distanceToObstacles(_p));
            return false;
        }
    }    
    else{
        ROS_ERROR("Point %s : Out of WorkSpace", _s.c_str());
        return false;
    }
}

void GlobalPlanner::configRandomPlanner()
{
    rrt_star_planner->configRRTParameters( n_iter, n_loop, radius_near_nodes, step_steer, samp_goal_rate, w_nearest_ugv ,w_nearest_smooth);
}

// namespace PathPlannersgeometry_msgs::Vector3
