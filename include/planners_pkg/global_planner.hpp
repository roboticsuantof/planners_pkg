/*
Simon Martinez Rozas, 2021 UPO

Global Planner Class using RANDOM Algorithms (RRT, RRT*, biRRT)
*/
#ifndef _GLOBALPLANNER__HPP__
#define _GLOBALPLANNER__HPP__

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <memory>
#include <planners_pkg/rrt_star.hpp>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/Marker.h>

#include "tf2/transform_datatypes.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//Dynamic reconfigure auto generated libraries
#include <dynamic_reconfigure/server.h>

#include <time.h>
#include <sys/timeb.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <planners_pkg/SetPath.h>
#include <planners_pkg/rrt_star.hpp>

    class GlobalPlanner 
    {
        public:
            //Default constructor
            GlobalPlanner(std::string node_name);

            /**
                Default destructor
            **/
            // ~GlobalPlanner();

            /*
            @brief: This is the main function that should be executed in loop by the node
            */
            void plan();
            
        private:
            // bool replan();
            void replan();
            
            /*
            @brief: Loads parameters from ros param server, if they are not present, load defaults ones
                    It also configure markers and global map geometry 
            */
            void configParams();
            /*
            @brief: Load topics names from param server and if they are not present, set defaults topics names for 
                    Subscribers and publishers
            */
            void configTopics();
            /*
            @brief: Config thetastar class parameters
            */
            void configRRTStar();
            /*
            @brief: It declares the two service servers
            */
            void configServices();

            /*
            @brief: 
            */
            void readPointCloudTraversabilityMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            void readPointCloudUGVObstaclesMapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            bool requestPathService(planners_pkg::SetPathRequest &_req, planners_pkg::SetPathResponse &_rep);

            /*
            @brief: These functions tries to pass the start and goal positions to the thetastar object
                    They return true if the points are not occupied
            */
            bool isPointFeasible(geometry_msgs::Point _p, std::string _s);

            /*
            @brief: Get tf reel tether to compute catenary.
            */
            void configRandomPlanner();

            /*              Class Variables                 */
            ros::NodeHandlePtr nh;
            ros::NodeHandle lnh_{"~"};

            geometry_msgs::Point start_point, goal_point;

            //Publishers and Subscribers
            ros::Subscriber point_cloud_map_ugv_sub_, point_cloud_map_trav_sub_;

            ros::ServiceServer planner_server_;

            //Services servers
            ros::ServiceServer global_replanning_service, reset_global_costmap_service, plan_request_service;
            ros::ServiceClient recovery_rot_srv_client;

            //tf buffer used to get the base_link position on the map(i.e. tf base_link-map)
            std::shared_ptr<tf2_ros::Buffer> tfBuffer;
            std::unique_ptr<tf2_ros::TransformListener> tf2_list;

            std::unique_ptr<tf::TransformListener> tf_list_ptr;

            std_msgs::Bool flg_replan_status;

            std::string robot_frame, world_frame, node_name;

            //Output variables
            int number_of_points;
            int seq;

            // Class declaration
	        Grid3d *grid_3D;
            rrtStarPlanner *rrt_star_planner;

            //These two flags can be configured as parameters
            bool showConfig;

	        std::string path, map_file;

            //Variables to fill up the feedback 
            struct timespec start, finish;
            float seconds, milliseconds;
            ros::Time start_time;

            //! 3D specific variables
            bool use_distance_function; //Only related with tether and UAV distance
            sensor_msgs::PointCloud2::ConstPtr pc_obs_ugv;

            octomap_msgs::OctomapConstPtr map;

            double ws_x_max, ws_y_max, ws_z_max;
            double ws_x_min, ws_y_min, ws_z_min;

            double radius_near_nodes, step_steer;
            int n_iter, n_loop, samp_goal_rate;
            double goal_gap_m;
            double distance_obstacle_robot; //Safe distance to obstacle to accept a point valid for UGV and UAV
            int sample_mode; // 0: random sample for UGV and UAV , 1: random sample only for UAV  
            double w_nearest_ugv ,w_nearest_uav ,w_nearest_smooth;

            double min_distance_add_new_point;

            std::string planner_type;

    }; //class GlobalPlanner


#endif