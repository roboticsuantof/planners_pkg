#ifndef _RRT_STAR_PLANNER_HPP__
#define _RRT_STAR_PLANNER_HPP__

#include <math.h>
#include <random>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <octomap_msgs/Octomap.h> //Octomap Binary
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <Eigen/StdVector>

#include "planners_pkg/RRTNode.h"
#include "misc/n_kdtree.hpp"
#include "misc/grid3d.hpp"
#include "misc/near_neighbor.hpp"
#include "misc/status_point_manager.hpp"
#include "misc/planner_graph_markers.hpp"

#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"
#define PRINTF_ORANGE  "\x1B[38;2;255;128;0m"
#define PRINTF_ROSE    "\x1B[38;2;255;151;203m"
#define PRINTF_LBLUE   "\x1B[38;2;53;149;240m"
#define PRINTF_LGREEN  "\x1B[38;2;17;245;120m"
#define PRINTF_GRAY    "\x1B[38;2;176;174;174m"

typedef geometry_msgs::Vector3 Vector3;
typedef geometry_msgs::Quaternion Quaternion;

//*****************************************************************
// 				rrtStarPlanner Algoritm Class Declaration
//*****************************************************************

class rrtStarPlanner
{
public:
	rrtStarPlanner();

	/**
		  Constructor with arguments
		   @param planner name for topic names 
		   @param frame_id for debug markers 
		   @param simetric or asimetric workspace centered at (0,0,0) [meters]
		   @param occupancy matrix resolution [meters]
		   @param occupancy matrix nodes inflation (horizontal and vertical, real + safety) [meters]
		   @param Lazy Theta* with Optimization: goal point factor [0 to inf]. Bigger -> distance to target more weight than distance to origin -> minor exploration -> shorter runtime, grater path length
		   @param Lazy Theta* weighted: Z axis weight cost [0 to inf]. 0 to 1 for Z priority exploration, 1 for symetric exploration and inf(~100) to not explore in Z.
		   @param Lazy Theta* bounded: Minimum Z that will be inflated vertically 
		   @param NodeHandle 
		**/
	void init(float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_,
				   std::string frame_id_, double goal_gap_m_, double distance_obstacle_ugv_, Grid3d *grid3D_, std::string map_file_, std::string path_, ros::NodeHandlePtr nh_);

  	~rrtStarPlanner();
  
  	virtual int computePath();      

  	std::list<RRTNode*> nodes_tree; // TODO: single tree planners

	std::list<RRTNode*> rrt_path;

  	virtual void clearStatus();

	/**
		  Publish via topic the discrete map constructed
		**/
	// virtual void publishOccupationMarkersMap();
	
	void configRRTParameters(int n_iter_ , int n_loop_, double r_nn_, double s_s_, int s_g_r_, double w_n_ugv_, double w_n_smooth_);
	/** 
	   Receive segmented PointCloud2 for UGV traversability
	**/
	void readPointCloudTraversabilityMapUGV(const sensor_msgs::PointCloud2::ConstPtr& msg);
	/** 
	   Receive PointCloud2 from map to get UGV obstacles
	**/
	void readPointCloudMapForUGV(const sensor_msgs::PointCloud2::ConstPtr& msg);
	/** 
	   Set the point start and goal to compute the path
	**/
	void setStartAndGoal(geometry_msgs::Point s_, geometry_msgs::Point g_);
  	/** 
	   Save the path in a RRTNode list
	**/
	std::list<RRTNode*> getPath();
	/** 
	   Compute an unic ID for a point
	**/
	int getIdentificationNumber(const int x_, const int y_, const int z_){
		int px_ = round(x_ * 100);
		int py_ = round(y_ * 100);
		int pz_ = round(z_ * 100);
		// Sign codification
		int Lx_ = ws_x_max - ws_x_min + 1;
		int Ly_ = ws_y_max - ws_y_min + 1;
		// Generate ID
		int ID = (int)((px_ - ws_x_min) + (Lx_) * ((py_ - ws_y_min) + (Ly_) * (pz_ - ws_z_min)));
		return ID;
	}

	/** Variables **/
	RRTNode *pose_initial, *pose_final; 
	RRTNode *pos_goal; // That node is fill it by the node that approach the goal in Independent configuration
	
	pointVec v_nodes_kdtree;

	double w_nearest_pos ,w_nearest_rot;
	NearNeighbor nn_trav_ugv, nn_obs_ugv, nn_obs_uav;
	Grid3d *grid_3D;
	statusPointAnalisys *status_point;
	PlannerGraphMarkers graphM;
	
	std::string node_name, path_name;
	std::string map_file;
	std::string path;

	ros::NodeHandlePtr nh; // Pointer to the process NodeHandle to publish topics


protected:
	
	bool getRandomNode(RRTNode &q_rand_, bool go_to_goal_ = false); 
	bool extendGraph(const RRTNode q_rand_);
	RRTNode* getNearestNode(const RRTNode q_rand_);
  	bool steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, RRTNode &q_new_);
	void getOrientation(RRTNode &n_ , const RRTNode p_);
	bool checkNodeFeasibility(const RRTNode &p_);
	void updateKdtreeNode(const RRTNode ukT_);
	void getParamsNode(RRTNode &node_, bool is_init_= false);
	void updateParamsNode(RRTNode &node_);
	void saveNode(RRTNode* sn_, bool is_init_=false);
	double costNode(const RRTNode q_new_);
	double costBetweenNodes(const RRTNode q_near_, const RRTNode q_new_);
  	
	bool isGoal(const RRTNode st_);

	std::vector<int> getNearNodes(const RRTNode &q_new_, float radius_);

	/** Variables **/
	int num_goal_finded;

	octomap::OcTree *map;
	std::vector<geometry_msgs::Vector3> v_points_ws_ugv;

    sensor_msgs::PointCloud2::ConstPtr pc_obs_ugv;

	double goal_gap_m;

	// Max time to get path
	int timeout;

	int count_iter, count_total_loop; // count times that fail get a new q_new;
	int n_iter, n_loop;
	int ws_x_max, ws_y_max, ws_z_max; // WorkSpace lenghts from origin (0,0,0)
	int ws_x_min, ws_y_min, ws_z_min;

	std::string frame_id, planner_type;
	double length_tether_max, radius_near_nodes, step_steer;
	int samp_goal_rate;
    double distance_obstacle_ugv; //Safe distance to obstacle to accept a point valid for UGV 
	int id_node, id_node_init;
};

#endif
