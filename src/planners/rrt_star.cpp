#include "planners_pkg/rrt_star.hpp"

//*****************************************************************
// 		 Random Algorithm Class Definitions (RRT, RRT*, biRRT)
//*****************************************************************

// Default constructor
rrtStarPlanner::rrtStarPlanner()
{}

rrtStarPlanner::~rrtStarPlanner()
{
	grid_3D->~Grid3d();
	v_points_ws_ugv.clear();
	nodes_tree.clear();
	rrt_path.clear();
	v_nodes_kdtree.clear();
}

// Initialization: creates the occupancy matrix (discrete nodes) from the bounding box sizes, resolution, inflation and optimization arguments
void rrtStarPlanner::init(float ws_x_max_, float ws_y_max_, float ws_z_max_, float ws_x_min_, float ws_y_min_, float ws_z_min_,
				   std::string frame_id_, double goal_gap_m_, double distance_obstacle_ugv_, Grid3d *grid3D_, std::string map_file_, std::string path_, ros::NodeHandlePtr nh_)
{
	// Pointer to the nodeHandler
	planner_type = "RRT_Star";
	printf("Global Planner RRT Star Node : \n");// Not target initially
	ws_x_max = (ws_x_max_);
	ws_y_max = (ws_y_max_);
	ws_z_max = (ws_z_max_);
	ws_x_min = (ws_x_min_);
	ws_y_min = (ws_y_min_);
	ws_z_min = (ws_z_min_);

	frame_id = frame_id_;
	goal_gap_m = goal_gap_m_;
	distance_obstacle_ugv = distance_obstacle_ugv_;
	grid_3D = grid3D_;
	map_file = map_file_;
	path = path_;
	status_point = new statusPointAnalisys();
	status_point->initGrid(grid_3D);
	graphM.initPlannerGraphMarkers(nh_);
}

int rrtStarPlanner::computePath()
{
 	double ret_val = -1.0; 
	
	clearStatus();

	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;
	printf("rrtStarPlanner::computePath -->  STARTING --> star_point_ugv[%.2f %.2f %.2f]  goal_point=[%.2f %.2f %.2f] \n\n", 
			pose_initial->pos.x, pose_initial->pos.y, pose_initial->pos.z, pose_final->pos.x, pose_final->pos.y, pose_final->pos.z);  
    
	saveNode(pose_initial,true);
	updateKdtreeNode(*pose_initial);

    num_goal_finded = count_iter = count_total_loop = 0; 

	while (count_iter < n_iter) { // n_iter Max. number of nodes to expand for each round
		printf("\t\t-----  Planner (%s) :: computeTree: iter=[%i/%i] , loop=[%i/%i] , total_node_save[%lu/%i]-----\r",planner_type.c_str(), count_iter+1, n_iter, count_total_loop+1, n_loop, nodes_tree.size(), (count_total_loop+1)+n_iter);
		RRTNode q_rand;
		bool is_q_rand = false;

		do{
			if ((count_iter%samp_goal_rate)!=0)
				is_q_rand = getRandomNode(q_rand);	
			else
				is_q_rand = getRandomNode(q_rand, true);	
		}while(!is_q_rand);
		
		graphM.randNodeMarker(q_rand, count_iter+ (count_total_loop+1));

		extendGraph(q_rand);
		count_iter++;

		if ( ( (num_goal_finded>0) && (planner_type == "rrt") ) || 
			 ( (num_goal_finded>0) && (planner_type == "rrt_star") ) ||
			 ( (num_goal_finded>0) && (planner_type == "birrt") ) ){

			printf("\n\n\nRandomPlanner::computePath -->  finded goal for Coupled Marsupial Configuration.\n")	; 
			rrt_path = getPath(); 
			graphM.pathMarker(rrt_path);

			printf("rrtStarPlanner::computePath -->  finded path for Coupled Marsupial Configuration--> (path size: %lu , iteration numbers: %i) : \n",rrt_path.size(), (count_iter)+(500*count_total_loop)); 
			if (planner_type == "rrt_star")
				printf("rrtStarPlanner::computePath -->  number of goals finded: %i\n",num_goal_finded); 
			int i_=0;   
			printf("\tPrinting the Path Nodes obteinded through planner (%s) : \n",planner_type.c_str());
			for (auto pt_: rrt_path){
				printf("\tRandom_planner_node[%i/%lu] :  ugv=[%.4f %.4f %.4f / %.3f %.3f %.3f %.3f]  cost=%.3f  \n", i_, rrt_path.size(),
				pt_->pos.x, pt_->pos.y, pt_->pos.z, pt_->rot.x, pt_->rot.y, pt_->rot.z, pt_->rot.w,  pt_->cost);
				i_++;
			}
			ret_val = rrt_path.size();
			break;
		}
		else if (count_iter >= n_iter){
			count_total_loop++;
			count_iter = 0;
			if (count_total_loop > n_loop-1){
				printf("rrtStarPlanner::computePath -->  could't find path for Coupled Marsupial Configuration-->  number iteration: %lu \n\n", nodes_tree.size());    
				ret_val = 0;
				break;
			}
			else
				printf("\n\t\t       Planner (%s) :: computeTree: Starting new Loop      \n",planner_type.c_str());
		}
	}
  	std::cout << "Finishing Random Planner: Explored Graph Nodes Numbers: " << nodes_tree.size() <<std::endl;
	std::cout << std::endl << "---------------------------------------------------------------------" << std::endl << std::endl;

  	return ret_val; 
}

bool rrtStarPlanner::extendGraph(const RRTNode q_rand_)
{ 
		RRTNode* new_node = new RRTNode();
		RRTNode q_new;	//Take the new node value before to save it as a node in the list
		RRTNode* q_nearest = getNearestNode(q_rand_); 
		
		bool exist_q_new_ = steering(*q_nearest, q_rand_, q_new);

		if (!exist_q_new_)
			return false;

		q_new.parentNode = q_nearest;
		getParamsNode(q_new);

		graphM.newNodeMarker(q_new, count_iter+ (count_total_loop+1));
		
		RRTNode *q_min;
		q_min = q_nearest;

		std::vector<int> v_near_nodes = getNearNodes(q_new, radius_near_nodes) ;

		updateKdtreeNode(q_new); 	//KdTree is updated after get Near Nodes because to not take it own node as a near node
		bool new_parentNode_ = false;

		// I . Open near nodes and connected with minimum accumulated cost
		if (planner_type == "rrt_star"){
			for (size_t i = 0 ; i < v_near_nodes.size(); i++){
				for (auto nt_:nodes_tree) {
					if(nt_->id == v_near_nodes[i] && nt_->id != q_nearest->id){
						double C_ = nt_->cost + costBetweenNodes(*nt_,q_new);   
						if (C_ < q_new.cost){
							q_min = nt_;
							new_parentNode_ = true;
						}
						break;
					}
				}
			}
		}
		if (new_parentNode_ ){
			q_new.parentNode = q_min;
			updateParamsNode(q_new);
		}
		*new_node = q_new;
		
			// II . Rewire Proccess for UGV
			if (planner_type == "rrt_star"){
				for (size_t i = 0 ; i < v_near_nodes.size(); i++){
					for (auto nt_:nodes_tree) {
						if(nt_->id == v_near_nodes[i] && nt_->id != q_min->id){
							if( nt_->cost > (new_node->cost + costBetweenNodes(*new_node, *nt_)) ){
								nt_->parentNode = new_node;
								nt_->cost = new_node->cost + costBetweenNodes(*new_node, *nt_);
							}
						}
					}
				}
			}

		saveNode(new_node);
		if (isGoal(q_new)){
			if(new_node->cost < pos_goal->cost){
				printf("\n");
				ROS_INFO(PRINTF_ROSE"\n\n\n\t\t !!!!!!!!!!!!!  Got it GOAL position quien new node->point : [%f %f %f]  !!!!!!!!!!!!! \n\n",
						  new_node->pos.x, new_node->pos.y, new_node->pos.z);
				pos_goal = new_node;
				num_goal_finded++;
			}
		}

	return true;
}

bool rrtStarPlanner::getRandomNode(RRTNode &q_rand_, bool go_to_goal_) 
{
	// Random numbers
    std::random_device rd;   // obtain a random number from hardware
  	std::mt19937 eng(rd());  // seed the generator
	int max_ugv = (int)v_points_ws_ugv.size() -1;
  	std::uniform_int_distribution<int> distr(0, max_ugv);  // define the discrete range

	bool finded_node = false;

	// Get random position for UGV
	if (!go_to_goal_){
		int num_rand = distr(eng);
		q_rand_.pos.x = v_points_ws_ugv[num_rand].x;
		q_rand_.pos.y = v_points_ws_ugv[num_rand].y;
		q_rand_.pos.z = v_points_ws_ugv[num_rand].z;  
	}
	else{ // Instead to go to goal, UGV fallow UAV
		q_rand_.pos.x = pose_final->pos.x;
		q_rand_.pos.y = pose_final->pos.y;
		q_rand_.pos.z = pose_final->pos.z;
	}

	if (checkNodeFeasibility(q_rand_))	
		return true;
	else{
		// ROS_ERROR(" getRandomNode : NOT possible to set point [%f %f %f]",q_rand_.pos.x ,q_rand_.pos.y, q_rand_.pos.z);
		return false;
	}
}

RRTNode* rrtStarPlanner::getNearestNode(const RRTNode q_rand_) 
{
  	RRTNode* q_nearest_; 

	double p_ugv_x_, p_ugv_y_, p_ugv_z_, p_ugv_parent_x_, p_ugv_parent_y_, p_ugv_parent_z_;
	double d_ugv_ ,  cos_angle, k0_ , k2_ ;
	k0_ = w_nearest_pos; //  UGV
	k2_ = w_nearest_rot;  // Smoothness

	double cost_nearest_ = 10000000;
	for (auto nt_:nodes_tree) {
		p_ugv_x_ = q_rand_.pos.x  - nt_->pos.x ;
		p_ugv_y_ = q_rand_.pos.y  - nt_->pos.y ;
		p_ugv_z_ = q_rand_.pos.z  - nt_->pos.z ;
		d_ugv_ =  ((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));

		if (nt_->id != id_node_init){	
			//To get smoothness
			p_ugv_parent_x_ = nt_->parentNode->pos.x  - nt_->pos.x ;
			p_ugv_parent_y_ = nt_->parentNode->pos.y  - nt_->pos.y ;
			p_ugv_parent_z_ = nt_->parentNode->pos.z  - nt_->pos.z ;
			//Compute dot product and norm of vectors
			double dot_product = (p_ugv_parent_x_*p_ugv_x_) + (p_ugv_parent_y_*p_ugv_y_) + (p_ugv_parent_z_*p_ugv_z_);
			double norm_vector1 = sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
			double norm_vector2 = sqrt((p_ugv_parent_x_*p_ugv_parent_x_) + (p_ugv_parent_y_*p_ugv_parent_y_) + (p_ugv_parent_z_*p_ugv_parent_z_));
			if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001 )
				cos_angle = 1.0;
			else
				cos_angle = dot_product/(norm_vector1 * norm_vector2);
		}else
			cos_angle = 1.0;

		double cost_ = k0_ * d_ugv_ + k2_ * (1.0 - cos_angle ); //Cost to choose q_nearest

		if(cost_nearest_ > cost_){
			q_nearest_ = nt_;
			cost_nearest_ = cost_;
		}
	}
	return q_nearest_;
}

bool rrtStarPlanner::steering(const RRTNode &q_nearest_, const RRTNode &q_rand_, RRTNode &q_new_)	
{
	// RRTNode q_new_;
	float dir_ugv_x, dir_ugv_y, dir_ugv_z, uni_ugv_x, uni_ugv_y, uni_ugv_z, dist_nearest_rand; 
	geometry_msgs::Vector3 steer_point_, x_ugv_, y_ugv_, z_ugv_;

	//Get the unitary vector from nearest to rand direction
	dir_ugv_x = q_rand_.pos.x  - q_nearest_.pos.x;
	dir_ugv_y = q_rand_.pos.y  - q_nearest_.pos.y; 
	dir_ugv_z = q_rand_.pos.z  - q_nearest_.pos.z;
	dist_nearest_rand = sqrt(dir_ugv_x*dir_ugv_x + dir_ugv_y*dir_ugv_y+ dir_ugv_z*dir_ugv_z );

	// To avoid den = 0
	if (dist_nearest_rand < 0.01){
		steer_point_.x = q_rand_.pos.x; 
		steer_point_.y = q_rand_.pos.y; 
		steer_point_.z = q_rand_.pos.z; 
	}
	else{
		uni_ugv_x = dir_ugv_x/ dist_nearest_rand;
		uni_ugv_y = dir_ugv_y/ dist_nearest_rand;
		uni_ugv_z = dir_ugv_z/ dist_nearest_rand;
		// Move in direction nearest to rand with magnitude proporcional to step_steer
		steer_point_.x = (q_nearest_.pos.x + uni_ugv_x * step_steer); 
		steer_point_.y = (q_nearest_.pos.y + uni_ugv_y * step_steer); 
		steer_point_.z = (q_nearest_.pos.z + uni_ugv_z * step_steer); 
	}

	// Next line to get the upperest point in a specific x-y PointCloud position
	geometry_msgs::Vector3 trav_point_ = status_point->getUpperestPointInPC(steer_point_); 
	q_new_.pos.x = trav_point_.x; 
	q_new_.pos.y = trav_point_.y; 
	q_new_.pos.z = trav_point_.z;

	getOrientation(q_new_, q_nearest_);	

	bool check_ugv_feasible_ = checkNodeFeasibility(q_new_);

	if (check_ugv_feasible_ )
		return true;
	else
		return false;
}

std::vector<int> rrtStarPlanner::getNearNodes(const RRTNode &q_new_, float radius_) 
{
	std::vector<int> v_q_near_;
	std::vector<int> values_;
	v_q_near_.clear();
	values_.clear();

	point_t pt_;
	pt_ = {(float)q_new_.pos.x, (float)q_new_.pos.y, (float)q_new_.pos.z};
		
	KDTree tree(v_nodes_kdtree);
	auto nears_ = tree.neighborhood_points(pt_, radius_);
	for (auto nt_ : nears_) {
		values_.clear();
		for (int v_ : nt_) {
			values_.push_back(v_);
		}
		float x_ = values_[0]; 
		float y_ = values_[1];
		float z_ = values_[2];
		// std::cout << "values_.size()= " << values_.size() << " , values_= " << values_[0] << ", " << values_[1] << ", " << values_[2] << std::endl;
		int id_ = getIdentificationNumber(x_, y_, z_);
		v_q_near_.push_back(id_);
	}
	return v_q_near_;
}

void rrtStarPlanner::getOrientation(RRTNode &n_ , const RRTNode p_)
{
	float yaw_;
	tf::Quaternion _quat;

	yaw_ = atan2(n_.pos.y - p_.pos.y, n_.pos.x - p_.pos.x);
	_quat.setRPY(0.0, 0.0, yaw_);
	n_.rot.x = _quat.x();
	n_.rot.y = _quat.y();
	n_.rot.z = _quat.z();
	n_.rot.w = _quat.w();
}

double rrtStarPlanner::costBetweenNodes(const RRTNode q_near_, const RRTNode q_new_)
{
	double cost_;
	double k0_, k2_;
	double dist_ugv_, cos_angle;
	double p_ugv_x_, p_ugv_y_, p_ugv_z_;
	k0_ = 10.0;	// For ugv
	k2_ = 5.0;	// For ugv smoothness
	
	p_ugv_x_ = q_new_.pos.x - q_near_.pos.x;
	p_ugv_y_ = q_new_.pos.y - q_near_.pos.y;
	p_ugv_z_ = q_new_.pos.z - q_near_.pos.z;
	
	if (q_new_.id != id_node_init){ //This condition to not compute in the first point	
		//To get smoothness
		double p_ugv_parent_x_ = q_new_.parentNode->pos.x - q_new_.pos.x;
		double p_ugv_parent_y_ = q_new_.parentNode->pos.y - q_new_.pos.y;
		double p_ugv_parent_z_ = q_new_.parentNode->pos.z - q_new_.pos.z;
		//Compute dot product and norm of vectors
		double dot_product = (p_ugv_parent_x_*p_ugv_x_) + (p_ugv_parent_y_*p_ugv_y_) + (p_ugv_parent_z_*p_ugv_z_);
		double norm_vector1 = sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
		double norm_vector2 = sqrt((p_ugv_parent_x_*p_ugv_parent_x_) + (p_ugv_parent_y_*p_ugv_parent_y_) + (p_ugv_parent_z_*p_ugv_parent_z_));
		if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001 )
			cos_angle = 1.0;
		else
		cos_angle = dot_product/(norm_vector1 * norm_vector2);
	}else
		cos_angle = 1.0;

	dist_ugv_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_x_*p_ugv_x_) + (p_ugv_z_*p_ugv_z_));
	cost_ = k0_ * dist_ugv_ + k2_* (1.0 - cos_angle );
	return cost_;
}

void rrtStarPlanner::getParamsNode(RRTNode &node_, bool is_init_)
{
	double dist_obs_ = status_point->distanceToObstacles(node_);

	node_.min_dist_obs = dist_obs_;

	if (is_init_){
		node_.id = id_node_init = getIdentificationNumber(node_.pos.x, node_.pos.y, node_.pos.z);
		node_.cost = 0.0;
	}
	else{
		node_.id = getIdentificationNumber(node_.pos.x, node_.pos.y, node_.pos.z);
		node_.cost = costNode(node_);
	}
}

double rrtStarPlanner::costNode(const RRTNode q_new_)
{
	double cost_, k0_, F0_;
	k0_ = 10.0;	// For ugv : 20.0 

	double p_ugv_x_ = q_new_.pos.x - q_new_.parentNode->pos.x ;
	double p_ugv_y_ = q_new_.pos.y - q_new_.parentNode->pos.y ;
	double p_ugv_z_ = q_new_.pos.z - q_new_.parentNode->pos.z ;

	F0_ =  sqrt((p_ugv_x_*p_ugv_x_) + (p_ugv_y_*p_ugv_y_) + (p_ugv_z_*p_ugv_z_));
	cost_ = k0_ * F0_ + q_new_.parentNode->cost;

	return cost_;
}

void rrtStarPlanner::updateParamsNode(RRTNode &node_)
{
	node_.cost = costNode(node_);
}

bool rrtStarPlanner::checkNodeFeasibility(const RRTNode &p_)
{
	bool ret;

	double d_ = status_point->distanceToObstacles (p_);

	if (d_ > distance_obstacle_ugv && p_.pos.x < ws_x_max && p_.pos.x > ws_x_min &&
		p_.pos.y < ws_y_max && p_.pos.y > ws_y_min && p_.pos.z < ws_z_max && p_.pos.z > ws_z_min)
		ret = true;
	else
		ret = false;	
	return ret;
}

void rrtStarPlanner::updateKdtreeNode(const RRTNode ukT_)
{
	point_t p_;

	p_ = {(float)ukT_.pos.x, (float)ukT_.pos.y, (float)ukT_.pos.z};
	v_nodes_kdtree.push_back(p_);
}

void rrtStarPlanner::saveNode(RRTNode* sn_, bool is_init_)
{
	if(is_init_){
		getParamsNode(*sn_,is_init_);
		printf("rrtStarPlanner::computePath --> Saved initial node\n\n");
	}
	nodes_tree.push_back(sn_);
}

inline void rrtStarPlanner::clearStatus()
{
	nodes_tree.clear();
	rrt_path.clear();

	num_goal_finded = 0;
	v_nodes_kdtree.clear();
}

void rrtStarPlanner::readPointCloudTraversabilityMapUGV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_trav_ugv.setInput(*msg);
	status_point->initNearNeighbor(nn_trav_ugv);

	ROS_INFO_COND(true, PRINTF_BLUE "rrtStarPlanner Planner: Receiving point cloud map to create Kdtree for Traversability UGV");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg,*cloud_in);

	ROS_INFO(PRINTF_BLUE"rrtStarPlanner::readPointCloudTraversabilityMapUGV  size point cloud = [%lu]",cloud_in->size());
	geometry_msgs::Vector3 point_;
	for (size_t i = 0 ; i < cloud_in->size() ; i ++){
		point_.x = cloud_in->points[i].x;
		point_.y = cloud_in->points[i].y;
		point_.z = cloud_in->points[i].z;
		v_points_ws_ugv.push_back(point_);
	}
	ROS_INFO(PRINTF_BLUE"rrtStarPlanner::readPointCloudTraversabilityMapUGV  size v_points_ws_ugv = [%lu]",v_points_ws_ugv.size());
}

void rrtStarPlanner::readPointCloudMapForUGV(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	nn_obs_ugv.setInput(*msg);
    pc_obs_ugv = msg;
	ROS_INFO_COND(true, PRINTF_BLUE "rrtStarPlanner Planner: Receiving point cloud map to create Kdtree for UGV Obstacles");
}

std::list<RRTNode*> rrtStarPlanner::getPath()
{
	std::list<RRTNode*> path_;
	RRTNode* current_node;

	current_node = pos_goal;
	path_.push_front(current_node);
	while (current_node->parentNode != NULL){ 
		current_node = current_node->parentNode;
		path_.push_front(current_node);
	}
	
	return path_;
}

bool rrtStarPlanner::isGoal(const RRTNode st_) 
{
	geometry_msgs::Vector3 point_;

	point_.x = st_.pos.x;
	point_.y = st_.pos.y;
	point_.z = st_.pos.z;

	double dist_goal_ = sqrt(pow(point_.x - pose_final->pos.x,2) + 
							 pow(point_.y - pose_final->pos.y,2) +
				 			 pow(point_.z - pose_final->pos.z,2) );
	if(dist_goal_ < 1.0)
		return true;
	else
		return false;
}

void rrtStarPlanner::setStartAndGoal(geometry_msgs::Point s_, geometry_msgs::Point g_){
	pose_initial = new RRTNode();
	pose_final = new RRTNode();

	pose_initial->pos.x = s_.x; 
	pose_initial->pos.y = s_.y;
	pose_initial->pos.z = s_.z;
	pose_final->pos.x = g_.x; 
	pose_final->pos.y = g_.y;
	pose_final->pos.z = g_.z;
	ROS_INFO("\t\t StartPoint[%f %f %f ] GoalPoint[%f %f %f]",
                    pose_initial->pos.x,pose_initial->pos.y, pose_initial->pos.z, pose_final->pos.x, pose_final->pos.y, pose_final->pos.z);
	graphM.graphStartFinalPoint(s_, g_);

	pos_goal = new RRTNode();
	pos_goal->pos.x = s_.x;
	pos_goal->pos.y = s_.y;
	pos_goal->pos.z = s_.z;
	pos_goal->parentNode = NULL;
	pos_goal->cost = 1000000000;
}

void rrtStarPlanner::configRRTParameters(int n_iter_ , int n_loop_, double r_nn_, double s_s_, int s_g_r_, double w_n_ugv_, double w_n_smooth_)
{
	n_iter = n_iter_;
	n_loop = n_loop_;
	radius_near_nodes = r_nn_;
	step_steer = s_s_;
	samp_goal_rate = s_g_r_;
	w_nearest_pos = w_n_ugv_ ;
	w_nearest_rot = w_n_smooth_ ;

}

