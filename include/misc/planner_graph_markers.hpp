#ifndef _PLANER_GRAPH_MARKERS_H_
#define _PLANER_GRAPH_MARKERS_H_

#include <list>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3.h>

#include <planners_pkg/RRTNode.h>

class PlannerGraphMarkers
{
	public:

    ros::NodeHandlePtr nh;

	ros::Publisher goal_point_pub_, start_point_pub_, rand_points_pub_, new_point_pub_, tree_rrt_star_ugv_pub_, path_marker_pub_;

	visualization_msgs::MarkerArray pointTreeMarkerUGV;
  	visualization_msgs::MarkerArray lines_ugv_marker;

	PlannerGraphMarkers()
	{}

	void initPlannerGraphMarkers(ros::NodeHandlePtr nh_)
	{
    	nh = nh_;
		start_point_pub_ = nh->advertise<visualization_msgs::Marker>("start_point_planner", 2, true);
    	goal_point_pub_ = nh->advertise<visualization_msgs::Marker>("goal_point_planner", 2, true);
    	rand_points_pub_ = nh->advertise<visualization_msgs::Marker>("rand_points_planner", 10, true);
    	new_point_pub_ = nh->advertise<visualization_msgs::Marker>("new_point_planner", 10, true);
		path_marker_pub_ = nh->advertise<visualization_msgs::MarkerArray>("path_marker_planner", 20, true);
	}

	void graphStartFinalPoint(geometry_msgs::Point i_pos_, geometry_msgs::Point f_pos_)
	{
		visualization_msgs::Marker marker_;
		marker_.header.frame_id = "map";
		marker_.header.stamp = ros::Time();
		marker_.ns = "goal_point";
		marker_.id = 0;
		marker_.type = visualization_msgs::Marker::SPHERE;
		marker_.action = visualization_msgs::Marker::ADD;
		marker_.lifetime = ros::Duration(0);
		marker_.pose.position.x = f_pos_.x;
		marker_.pose.position.y = f_pos_.y;
		marker_.pose.position.z = f_pos_.z;
		marker_.pose.orientation.x = 0.0;
		marker_.pose.orientation.y = 0.0;
		marker_.pose.orientation.z = 0.0;
		marker_.pose.orientation.w = 1.0;
		marker_.scale.x = 1.0;
		marker_.scale.y = 1.0;
		marker_.scale.z = 1.0;
		marker_.color.r = 0.4;
		marker_.color.g = 0.0;
		marker_.color.b = 1.0;
		marker_.color.a = 1.0; 
		goal_point_pub_.publish(marker_);

		marker_.ns = "start_point";
		marker_.pose.position.x = i_pos_.x;
		marker_.pose.position.y = i_pos_.y;
		marker_.pose.position.z = i_pos_.z;
		marker_.color.r = 0.0;
		marker_.color.g = 0.0;
		marker_.color.b = 1.0;
		marker_.color.a = 1.0; 
		start_point_pub_.publish(marker_);
	}

	void randNodeMarker(const RRTNode rn_, int id_)
	{
		visualization_msgs::Marker marker_rand;
		marker_rand.header.frame_id = "map";
		marker_rand.header.stamp = ros::Time();
		marker_rand.ns = "rand_point";
		marker_rand.id = id_;
		marker_rand.type = visualization_msgs::Marker::SPHERE;
		marker_rand.action = visualization_msgs::Marker::ADD;
		marker_rand.lifetime = ros::Duration(0);
		marker_rand.pose.position.x = rn_.pos.x;
		marker_rand.pose.position.y = rn_.pos.y;
		marker_rand.pose.position.z = rn_.pos.z;
		marker_rand.pose.orientation.x = 0.0;
		marker_rand.pose.orientation.y = 0.0;
		marker_rand.pose.orientation.z = 0.0;
		marker_rand.pose.orientation.w = 1.0;
		marker_rand.scale.x = 0.8;
		marker_rand.scale.y = 0.8;
		marker_rand.scale.z = 0.8;
		marker_rand.color.r = 0.0;
		marker_rand.color.g = 0.0;
		marker_rand.color.b = 0.0;
		marker_rand.color.a = 1.0; 
		
		rand_points_pub_.publish(marker_rand);
	}

	void newNodeMarker(const RRTNode rn_, int id_)
	{
		visualization_msgs::Marker marker_new;
		marker_new.header.frame_id = "map";
		marker_new.header.stamp = ros::Time();
		marker_new.ns = "new_node";
		marker_new.id = id_;
		marker_new.type = visualization_msgs::Marker::SPHERE;
		marker_new.action = visualization_msgs::Marker::ADD;
		marker_new.lifetime = ros::Duration(0);
		marker_new.pose.position.x = rn_.pos.x;
		marker_new.pose.position.y = rn_.pos.y;
		marker_new.pose.position.z = rn_.pos.z;
		marker_new.pose.orientation.x = rn_.rot.x;
		marker_new.pose.orientation.y = rn_.rot.y;
		marker_new.pose.orientation.z = rn_.rot.z;
		marker_new.pose.orientation.w = rn_.rot.w;
		marker_new.scale.x = 0.8;
		marker_new.scale.y = 0.8;
		marker_new.scale.z = 0.8;
		marker_new.color.r = 1.0;
		marker_new.color.g = 1.0;
		marker_new.color.b = 1.0;
		marker_new.color.a = 1.0; 
		
		new_point_pub_.publish(marker_new);
	}

	void pathMarker(std::list<RRTNode*> pt_)
	{
		geometry_msgs::Point _p1, _p2; 
		lines_ugv_marker.markers.resize(pt_.size()-1);
		int i_ = 0;
		for (auto p_:pt_){
			_p2.x = p_->pos.x;
			_p2.y = p_->pos.y;
			_p2.z = p_->pos.z;
			if (i_ > 0){
				lines_ugv_marker.markers[i_-1].header.frame_id = "map";
				lines_ugv_marker.markers[i_-1].header.stamp = ros::Time::now();
				lines_ugv_marker.markers[i_-1].ns = "RRTStar_Path";
				lines_ugv_marker.markers[i_-1].id = i_ + pt_.size();
				lines_ugv_marker.markers[i_-1].action = visualization_msgs::Marker::ADD;
				lines_ugv_marker.markers[i_-1].type = visualization_msgs::Marker::LINE_STRIP;
				lines_ugv_marker.markers[i_-1].lifetime = ros::Duration(0);
				lines_ugv_marker.markers[i_-1].points.push_back(_p1);
				lines_ugv_marker.markers[i_-1].points.push_back(_p2);
				lines_ugv_marker.markers[i_-1].pose.orientation.x = 0.0;
				lines_ugv_marker.markers[i_-1].pose.orientation.y = 0.0;
				lines_ugv_marker.markers[i_-1].pose.orientation.z = 0.0;
				lines_ugv_marker.markers[i_-1].pose.orientation.w = 1.0;
				lines_ugv_marker.markers[i_-1].scale.x = 0.1;
				// lines_ugv_marker.markers[i].scale.y = 0.3;
				// lines_ugv_marker.markers[i].scale.z = 0.1;
				lines_ugv_marker.markers[i_-1].color.a = 1.0;
				lines_ugv_marker.markers[i_-1].color.r = 0.0;
				lines_ugv_marker.markers[i_-1].color.g = 0.0;
				lines_ugv_marker.markers[i_-1].color.b = 1.0;
			}
			_p1.x = p_->pos.x;
			_p1.y = p_->pos.y;
			_p1.z = p_->pos.z;	//Move in Z to see the point over the map surface
			i_++;
		}
		path_marker_pub_.publish(lines_ugv_marker);
	}

	void clearPathMarkers(int n_iter_)
	{
		auto size_ = n_iter_;
		lines_ugv_marker.markers.clear();
		lines_ugv_marker.markers.resize(size_);
		for (auto i = 0 ; i < size_; i++){
			lines_ugv_marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
		}
		path_marker_pub_.publish(lines_ugv_marker);
	}

	void clearMarkersNodesTree()
	{
		auto size_ = pointTreeMarkerUGV.markers.size();

		pointTreeMarkerUGV.markers.clear();
		pointTreeMarkerUGV.markers.resize(size_);
		for (auto i = 0 ; i < size_; i++){
			pointTreeMarkerUGV.markers[i].action = visualization_msgs::Marker::DELETEALL;
		}
		tree_rrt_star_ugv_pub_.publish(pointTreeMarkerUGV);
	}

}; //class

#endif
