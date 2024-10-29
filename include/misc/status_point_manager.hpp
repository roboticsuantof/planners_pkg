#ifndef _POINT_ANALISYS_HPP__
#define _POINT_ANALISYS_HPP__

#include "misc/near_neighbor.hpp"
#include "misc/grid3d.hpp"

//**********************************************************************************************************************************
// 				statusPointAnalisys : Provide information about point distance to obstacles and PointClod
//**********************************************************************************************************************************

class statusPointAnalisys
{
    public:
        /** 
        Constructor
        **/
        statusPointAnalisys()
        {}


        /** Variables **/

        Grid3d *grid_3D;
        NearNeighbor nn_trav_ugv;

        void initGrid(Grid3d *g3D){
            grid_3D = g3D;
	        printf("statusPointAnalisys : Initialized Grid3D\n");
        }

        void initNearNeighbor(NearNeighbor nnt_ugv){
            nn_trav_ugv = nnt_ugv;
	        printf("statusPointAnalisys : Initialized NearNeighbor\n");
        }
        
        geometry_msgs::Vector3 getUpperestPointInPC(geometry_msgs::Vector3 p_){
            // Get steer point
            Eigen::Vector3d p_node_, trav_point_ugv_, trav_point_ugv_aux;
            p_node_.x() = p_.x;
            p_node_.y() = p_.y;
            p_node_.z() = p_.z;
            trav_point_ugv_aux = nn_trav_ugv.nearestObstacleMarsupial(nn_trav_ugv.kdtree, p_node_, nn_trav_ugv.obs_points);
            trav_point_ugv_ = nn_trav_ugv.nearestTraversabilityUGVMarsupial(nn_trav_ugv.kdtree, trav_point_ugv_aux, nn_trav_ugv.obs_points, 0.15);

            geometry_msgs::Vector3 ret;
            ret.x = trav_point_ugv_.x();
            ret.y = trav_point_ugv_.y();
            ret.z = trav_point_ugv_.z();

            return ret;
        }

        double distanceToObstacles(const RRTNode &node_){
            if(grid_3D->isIntoMap(node_.pos.x, node_.pos.y, node_.pos.z )){
                double x = (double)node_.pos.x;
                double y = (double)node_.pos.y;
                double z = (double)node_.pos.z;
                TrilinearParams p = grid_3D->getPointDistInterpolation(x,y,z);
                double dist = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
                return dist;
            }
            else{
                // printf("Pose in collision \n");// Not target initially
                return -1.0;
            }
        }

        double distanceToObstacles(const geometry_msgs::Point p_){
            if(grid_3D->isIntoMap(p_.x, p_.y, p_.z )){
                double x = (double)p_.x;
                double y = (double)p_.y;
                double z = (double)p_.z;
                TrilinearParams p = grid_3D->getPointDistInterpolation(x,y,z);
                double dist = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
                return dist;
            }
            else{
                // printf("Pose in collision \n");// Not target initially
                return -1.0;
            }
        }

}; //class 

#endif