#ifndef __RRT_NODE_H__
#define __RRT_NODE_H__

#include <geometry_msgs/Point.h>

//*****************************************************************
//				Auxiliar Class for RRTPlanner Algorithm
//*****************************************************************

class NodePosition
{
public:
	float x, y, z;
};

class nodeOrientation
{
public:
	float x, y, z, w;
};

class RRTNode;

class RRTNodeLink3D
{
public:
	RRTNodeLink3D() : node(NULL), isInOpenList(false), isInCandidateList(false), notOccupied(true)
	{
	}
	RRTNode *node;  // Node from this link
	bool isInOpenList;		// algorithm open list flag
	bool isInCandidateList; // algorithm candidate list flag
	bool notOccupied;		// occupancy mark
	float cost;
};

class RRTNode
{
public:
	// NodeState st;
	NodePosition pos;
	nodeOrientation rot;
	int id;
	double min_dist_obs;	//Minimun distance from UGV node to obstacle
	double cost;	// Refer to the cost in the node without consider the catenary
	float dist;
	std::vector<geometry_msgs::Vector3> p_cat;
	// double h_cost;
	RRTNode *parentNode;
	RRTNodeLink3D *nodeInWorld; // pointer to link from this node to its parent
	// RRTNode()
	// {
	//   parentNode = NULL;
	// }

	// // Comparator '!=' definition
	// friend bool operator!=(const RRTNode &lhs, const RRTNode &rhs)
	// {
	// 	return lhs.point.x != rhs.point.x ||
	// 		   lhs.point.y != rhs.point.y ||
	// 		   lhs.point.z != rhs.point.z;
	// }
};

#endif
