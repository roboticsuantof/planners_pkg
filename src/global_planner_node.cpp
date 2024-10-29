#include <planners_pkg/global_planner.hpp>


int main(int argc, char **argv)
{
    std::string node_name = "global_planner_node";

	ros::init(argc, argv, node_name);

    GlobalPlanner RandomStarGP(node_name);
    
    // dynamic_reconfigure::Server<theta_star_2d::GlobalPlannerConfig> server;
  	// dynamic_reconfigure::Server<theta_star_2d::GlobalPlannerConfig>::CallbackType f;

  	// f = boost::bind(&RandomGlobalPlanner::dynReconfCb,&RRTStarGP,  _1, _2);
  	// server.setCallback(f);

	ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        // RandomStarGP.plan();

        loop_rate.sleep();
    }
    return 0;
}
