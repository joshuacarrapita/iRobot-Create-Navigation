#include "ros/ros.h"
#include "red_fundamentals/ExecutePlan.h"
#include <cstdlib>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 2)
  {
    ROS_INFO("usage: execute_plan_client PLAN");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<red_fundamentals::ExecutePlan>("execute_plan_client");
  red_fundamentals::ExecutePlan srv;

  std::string input = argv[1];
  std::string tmp;
  std::vector<int32_t> plan;
  for(char c : input){
  	if(std::isdigit(c)){
  		tmp = c;
  		plan.push_back(std::stoi(tmp));
  	}
  }
  
  srv.request.plan = plan;
  if (client.call(srv))
  {
    ROS_INFO("travel succeeded?: %s", srv.response.success ? "true" : "false");
  }
  else
  {
    ROS_ERROR("Failed to call service execute_plan");
    return 1;
  }

  return 0;
}
