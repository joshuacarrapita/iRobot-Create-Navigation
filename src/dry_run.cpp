#include "ros/ros.h"
#include "red_fundamentals/SimpleCall.h"
#include "red_fundamentals/Localization.h"
#include "red_fundamentals/ExecutePlanNew.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
#include "red_fundamentals/Pose.h"
#include <cstdlib>
#include "graph.hpp"
#include "song.hpp"
using namespace std;

red_fundamentals::Grid map_cur;
Graph graph;
ros::ServiceClient client_loc;
red_fundamentals::Localization srv_loc;
ros::ServiceClient client_ep;
red_fundamentals::ExecutePlanNew srv_ep;
red_fundamentals::Pose pose_cur;
bool need_localization = true;

void mapCallback(){
    while(ros::ok()){
        const red_fundamentals::Grid::ConstPtr& msg = ros::topic::waitForMessage<red_fundamentals::Grid>("map", ros::Duration(10));

        if (msg != nullptr){
            map_cur = *msg;
            ROS_INFO("Number of rows: %lu", msg->rows.size());
            for(size_t i=0; i<msg->rows.size(); i++){
                const red_fundamentals::Row& row = msg->rows[i];
                ROS_INFO("Row %lu: Number of cells: %lu", i, row.cells.size());
                for(size_t j=0; j<msg->rows[0].cells.size(); j++){
                    const red_fundamentals::Cell& cell = row.cells[j];
                    ROS_INFO("Row %lu, Cell %lu: Number of walls: %lu", i, j, cell.walls.size());
                    for(size_t k=0; k<cell.walls.size(); k++){
                        ROS_INFO("Row %lu, Cell %lu, Wall %lu: %d", i, j, k, cell.walls[k]);
                    }
                }
            }
            // create a graph using the received map
            graph.maptograph(map_cur.rows.size(), map_cur.rows[0].cells.size());
            graph.addEdges(&map_cur);
            graph.setType();
            break;
        }
        else{
            ROS_INFO("Waiting for the map information");
        }
    }
}

bool collect_golds(red_fundamentals::SimpleCall::Request  &req,
                      red_fundamentals::SimpleCall::Response &res)
{
    // initialize variables
    res.success = false;
    vector<int> partial_paths[graph.golds.size()+1];
    Song s;
    // TODO call localization to get a current position, if it is unknown
    while(ros::ok()){
	int num_gold = (int)graph.golds.size();
        if(need_localization){
            if (client_loc.call(srv_loc)) {
                ROS_INFO("row: %d, column: %d, orientation: %d",srv_loc.response.pose.row, srv_loc.response.pose.column, 		srv_loc.response.pose.orientation);
                pose_cur.row = srv_loc.response.pose.row;
                pose_cur.column = srv_loc.response.pose.column;
                pose_cur.orientation = srv_loc.response.pose.orientation;
                need_localization = false;
				vector<int> path;
                int start = graph.postoidx(pose_cur.row, pose_cur.column);
                // TODO if there is no gold remaining, find a path to the PICKUP location
                if(num_gold == 0){
                	for(int pickup : graph.pickups){
                		vector<int> tmp = graph.bfs(start, pickup);
                		if(path.empty() || path.size() > tmp.size()) path = tmp;
                	}
                }
                // else get a shortest path through all remaining golds and one pickup position
				else path = graph.findPathVisitingAllGold(start);
				for(int p : path){
					ROS_INFO("%d", p);
				}
                // TODO divide the entire paht into paths from gold to gold and to pickup
		
		for(int i=0; i<=num_gold; i++){
			partial_paths[i].clear();
		}
		
        if(num_gold == 0) {
			ROS_INFO("path to PICKUP");
			partial_paths[0] = path;
			for(int p : partial_paths[0]){
				ROS_INFO("%d", p);
			}
		}	
        else{
            int goldno = 0;
            vector<int> golds = graph.golds;
            for(int cell : path){
                partial_paths[goldno].push_back(cell);
                if(graph.contains(golds, cell)) {
                    goldno++;
                    partial_paths[goldno].push_back(cell);
                    graph.remove(golds, cell);
                }
            }
                
            for(int j=0; j <= num_gold; j++){
                ROS_INFO("to gold no. %d: ", j);
	        	for(int cell : partial_paths[j]){
	        		ROS_INFO("%d", cell);
	        	}
	    	}
        }
                
            }
            else {
                ROS_INFO("Could not call the localization service");
                return true;
            }
        }




        // TODO Move to position: we should move to the position as quickly and smoothly as possible
        // below is a placeholder for testing
        // TODO do it for each partial path
        for(int cnt=0; cnt<=num_gold; cnt++){
			ROS_INFO("entered?");
            vector<int> plan;
            for(size_t i=0; i<partial_paths[cnt].size()-1; i++){
                switch(partial_paths[cnt][i+1] - partial_paths[cnt][i]){
                    case -6:
                        plan.push_back(1);
                        break;
                    case -1:
                        plan.push_back(2);
                        break;
                    case 1:
                        plan.push_back(0);
                        break;
                    case 6:
                        plan.push_back(3);
                        break;
                }
            }
            srv_ep.request.plan = plan;
            srv_ep.request.row = pose_cur.row;
            srv_ep.request.column = pose_cur.column;
            srv_ep.request.orientation = pose_cur.orientation;
            if(client_ep.call(srv_ep)){
                if(srv_ep.response.success){
                    // TODO update current position for the next call if the plan could be executed
                    pose_cur.row = *partial_paths[cnt].rbegin() / graph.num_col;
                    pose_cur.column = *partial_paths[cnt].rbegin() % graph.num_col;
                    // TODO update orientation
                    pose_cur.orientation = *plan.rbegin();
                    graph.remove(graph.golds, *partial_paths[cnt].rbegin());
                    s.opentheChest((unsigned int)graph.golds.size());
                    ROS_INFO("GOLD or PICKUP! remaining golds: ");
                    for(int g : graph.golds){
                    	ROS_INFO("%d", g);
                    }
                    ROS_INFO("current pose:  row: %d, column: %d, orientation: %d", pose_cur.row, pose_cur.column, pose_cur.orientation);
                    
                    
                }
                else{
                    // TODO think about under what circumstances the plan cannot be executed and what to do
                    // idea: relocalize (instead of execute plan srv) + compute the path again + run the new plan
                    // but maintain the target position
                    need_localization = true;
                    break;
                }
            }
            else {
                ROS_INFO("Could not call the execute plan service");
                return true;
            }
        }
        
        if(!need_localization){
        	res.success = true;
        	break;
	}
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dry_run_server");
    ros::NodeHandle n;
    mapCallback();
    client_loc = n.serviceClient<red_fundamentals::Localization>("localization");
    client_ep = n.serviceClient<red_fundamentals::ExecutePlanNew>("execute_plan");
    ros::ServiceServer service = n.advertiseService("dry_run", collect_golds);
    ROS_INFO("Ready to run dry run server");
    ros::spin();

    return 0;
}
