#include "ros/ros.h"
#include "red_fundamentals/MoveToPosition.h"
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

bool move_to_position(red_fundamentals::MoveToPosition::Request  &req,
                      red_fundamentals::MoveToPosition::Response &res)
{
    // initialize variables
    res.success = false;
    if(graph.num_row <= 0 || graph.num_col <= 0 || req.row >= graph.num_row || req.column >= graph.num_col){
        ROS_INFO("requested position is outside of the map");
        return true;
    }
    int row = req.row;
    int column = req.column;
    Song s;

    // TODO call localization to get a current position, if it is unknown
    while(ros::ok()){
        if(need_localization){
            if (client_loc.call(srv_loc)) {
                ROS_INFO("row: %d, column: %d, orientation: %d",srv_loc.response.pose.row, srv_loc.response.pose.column, 		srv_loc.response.pose.orientation);
                pose_cur.row = srv_loc.response.pose.row;
                pose_cur.column = srv_loc.response.pose.column;
                pose_cur.orientation = srv_loc.response.pose.orientation;
                need_localization = false;
            }
            else {
                ROS_INFO("Could not call the localization service");
                return true;
            }
        }

        if(pose_cur.row == row && pose_cur.column == column){
            res.success = true;
            s.playSuccessSound();
            return true;
        }

        int start = graph.postoidx(pose_cur.row, pose_cur.column);
        int goal = graph.postoidx(req.row, req.column);

        // TODO get a shortest path
        vector<int> path = graph.bfs(start, goal);
        for(int cell : path){
            ROS_INFO("%d", cell);
        }

        // TODO Move to position: we should move to the position as quickly and smoothly as possible
        // below is a placeholder for testing
        vector<int> plan;
        for(size_t i=0; i<path.size()-1; i++){
            switch(path[i+1] - path[i]){
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
                pose_cur.row = req.row;
                pose_cur.column = req.column;
                // TODO update orientation
                pose_cur.orientation = *plan.rbegin();
                ROS_INFO("Moved to the given position! Ready to run another one!");
                ROS_INFO("current pose:  row: %d, column: %d, orientation: %d", pose_cur.row, pose_cur.column, pose_cur.orientation);
                res.success = true;
                s.playSuccessSound();
                break;
            }
            else{
                // TODO think about under what circumstances the plan cannot be executed and what to do
                // idea: relocalize (instead of execute plan srv) + compute the path again + run the new plan
                // but maintain the target position
                need_localization = true;
            }
        }
        else {
            ROS_INFO("Could not call the execute plan service");
            return true;
        }
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_to_position_server");
    ros::NodeHandle n;
    mapCallback();
    client_loc = n.serviceClient<red_fundamentals::Localization>("localization");
    client_ep = n.serviceClient<red_fundamentals::ExecutePlanNew>("execute_plan");
    ros::ServiceServer service = n.advertiseService("move_to_position", move_to_position);
    ROS_INFO("Ready to run Move To Position");
    ros::spin();

    return 0;
}
