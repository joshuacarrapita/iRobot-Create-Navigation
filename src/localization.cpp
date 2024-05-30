#include <utility>
#include <vector>
#include "ros/ros.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
#include "red_fundamentlas/Pose.h"
#include "paradriver.hpp"
using namespace std;

red_fundamentals::Grid map;

bool receivedMap = false;

// <row, column>, orientation
vector<pair<pair<int, int>, int>> potential_positions;

bool walls[3];
vector<int> wall_nums;
float coords[6];

ros::Publisher pose_pub;

void mapCallback(const red_fundamentals::Grid::ConstPtr& msg) {
    if(msg != nullptr){
        map = *msg;
        receivedMap = true;
        ROS_INFO("Received map");
    }
    else{
        ROS_INFO("Waiting for the map information");
    }
}

void wallCallback(){
    red_fundamentals::Walls::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::Walls>("walls_topic", ros::Duration(0.2));
    
    wall_nums.clear();
    for (int i = 0; i < 3; ++i) {
        walls[i] = msg->walls[i];
        if(msg->walls[i]){
            wall_nums.push_back(i);
        }
    }
    for (int i = 0; i < 6; ++i) {
        coords[i] = msg->coords[i];
    }
}

bool contains(int arr[], int size, int value) {
    // Use std::find to search for the value in the array
    return std::find(arr, arr + size, value) != arr + size;
}

bool contains(const std::vector<int>& vec, int value) {
    // Use std::find to search for the value in the vector
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}

bool compareWalls(red_fundamentals::Cell cell, int orientation){
    if(wall_nums.size() != cell.walls.size()) return false;
    bool walls_map[4];
    for(int wall : cell.walls){
        // we convert wall direction in the map to one the robot is facing
        switch (orientation) {
            case 0:
                walls_map[(wall + 1) % 4] = true;
                break;
            case 1:
                walls_map[wall] = true;
                break;
            case 2:
                walls_map[(wall + 3) % 4] = true;
                break;
            case 3:
                walls_map[(wall + 2) % 4] = true;
                break;
            default:
                break;
        }
    }

    for(int i=0; i<3; i++){
        if(walls[i] != walls_map[i]) return false;
    }

    // Since we only call this function after we move from one cell to another cell,
    // it is not possible that there is a wall behind us
    if(walls_map[3]) return false;
    return true;
}

void localization(){
    // Maybe get the walls once after we moved?
    // wallCallback();
    // First, we compare the current candidates with the walls we received
    // if potential_positions is empty, compare the number of walls
    // All cells in the map with the same amount of walls are potential positions
    if(potential_positions.empty()){
        // Go through the rows
        for (size_t i = 0; i < map.rows.size(); ++i) {
            const red_fundamentals::Row& row = map.rows[i];

            // Go through the cells
            for (size_t j = 0; j < row.cells.size(); ++j) {
                const red_fundamentals::Cell& cell = row.cells[j];
                ROS_INFO("At coordinates (%lu, %lu), there are %lu walls", i, j, cell.walls.size());

                // Same amount of walls get pushed back as potential position
                // if NumWalls == 2 -> compare the difference of wall numbers
                // since we do not know the orientation, save -1 for it
                if(wall_nums.size() == cell.walls.size()){
                    if(wall_nums.size() != 2 || abs(wall_nums[0]-wall_nums[1]) % 2 == abs(cell.walls[0] - cell.walls[1]) % 2) {
                        potential_positions.emplace_back(make_pair(i, j), -1);
                    }
                }
            }
        }
    }

    // else: we already have candidates!
    // Go through the potential positions compare the number of walls AND their orientation
    else{
        for(auto candidate = potential_positions.begin(); candidate != potential_positions.end(); ){
            red_fundamentals::Cell cell = map.rows[candidate->first.first].cells[candidate->first.second];
            if(!compareWalls(cell, candidate->second)){
                candidate = potential_positions.erase(candidate);
            }
            else{
                candidate++;
            }
        }
    }

    //Output of all the potential positions:
    ROS_INFO("Potential Positions: ");
    for (size_t i = 0; i < potential_positions.size(); ++i) {
        ROS_INFO("(%d, %d)", potential_positions[i].first.first, potential_positions[i].first.second);
    }

    // if there is only one potential position left and the orientation is known ( != -1), return and publish it to /pose
    if(potential_positions.size() == 1 && potential_positions[0].second != -1){
        red_fundamentals::Pose pose;
        pose.row = potential_positions[0].first.first;
        pose.column = potential_positions[0].first.second;
        pose.orientation = potential_positions[0].second;
    
        ros::Duration(1.0).sleep();
        pose_pub.publish(pose);
        ROS_INFO("Pose message published");
        return;
    }

    else {
        // Lets go through the potential positions
        // Looking at a potential position: Which cells can I reach from here?

        vector<pair<pair<int, int>, int>> reachable_cells;

        for(auto potential_position : potential_positions){

            int row = potential_position.first.first;
            int col = potential_position.first.second;

            int cell[] = map.rows[row].cells[col];
            
            
            if(! contains(cell, cell.size(), 0)){
                // if we don't have a 0, then a reachable cell would be in the map at (row, col +1)
                if(map.rows[0].size() <= col + 1){
                    ROS_INFO("Map Error ?");
                } else {
                    reachable_cells.emplace_back(std::make_pair(row, col + 1), 0);
                }
               
            }

            if(! contains(cell, cell.size(), 1)){
                // if we don't have a 1, then a reachable cell would be in the map at (row -1, col)
                if(0 > row - 1){
                    ROS_INFO("Map Error ?");
                } else {
                    reachable_cells.emplace_back(std::make_pair(row - 1, col), 1);
                }
            }

            if(! contains(cell, cell.size(), 2)){
                // if we don't have a 2, then a reachable cell would be in the map at (row, col -1)
                if(0 > col - 1){
                    ROS_INFO("Map Error ?");
                } else {
                    reachable_cells.emplace_back(std::make_pair(row, col - 1), 2);
                }
            }

            if(! contains(cell, cell.size(), 3)){
                // if we don't have a 3, then a reachable cell would be in the map at (row +1, col)
                if(map.size() >= row + 1){
                    ROS_INFO("Map Error ?");
                } else {
                    reachable_cells.emplace_back(std::make_pair(row + 1, col), 3);
                }
            }
        }

        // save all reachable cells as next potential positions
        potential_positions.clear();
        potential_positions = reachable_cells;
    }
    
    // compute the direction in which we want to move
    // we just choose the smallest possible direction (where we couldn't find a wall)
    int move_direction = 0;

    for(int i=0; i < 4; i++){
        if(! contains.(wall_nums, i)){
            move_direction = i;
            break;
        }
    }

    // move into right direction

    DiffDriving driver;
    
    switch(move_direction){
        case 0:
            driver.drive(0, 800, 0, 0, 1);
            break;
        case 1:
            driver.drive(0, 0, 800, 0, 1);
            break;
        case 2:
            driver.drive(0, -800, 0, 0, 1);
            break;
        case 3:
            driver.drive(0, 0, -800, 0, 1);
            break;
        default:
            ROS_INFO("Wrong Direction");
    }
    
    // get walls
    wallCallback();

    localization();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "localization");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("map", 1000, mapCallback);

    ros::Publisher pose_pub = n.advertise<red_fundamentals::Pose>("/pose", 1);

    // Get the map
    while(!receivedMap){
        ros::spinOnce();
    }

    // Get walls
    wallCallback();

    localization();

    return 0;

}