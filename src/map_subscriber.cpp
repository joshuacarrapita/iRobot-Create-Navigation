#include <utility>
#include <vector>
#include "ros/ros.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
using namespace std;

red_fundamentals::Grid map;

bool receivedMap = false;

// <row, column>, orientation
vector<pair<pair<int, int>, int>> potential_positions;

bool walls[3];
vector<int> wall_nums;
float coords[6];

void localization(){
    // Maybe get the walls once after we moved?

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
        for(auto pos : potential_positions){
            
        }
    }
    //Output of all the potential positions:
    ROS_INFO("Potential Positions: ");
    for (size_t i = 0; i < potential_positions.size(); ++i) {
        ROS_INFO("(%d, %d)", potential_positions[i].first, potential_positions[i].second);
    }

    // if there is only one potential position left and the orientation is known ( != -1), return and publish it to /pose

    // else:
    // Lets go through the potential positions
    // Looking at a potential position: Which cells can I reach from here?
    // save all reachable cells as next potential positions
    
    // compute the direction in which we want to move

    // move
    // DiffDriving driver;
    
    // get walls
    // red_fundamentals::Walls::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::Walls>("walls_topic", ros::Duration(0.2));

    // localization()


    // Lets go through all the reachable cells
    // Looking at a reachable cell: Do the walls we see match the walls from the cell?
    // Yes: look at the next reachable cell
    // No: Delete the position from the list of potential positions

}



void mapCallback(const red_fundamentals::Grid::ConstPtr& msg) {

    map = *msg;
    receivedMap = true;

    ROS_INFO("Received map");
}

void wallCallback(const red_fundamentals::Walls::ConstPtr& msg){
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

int main(int argc, char **argv){
    ros::init(argc, argv, "map_subscriber");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("map", 1000, mapCallback);
    ros::Subscriber sub1 = n.subscribe("walls_topic", 1, wallCallback);

    // Get the map
    while(!receivedMap){
        ros::spinOnce();
    }

    localization();

    return 0;

}