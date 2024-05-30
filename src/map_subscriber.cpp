#include "ros/ros.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"

red_fundamentals::Grid map;

bool receivedMap = false;

std::vector<std::pair<int, int>> potential_positions;

bool walls[3];
float coords[6];

void localization(){
    // How many walls are around me?

    int numWalls = countNumberofWalls();
    
    // All cells in the map with the same amount of walls are potential positions
    
    // Go through the rows
    for (size_t i = 0; i < map.rows.size(); ++i) {
        const red_fundamentals::Row& row = map.rows[i];

        // Go through the cells
        for (size_t j = 0; j < row.cells.size(); ++j) {
            const red_fundamentals::Cell& cell = row.cells[j];
            ROS_INFO("At coordinates (%lu, %lu), there are %lu walls", i, j, cell.walls.size());
            
            // Same amount of walls get pushed back as potential position
            if(numWalls == cell.walls.size()){
                potential_positions.push_back(std::make_pair(i, j));
            } 
        }
    }

    //Output of all the poteential positions:
    ROS_INFO("Potential Positions: ");
    for (size_t i = 0; i < potential_positions.size(); ++i) {
        ROS_INFO("(%d, %d)", potential_positions[i].first, potential_positions[i].second);
    }

    //Lets go through the potential positions
    // Looking at a potential position: Which cells can I reach from here?
        // Lets go through all the reachable cells
        // Looking at a reachable cell: Do the walls we see match the walls from the cell?
        // Yes: look at the next reachable cell
        // No: Delete the position from the list of potential positions

}

int countNumberofWalls(){
    int numWalls = 0;
    for (int i = 0; i < 3; ++i) {
        if(walls[i] == 1){
            numWalls++;
        } 
    }
    return numWalls;
}

void mapCallback(const red_fundamentals::Grid::ConstPtr& msg) {

    map = *msg;
    receivedMap = true;

    ROS_INFO("Received map");
}

void wallCallback(const red_fundamentals::Walls::ConstPtr& msg){
    for (int i = 0; i < 3; ++i) {
        walls[i] = msg->walls[i];
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