#include <utility>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
#include "red_fundamentals/Pose.h"
#include "red_fundamentals/Walls.h"
#include "paradriver.hpp"
#include "moving.hpp"

using namespace std;

red_fundamentals::Grid map_cur;
ros::Publisher pose_pub;

// bool receivedMap = false;
bool ready = false;
// <row, column>, orientation
vector<pair<pair<int, int>, int>> potential_positions;

bool walls[3] = {false,};
vector<int> wall_nums;
// float coords[6];

double laserCallback(){
    sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan_filtered", ros::Duration(1));
    if (!msg) return -1;
    return msg->ranges[msg->ranges.size()/2];
}

void mapCallback() {
    while(ros::ok()){
	    const red_fundamentals::Grid::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::Grid>("map",
                                                                                                    ros::Duration(1.0));

	    if(msg != nullptr){
            	map_cur = *msg;
            	// receivedMap = true;
        	ROS_INFO("Received map");
		break;
    	    }
    	else{
        	ROS_INFO("Waiting for the map information");
    	}
    }
}

void wallCallback(){
    while(ros::ok()) {
        const red_fundamentals::Walls::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::Walls>("walls_topic",
                                                                                                    ros::Duration(0.2));
        if(msg != nullptr) {
            wall_nums.clear();
            for (int i = 0; i < 3; ++i) {
                walls[i] = msg->walls[i];
                if (msg->walls[i]) {
                    wall_nums.push_back(i);
                }
            }
            /* for (int i = 0; i < 6; ++i) {
                coords[i] = msg->coords[i];
            } */
            break;
        }

        else{
            ROS_INFO("Waiting for the wall information...");
        }
    }

    ROS_INFO("walls = [ ");
    for(int i = 0; i < 3; i++){
        ROS_INFO("%d", walls[i]);
    }
    ROS_INFO("]");

    ROS_INFO("wall_nums = [ ");
    for(int i = 0; i < wall_nums.size(); i++){
        ROS_INFO("%d", wall_nums[i]);
    }
    ROS_INFO("]");
}

/* bool contains(int arr[], int size, int value) {
    // Use std::find to search for the value in the array
    return std::find(arr, arr + size, value) != arr + size;
} */

bool contains(const std::vector<int>& vec, int value) {
    // Use std::find to search for the value in the vector
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}

bool compareWalls(red_fundamentals::Cell cell, int orientation){
    if(wall_nums.size() != cell.walls.size()) return false;
    bool walls_robot[4] = {false,};
    for(int wall : cell.walls){
        // we convert wall direction in the map to one the robot is facing
        switch (orientation) {
            case 0:
                walls_robot[(wall + 1) % 4] = true;
                break;
            case 1:
                walls_robot[wall] = true;
                break;
            case 2:
                walls_robot[(wall + 3) % 4] = true;
                break;
            case 3:
                walls_robot[(wall + 2) % 4] = true;
                break;
            default:
                break;
        }
    }

    for(int i=0; i<3; i++){
        if(walls[i] != walls_robot[i]) return false;
    }

    // Since we only call this function after we move from one cell to another cell,
    // it is not possible that there is a wall behind us
    if(walls_robot[3]) return false;
    return true;
}

void localization() {
    // after centering, we have to rotate and check if there is a wall behind the robot
    // rotate so that we do not have a wall behind us
    // maybe we can put this part outside of this function
    Moving mover;
    DiffDriving driver;
    if(!ready){
        // Rotate 180
        // mover.turnAround();
	driver.turn(180);
        // check if there is a wall
        double dist = laserCallback();
        ROS_INFO("Distace = %f", dist);
        // if not, rotate 180
        if(isnan(dist) || dist > 0.4){
            ROS_INFO("No wall behind detected! Turning back again");
            // mover.turnAround();
	    driver.turn(180);
            
        }
        // else rotate so that we do not have a wall behind us -> how?
        else{
            ROS_INFO("Wall behind detected!");
            int no_wall = 0;
            while(no_wall <3){
                if(!walls[no_wall]){
                    break;
                }
                no_wall ++;
            }
            ROS_INFO("No wall at %d", no_wall);
            switch (no_wall) {
                // rotate so that no_wall becomes BACK and update walls
                case 0: // no wall on the right BEFORE 180 turn -> turn 90 to the right
                    ROS_INFO("Turn 90 degrees to the right");
                    // mover.turnRA(1);
		    driver.turn(90);
                    walls[0] = walls[1];
                    walls[1] = walls[2];
                    walls[2] = true;
                    break;
                case 1: // no wall in front of the robot BEFORE 180 turn -> stay
                    ROS_INFO("Stay");
                    swap(walls[0], walls[2]);
                    walls[1] = true;
                    break;
                case 2: // no wall on the left BEFORE 180 turn -> turn 90 to the left
                    ROS_INFO("Turn 90 degrees to the left");
                    // mover.turnRA(0);
		    driver.turn(-90);
                    walls[2] = walls[1];
                    walls[1] = walls[0];
                    walls[0] = true;
                    break;
                default:
                    break;
            }
            wall_nums.clear();
            for(int i=0; i<3; i++){
                if(walls[i]) wall_nums.push_back(i);
            }

            ROS_INFO("walls = [ ");
            for(int i = 0; i < 3; i++){
                ROS_INFO("%d", walls[i]);
            }
            ROS_INFO("]");

            ROS_INFO("wall_nums = [ ");
            for(int i = 0; i < wall_nums.size(); i++){
                ROS_INFO("%d", wall_nums[i]);
            }
            ROS_INFO("]");
        }
        ready = true;
    }

    // First, we compare the current candidates with the walls we received
    // if potential_positions is empty, compare the number of walls
    // All cells in the map with the same amount of walls are potential positions
    while (ros::ok()) {
        if (potential_positions.empty()) {
            // Go through the rows
            for (size_t i = 0; i < map_cur.rows.size(); ++i) {
                const red_fundamentals::Row &row = map_cur.rows[i];

                // Go through the cells
                for (size_t j = 0; j < row.cells.size(); ++j) {
                    const red_fundamentals::Cell &cell = row.cells[j];
                    ROS_INFO("At coordinates (%lu, %lu), there are %lu walls", i, j, cell.walls.size());
                    // Same amount of walls get pushed back as potential position
                    // if NumWalls == 2 -> compare the difference of wall numbers
                    // since we do not know the orientation, save -1 for it
                    if (wall_nums.size() == cell.walls.size()) {
                    	if (wall_nums.size() != 2 ||
                    	abs(wall_nums[0] - wall_nums[1]) % 2 == abs(cell.walls[0] - cell.walls[1]) % 2) {
                    		int orientation = -1;
                            	if(wall_nums.size() == 1){
			 // save orientation if we can (in case there is only ONE wall)
			 		switch((wall_nums[0] + 4 - cell.walls[0]) % 4){
			 			case 0:
			 				orientation = 1;
			 				break;
			 			case 1:
			 				orientation = 0;
			 				break;
			 			case 2:
			 				orientation = 3;
			 				break;
			 			case 3:
			 				orientation = 2;
			 				break;
			 			default:
			 				break;
			 		}
				}	
                        potential_positions.emplace_back(make_pair(i, j), orientation);
                        }
                    }
                }
            }

            //Output of all the potential positions:
            ROS_INFO("Potential Positions: ");
            for (size_t i = 0; i < potential_positions.size(); ++i) {
                ROS_INFO("(%d, %d)", potential_positions[i].first.first, potential_positions[i].first.second);
            }

        }


            // else: we already have candidates!
            // Go through the potential positions compare the number of walls AND their orientation
        else {
            for (auto candidate = potential_positions.begin(); candidate != potential_positions.end();) {
                red_fundamentals::Cell cell = map_cur.rows[candidate->first.first].cells[candidate->first.second];
                if (!compareWalls(cell, candidate->second)) {
                    candidate = potential_positions.erase(candidate);
                } else {
                    candidate++;
                }
            }

            ROS_INFO("Potential Positions (After compareWalls: ");
            for (size_t i = 0; i < potential_positions.size(); ++i) {
                ROS_INFO("(%d, %d), (%d)", potential_positions[i].first.first, potential_positions[i].first.second, potential_positions[i].second);
            }
        }

	

        // if there is only one potential position left and the orientation is known ( != -1), return and publish it to /pose
        if (potential_positions.size() == 1 && potential_positions[0].second != -1) {
            red_fundamentals::Pose pose;
            pose.row = potential_positions[0].first.first;
            pose.column = potential_positions[0].first.second;
            pose.orientation = potential_positions[0].second;
            
	    // TODO play a song
	    for(int i=0; i<3; i++){
	    	pose_pub.publish(pose);
            	ROS_INFO("Pose message %d time(s) published", i);
		ros::Duration(1.0).sleep();
	    }
            break;
        } else {
            // Let's go through the potential positions
            // Looking at a potential position: Which cells can I reach from here?

            vector<pair<pair<int, int>, int>> reachable_cells;

            for (auto potential_position: potential_positions) {

                int row = potential_position.first.first;
                int col = potential_position.first.second;

                vector<int> cell = map_cur.rows[row].cells[col].walls;


                if (!contains(cell, 0)) {
                    // if we don't have a 0, then a reachable cell would be in the map at (row, col +1)
                    if (map_cur.rows[0].cells.size() <= col + 1) {
                        ROS_INFO("map_cur.rows[0].cells.size() = %lu", map_cur.rows[0].cells.size());
                        ROS_INFO("Map Error ?");
                    } else {
                        reachable_cells.emplace_back(std::make_pair(row, col + 1), 0);
                    }

                }

                if (!contains(cell, 1)) {
                    // if we don't have a 1, then a reachable cell would be in the map at (row -1, col)
                    if (0 > row - 1) {
                        ROS_INFO("Map Error ?");
                    } else {
                        reachable_cells.emplace_back(std::make_pair(row - 1, col), 1);
                    }
                }

                if (!contains(cell, 2)) {
                    // if we don't have a 2, then a reachable cell would be in the map at (row, col -1)
                    if (0 > col - 1) {
                        ROS_INFO("Map Error ?");
                    } else {
                        reachable_cells.emplace_back(std::make_pair(row, col - 1), 2);
                    }
                }

                if (!contains(cell, 3)) {
                    // if we don't have a 3, then a reachable cell would be in the map at (row +1, col)
                    if (map_cur.rows.size() <= row + 1) {
                        ROS_INFO("map_cur.rows.size() = %lu", map_cur.rows.size());
                        ROS_INFO("Map Error ?");
                    } else {
                        reachable_cells.emplace_back(std::make_pair(row + 1, col), 3);
                    }
                }
            }

            // save all reachable cells as next potential positions
            potential_positions.clear();
            potential_positions = reachable_cells;

            //Output of all the potential positions:
            ROS_INFO("Potential Positions (Reachable Cells): ");
            for (size_t i = 0; i < potential_positions.size(); ++i) {
                ROS_INFO("(%d, %d), (%d)", potential_positions[i].first.first, potential_positions[i].first.second, potential_positions[i].second);
            }
        }

        // compute the direction in which we want to move
        // we just choose the smallest possible direction (where we couldn't find a wall)
        int move_direction = 0;

        for (int i = 0; i < 4; i++) {
            if (!contains(wall_nums, i)) {
                move_direction = i;
                break;
            }
        }

        ROS_INFO("Move in direction %d", move_direction);

        // move into right direction

        switch (move_direction) { // relative to the direction the robot is heading
            case 0: // RIGHT
                // mover.turnRA(1);
		driver.turn(90);
                mover.moveDist(0.8);
                break;
            case 1: // UP
                mover.moveDist(0.8);
                break;
            case 2: // LEFT
            	// mover.turnRA(0);
		driver.turn(-90);
                mover.moveDist(0.8);
                break;
            case 3: // DOWN
                // mover.turnAround();
		driver.turn(180);
                mover.moveDist(0.8);
                break;
            default:
                ROS_INFO("Wrong Direction");
        }


        // get walls
        wallCallback();
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("map", 1000, mapCallback);
    pose_pub = n.advertise<red_fundamentals::Pose>("/pose", 1);

    // Get the map
    /*  while(!receivedMap){
        ros::spinOnce();
    } */
    mapCallback();

    // Get walls
    wallCallback();
    localization();

    return 0;

}

