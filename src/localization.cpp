#include <utility>
#include <vector>
#include <set>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
#include "red_fundamentals/Pose.h"
#include "red_fundamentals/Walls.h"
#include "paradriver.hpp"
#include "moving.hpp"
#include "song.hpp"

using namespace std;

red_fundamentals::Grid map_cur;
ros::Publisher pose_pub;

bool ready = false;
bool need_change = false;

// <row, column>, orientation
set<pair<pair<int, int>, int>> potential_positions;
set<set<pair<pair<int,int>, int>>> candidates_seen;

bool walls[3] = {false,};
vector<int> wall_nums;

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

int get_next_direction(){
    sort(wall_nums.begin(), wall_nums.end());
    set<int> no_wall = {0, 1, 2};
    for(int wall : wall_nums){
        no_wall.erase(wall);
    }
    if(no_wall.empty()) return 3;
    if(no_wall.size() == 1 || !need_change) return *no_wall.begin();
    return *no_wall.rbegin();
}

bool is_contained(const set<pair<pair<int, int>, int>>& positions_old, const set<pair<pair<int, int>, int>>& positions) {
    for (const auto& pos : positions_old) {
        if (positions.find(pos) == positions.end()) {
            return false;
        }
    }
    return true;
}

bool duplicate_check(){
    for(const auto& candidates_old : candidates_seen){
        if(candidates_old == potential_positions || is_contained(candidates_old, potential_positions)) {
            return true;
        }
    }
    return false;
}

void localization() {
    // after centering, we have to rotate and check if there is a wall behind the robot
    // rotate so that we do not have a wall behind us
    // maybe we can put this part outside of this function
    Moving mover;
    DiffDriving driver;
    if(!ready){
        // Rotate 180
       driver.turn(180);
        // check if there is a wall
        double dist = laserCallback();
        ROS_INFO("Distace = %f", dist);
        // if not, rotate 180
        if(isnan(dist) || dist > 0.4){
            ROS_INFO("No wall behind detected! Turning back again");
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
                            potential_positions.insert(make_pair(make_pair(i, j), orientation));
                        }
                    }
                }
            }

            //Output of all the potential positions:
            ROS_INFO("Potential Positions: ");
            for (auto pos : potential_positions) {
                ROS_INFO("(%d, %d)", pos.first.first, pos.first.second);
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
            for (auto pos : potential_positions) {
                ROS_INFO("(%d, %d), (%d)", pos.first.first, pos.first.second, pos.second);
            }
        }



        // if there is only one potential position left and the orientation is known ( != -1), return and publish it to /pose
        if (potential_positions.size() == 1 && potential_positions.begin()->second != -1) {
            red_fundamentals::Pose pose;
            pose.row = potential_positions.begin()->first.first;
            pose.column = potential_positions.begin()->first.second;
            pose.orientation = potential_positions.begin()->second;
            Song s;
            s.playLocalizedSound();
            for(int i=0; i<3; i++){
                pose_pub.publish(pose);
                ROS_INFO("Pose message %d time(s) published", i);
                ros::Duration(1.0).sleep();
            }
            break;
        } else {

            // if we enter a cell with the same orientation, we have to choose another direction (?)
            // TODO instead of this, we can choose the move direction completely randomly -> delete this part and modify get_next_direction function!!
            if(!need_change) {
                if (duplicate_check()) need_change = true;
                candidates_seen.insert(potential_positions);
            }
            // Let's go through the potential positions
            // Looking at a potential position: Which cells can I reach from here?
            set<pair<pair<int, int>, int>> reachable_cells;

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
                        reachable_cells.insert(make_pair(make_pair(row, col + 1), 0));
                    }

                }

                if (!contains(cell, 1)) {
                    // if we don't have a 1, then a reachable cell would be in the map at (row -1, col)
                    if (0 > row - 1) {
                        ROS_INFO("Map Error ?");
                    } else {
                        reachable_cells.insert(make_pair(make_pair(row - 1, col), 1));
                    }
                }

                if (!contains(cell, 2)) {
                    // if we don't have a 2, then a reachable cell would be in the map at (row, col -1)
                    if (0 > col - 1) {
                        ROS_INFO("Map Error ?");
                    } else {
                        reachable_cells.insert(make_pair(make_pair(row, col - 1), 2));
                    }
                }

                if (!contains(cell, 3)) {
                    // if we don't have a 3, then a reachable cell would be in the map at (row +1, col)
                    if (map_cur.rows.size() <= row + 1) {
                        ROS_INFO("map_cur.rows.size() = %lu", map_cur.rows.size());
                        ROS_INFO("Map Error ?");
                    } else {
                        reachable_cells.insert(make_pair(make_pair(row + 1, col), 3));
                    }
                }
            }

            // save all reachable cells as next potential positions
            potential_positions.clear();
            potential_positions = reachable_cells;

            //Output of all the potential positions:
            ROS_INFO("Potential Positions (Reachable Cells): ");
            for (auto pos : potential_positions) {
                ROS_INFO("(%d, %d), (%d)", pos.first.first, pos.first.second, pos.second);
            }
        }

        // compute the direction in which we want to move
        // we just choose the smallest possible direction (where we couldn't find a wall)
        int move_direction = get_next_direction();

        ROS_INFO("Move in direction %d", move_direction);

        // move into right direction

        switch (move_direction) { // relative to the direction the robot is heading
            case 0: // RIGHT
                driver.turn(90);
                mover.moveDist(0.8);
                break;
            case 1: // UP
                mover.moveDist(0.8);
                break;
            case 2: // LEFT
                driver.turn(-90);
                mover.moveDist(0.8);
                break;
            case 3: // DOWN
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
    pose_pub = n.advertise<red_fundamentals::Pose>("/pose", 1);

    // Get the map
    mapCallback();

    // Get walls
    wallCallback();
    localization();

    return 0;

}