#include <cstdlib>
#include <cmath>
#include "ros/ros.h"
#include "red_fundamentals/ExecutePlan.h"
#include "create_fundamentals/DiffDrive.h"
#include "sensor_msgs/LaserScan.h"
#include "moving.hpp"
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
#include "red_fundamentals/Pose.h"
#include "red_fundamentals/WallsAlt.h"
#include "paradriver.hpp"
#include "song.hpp"
#include "center.hpp"

using namespace std;

const int laser_msg_offset = 44;
const double wait_wall_msg = 10;
const double safe_wall_dist = 0.8;

enum drive_direction { RIGHT = 0, UP = 1, LEFT = 2, DOWN = 3 };
enum turn_direction { RIGHT_TURN = 1, ALSO_RIGHT_TURN = -3, NO_TURN = 0, LEFT_TURN = -1, ALSO_LEFT_TURN = 3, BACK_TURN = 2, ALSO_BACK_TURN = -2 };

red_fundamentals::Grid map_cur;
ros::Publisher pose_pub;

bool ready = false;
bool need_change = false;

// <row, column>, orientation
set<pair<pair<int, int>, int>> potential_positions;
set<set<pair<pair<int,int>, int>>> candidates_seen;

bool walls[3] = {false,};
vector<int> wall_nums;

red_fundamentals::Pose curr_pose;
red_fundamentals::Pose next_pose;

double lost = true;

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
        const red_fundamentals::WallsAlt::ConstPtr msg = ros::topic::waitForMessage<red_fundamentals::WallsAlt>("walls_alt_topic",
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
    ROS_INFO("We are here");
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
            curr_pose.row = potential_positions.begin()->first.first;
            curr_pose.column = potential_positions.begin()->first.second;
            curr_pose.orientation = potential_positions.begin()->second;
			Song s;
            s.playLocalizedSound();
            for(int i=0; i<3; i++){
                pose_pub.publish(curr_pose);
                ROS_INFO("Pose message %d time(s) published", i);
                ros::Duration(1.0).sleep();
            }
			ROS_INFO("We are now at (%d, %d) looking %d", curr_pose.row, curr_pose.column, curr_pose.orientation);
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
        ros::Duration(1.0).sleep();
        wallCallback();
    }
}

double wall_distance_check(){
	sensor_msgs::LaserScan::ConstPtr laser_message = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan_filtered", ros::Duration(wait_wall_msg));

	if (!laser_message) return -1; // return if "scan_filtered" didn't provide a message

 	// adjust for the first unusable values
	int valid_ranges_size = laser_message->ranges.size()-laser_msg_offset;

	/* The wanderer code scanned too wide of a radius 
	 * so I chose a different option.
	 * We could take the average over a few values in the middle if 
	 * we want to be sure we dont hit an outlier but this 
	 * narrow radius is generally more reliable.
	 */
	// get the distance in front of the robot
	double dist = laser_message->ranges[valid_ranges_size/2+laser_msg_offset]; 

	// if it is below the legal distance we can return something that is safe
	if (dist < laser_message->range_min) return safe_wall_dist+0.1; 

	return dist; // otherwise we return the distance for comparison
}

void getNextPose(int next_action){
	
	switch(next_action){
		// RIGHT
		case 0:
			next_pose.row = curr_pose.row;
			next_pose.column = curr_pose.column + 1;
			break;
		// UP
		case 1:
			ROS_INFO("case 1");
			next_pose.row = curr_pose.row - 1;
			ROS_INFO("next_pose.row = curr_pose.row - 1 = %d - 1", curr_pose.row);
			next_pose.column = curr_pose.column;
			ROS_INFO("next_pose.column = curr_pose.column = %d", curr_pose.column);
			break;
		// LEFT
		case 2:
			next_pose.row = curr_pose.row;
			next_pose.column = curr_pose.column - 1;
			break;
		// DOWN
		case 3:
			next_pose.row = curr_pose.row + 1;
			next_pose.column = curr_pose.column;
			break;
		default:
			ROS_INFO("Not a correct action. Cant get next pose");
            return;
	}

	next_pose.orientation = next_action;
	return;
}

//bool execute_plan_callback(red_fundamentals::ExecutePlan::Request &req,
//          red_fundamentals::ExecutePlan::Response &res){

bool execute_plan_callback(red_fundamentals::ExecutePlan::Request &req,
          red_fundamentals::ExecutePlan::Response &res){

    Center* c = new Center();
    if(lost == true){
        
        c->centering();  // center
        delete c;
        mapCallback();	// Get map
        ros::Duration(1.0).sleep();
        wallCallback();	// Get walls
		
        potential_positions.clear();
		candidates_seen.clear();
        localization();	// Localize
    }

	// Initialize and reset variables between callbacks
    vector<int> exec_plan = req.plan;
	res.success = false;
	drive_direction robot_dir = static_cast<drive_direction>(curr_pose.orientation);
	Moving mover;
    DiffDriving driver;
	
	// go over the whole execution plan
	for (int i = 0; i < exec_plan.size(); i++){

		int next_action = exec_plan[i];
		robot_dir = static_cast<drive_direction>(curr_pose.orientation); // update robot direction
		int turn_dir = robot_dir - next_action; // dynamically calculate the necessary turn amount based on current robot direction and desired cell

		switch(turn_dir){

			case LEFT_TURN:
			case ALSO_LEFT_TURN:
				// do left 90° turn
				driver.turn(-90);
				break;
			
			case RIGHT_TURN:
			case ALSO_RIGHT_TURN:
				// do right 90° turn
				driver.turn(90);
				break;
			
			case BACK_TURN:
			case ALSO_BACK_TURN:
				// do right 180° turn
				driver.turn(180);
				break;

			default:
				break;
		
		}

		Song s;
		
		// before moving, check for wall; this also catches the case of trying to move up after centering
		double dist = wall_distance_check();
		if (dist < safe_wall_dist) {
			if (dist < 0) {
				ROS_INFO("I couldn't find any laser message!");
				return false;
			}
		    ROS_INFO("Almost hit a wall!");
			lost = true;
			s.playLostSound();
            ROS_INFO("We got lost, because we almost hit a wall");
			return true;
		}
		
		// before moving checking the cell we would be in
        ROS_INFO("Get next pose");
		getNextPose(next_action);
        ROS_INFO("Next Pose would be at (%d, %d) looking %d", next_pose.row, next_pose.column, next_pose.orientation);

        ROS_INFO("We are moving");
		mover.moveDist(0.8);

		// after moving we check if the walls we see match the walls in the map
		ros::Duration(1.0).sleep();
		wallCallback();	// Get walls

		red_fundamentals::Cell cell = map_cur.rows[next_pose.row].cells[next_pose.column];
		if (!compareWalls(cell, next_pose.orientation)) {
			// We got lost!
			lost = true;
            s.playLostSound();
            ROS_INFO("The walls dont match. We got lost");
			return true;
		} else {
			// We know where we are and publish
			curr_pose = next_pose;
			s.playLocalizedSound();
            for(int i=0; i<3; i++){
                pose_pub.publish(curr_pose);
                ROS_INFO("Pose message %d time(s) published", i);
                ros::Duration(1.0).sleep();
	        }
            ROS_INFO("We are now at (%d, %d) looking %d", curr_pose.row, curr_pose.column, curr_pose.orientation);
			lost = false;
		}

	}

	res.success = true;
	return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "execute_plan_server");
  ros::NodeHandle n;

  pose_pub = n.advertise<red_fundamentals::Pose>("/pose", 1);

  ros::init(argc, argv, "execute_plan_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("execute_plan", execute_plan_callback);

  ROS_INFO("Ready to run execute plan");

  ros::spin();
  
  return 0;
}
