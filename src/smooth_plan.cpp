#include "smooth_plan.hpp"

// R: 03, 10, 21, 32 RIGHT
// S: 00, 11, 22, 33 UP
// L: 01, 12, 23, 30 LEFT
// B: 02, 13, 20, 31 BACK

bool is_free(std::vector<int> walls, int direction){
	for(int w : walls){
		if(w == direction) return false;
	}
	return true;
}

bool is_executable(int row_tmp, int col_tmp, std::vector<int> plan, Map *map){
	for(int p : plan){
  		red_fundamentals::Cell tmp = map->map.rows[row_tmp].cells[col_tmp];
  		// check if there is a wall in the direction we want to move according to the plan
  		if(!is_free(tmp.walls, p)) return false;
  		
  		// if free, change temporal row and column values to those of the next cell
  		switch(p){
  			case 0:
  				col_tmp += 1;
  				if(col_tmp < 0 || col_tmp > map->num_columns) return false;
  				break;
  			case 1:
  				row_tmp -= 1;
  				if(row_tmp < 0 || row_tmp > map->num_rows) return false;
  				break;
  			case 2:
  				col_tmp -= 1;
  				if(col_tmp < 0 || col_tmp > map->num_columns) return false;
  				break;
  			case 3:
  				row_tmp += 1;
  				if(row_tmp < 0 || row_tmp > map->num_rows) return false;
  				break;
  			default:
  				break;
  		}
  	}
  	return true;
}

std::vector<int> get_direction(std::vector<int> plan){
    std::vector<int> directions;
    // first one will be needed for the rotation to look in the direction of travel
    directions.push_back(plan[0]);
    
    // we check for two consecutive values of input array
    // goal is to reduce rotation by using curves
    // ex) go to the cell above (1) and then to one to the right (0) -> RIGHT CURVE -> store RIGHT as direction
    
    for(int i=0; i<plan.size()-1; i++) {
        int dir_prev = plan[i];
        int dir = plan[i+1];
	// read the comments above to check the case
        if ((dir_prev - dir + 4) % 4 == 1) directions.push_back(RIGHT);
        else if (dir_prev == dir) directions.push_back(UP);
        else if ((dir - dir_prev + 4) % 4 == 1) directions.push_back(LEFT);
        else directions.push_back(DOWN);
    }
    return directions;
}

// compute the movement sequence using the directions
std::vector<std::pair<int, double>> compute_plan(std::vector<int> directions) {
    std::vector<std::pair<int, double>> movements;
    
    // for the first movement we have to rotate to face the right direction
    switch (directions[0]) {
        case RIGHT:
            movements.emplace_back(ROTATE, 90);
            break;
        case LEFT:
            movements.emplace_back(ROTATE, -90);
            break;
        case DOWN:
            movements.emplace_back(ROTATE, 180);
            break;
    }

    // and then go straight for 40 cm to be located at the border of the cell
    movements.emplace_back(GO_STRAIGHT, 0.4);

    for(int i=1; i<directions.size(); i++){
        switch (directions[i]){
            case RIGHT:
                movements.emplace_back(RIGHT_CURVE, 90);
                break;
            case UP:
                movements.emplace_back(GO_STRAIGHT, 0.8);
                break;
            case LEFT:
                movements.emplace_back(LEFT_CURVE, 90);
                break;
            case DOWN:
            // we first move back to the center of the cell -> rotate 180 degs -> move to the border
            	movements.emplace_back(GO_STRAIGHT, 0.4);
                movements.emplace_back(ROTATE, 180);
                movements.emplace_back(GO_STRAIGHT, 0.4);
                break;
        }
    }

    // move to the center of the last cell
    movements.emplace_back(GO_STRAIGHT, 0.4);

    /* if we won't run the wall detection part every time we move from one cell to another
    we can combine consecutive same movements into one 
    so that we can call the move function only once 
    first written as an independent function below */
    return movements;
}

void merge_plan(std::vector<std::pair<int, double>> &plan) {
    std::vector<std::pair<int, double>> tmp;
    for(auto &p : plan){
        if(!tmp.empty() && tmp.back().first == p.first){
            tmp.back().second = tmp.back().second + p.second;
        }
        else tmp.emplace_back(p);
    }
    std::swap(plan, tmp);
}


