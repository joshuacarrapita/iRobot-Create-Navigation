#include "smooth_plan.hpp"

// R: 03, 10, 21, 32 0
// S: 00, 11, 22, 33 1
// L: 01, 12, 23, 30 2
// B: 02, 13, 20, 31 3

std::vector<int> get_direction(std::vector<int> plan){
    std::vector<int> directions;
    // first one will be needed for the rotation
    directions.push_back(plan[0]);
    for(int i=0; i<plan.size()-1; i++) {
        int dir_prev = plan[i];
        int dir = plan[i+1];

        if ((dir_prev - dir + 4) % 4 == 1) directions.push_back(RIGHT);
        else if (dir_prev == dir) directions.push_back(UP);
        else if ((dir - dir_prev + 4) % 4 == 1) directions.push_back(LEFT);
        else directions.push_back(DOWN);
    }
    return directions;
}


// 0: right curve 1: straight 2: left curve 3: rotate
std::vector<std::pair<int, double>> compute_plan(std::vector<int> directions) {
    std::vector<std::pair<int, double>> movements;
    // for the first movement we have to face the right direction
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

    // and then go straight for 40 cm
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
            	movements.emplace_back(GO_STRAIGHT, 0.4);
                movements.emplace_back(ROTATE, 180);
                movements.emplace_back(GO_STRAIGHT, 0.4);
                break;
        }
    }

    // move to the center of the last cell
    movements.emplace_back(GO_STRAIGHT, 0.4);

    /* TODO if we won't run the wall detection part every time we move from one cell to another
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


