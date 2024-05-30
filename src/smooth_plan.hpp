#ifndef SMOOTH_PLAN_H
#define SMOOTH_PLAN_H
#include <vector>

enum Direction { RIGHT = 0, UP = 1, LEFT = 2, DOWN = 3};
enum Movement { RIGHT_CURVE = 0, GO_STRAIGHT = 1, LEFT_CURVE = 2, ROTATE = 3};

std::vector<int> get_direction(std::vector<int> plan);
std::vector<std::pair<int, double>> compute_plan(std::vector<int> directions);
void merge_plan(std::vector<std::pair<int, double>> &plan);

#endif
