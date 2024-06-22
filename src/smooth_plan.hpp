#ifndef SMOOTH_PLAN_H
#define SMOOTH_PLAN_H
#include <vector>
#include "red_fundamentals/Grid.h"
#include "red_fundamentals/Row.h"
#include "red_fundamentals/Cell.h"
#include "red_fundamentals/Pose.h"
#include "red_fundamentals/Walls.h"

enum drive_direction { RIGHT = 0, UP = 1, LEFT = 2, DOWN = 3};
enum Movement { RIGHT_CURVE = 0, GO_STRAIGHT = 1, LEFT_CURVE = 2, ROTATE = 3};

bool is_free(std::vector<int> walls, int direction);
bool is_executable(int row, int col, std::vector<int> plan, red_fundamentals::Grid map);
std::vector<int> get_direction(std::vector<int> plan);
std::vector<std::pair<int, double>> compute_plan(std::vector<int> directions);
void merge_plan(std::vector<std::pair<int, double>> &plan);

#endif
