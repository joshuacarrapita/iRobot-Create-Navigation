#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/Walls.h"
#include <cstdlib>
#include <vector>

#define LARRSIZE 726
#define LARROFFSET 44
#define LARRACTSIZE 682
#define ANGLESTEP 0.351906
#define LARRUSED 512
#define LARRUSEDOFFSET 85
#define ANGLESTEPUSED 0.351563
#define TOTALOFFSET 129

using namespace std;

double errThresh = 0.02; // Toleranzgrenze für die Zuordnung von Punkten zu einer Linie
int candidateDist = 5; // smaller candidate distance 
int maxWalls = 3; // Maximum number of walls to detect

typedef struct Coords
{
    double x; // for polar = r
    double y; // for polar = angle
} coords;

const coords robotpos = {.x = 0, .y = 0};

//converts polar to cartesian coordinates
coords polToCart(coords polar)
{
    coords cart;
    cart.x = polar.x * cos(polar.y);
    cart.y = polar.x * sin(polar.y);
    return cart;
}

//calculates angle of distance measurement in radians
double calcSensAngle(int index)
{
    return((index * ANGLESTEPUSED / 180) * ((double)M_PI));
}

//calculates distance between point and line given through 2 points
double calcDist(coords pt_a, coords pt_b, coords pt_0)
{
    return (abs((pt_b.x - pt_a.x) * (pt_0.y - pt_a.y) - (pt_0.x - pt_a.x) * (pt_b.y - pt_a.y)) / sqrt((pt_b.x - pt_a.x) * (pt_b.x - pt_a.x) + (pt_b.y - pt_a.x) * (pt_b.y - pt_a.y)));
}

//calculates slope and y-axis intercept point of line given 2 points
coords calcLineEquation(coords pt_a, coords pt_b)
{
    double slope = (pt_b.y - pt_a.y) / (pt_b.x - pt_a.x);
    double y_inter = pt_a.y - slope * pt_a.x;
    return { .x = slope, .y = y_inter };
}

//calculates angle relative to y-axis via 2 winner points
double calcAngle(coords pt_a, coords pt_b)
{
    double ydiff = pt_a.y - pt_b.y;
    return ((acos(ydiff / sqrt((pt_a.x - pt_b.x) * (pt_a.x - pt_b.x) + ydiff * ydiff))) * (180 / (double)M_PI));
}

//simplified RANSAC making use of environment constraints
vector<red_fundamentals::Walls> sransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    vector<coords> coordinates;
    vector<coords> remaining_points;
    coords pol;
    vector<red_fundamentals::Walls> detected_walls;

    //get all valid measurements and convert to cartesian
    for (int i = (TOTALOFFSET - 1); i < LARRSIZE - (LARRUSEDOFFSET + 1); i++) {
        if (isnan(msg->ranges[i]) && msg->ranges[i] < msg->range_min) {
            continue;
        }
        pol.x = msg->ranges[i];
        pol.y = calcSensAngle(i - (TOTALOFFSET - 1));
        coordinates.push_back(polToCart(pol));
    }

    //iteratively apply SRANSAC to detect multiple walls
    for (int w = 0; w < maxWalls; w++) {
        if (coordinates.size() < candidateDist + 1) {
            break;
        }
        //actual SRANSAC start
        int maxInliers = 0;
        coords winner_a;
        coords winner_b;
        //take candidates with fixed interval instead of random
        for (int i = 0; i < coordinates.size() - (candidateDist + 1); i += candidateDist) {
            int curInliers = 0;
            coords candidate_a = coordinates[i];
            coords candidate_b = coordinates[i + candidateDist];
            //count number of inliers
            for (int j = 0; j < coordinates.size(); j++) {
                if (calcDist(candidate_a, candidate_b, coordinates[j]) < errThresh) {
                    curInliers++;
                }
            }
            //update winner if current candidates have more inliers than previous best
            if (curInliers > maxInliers) {
                maxInliers = curInliers;
                winner_a = candidate_a;
                winner_b = candidate_b;
            }
        }
        if (maxInliers > 0) {
            red_fundamentals::Walls wall;
            wall.dist = calcDist(winner_a, winner_b, robotpos);
            wall.angle = calcAngle(winner_a, winner_b);
            detected_walls.push_back(wall);

            // remove inliers from coordinates
            remaining_points.clear();
            for (int i = 0; i < coordinates.size(); i++) {
                if (calcDist(winner_a, winner_b, coordinates[i]) >= errThresh) {
                    remaining_points.push_back(coordinates[i]);
                }
            }
            coordinates = remaining_points;
        }
    }
    return detected_walls;
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        ROS_ERROR("Usage walls <candidateDist> <errThresh>");
        ROS_INFO("Setting default values...");
    } else {
        candidateDist = atoi(argv[1]);
        errThresh = atof(argv[2]);
    }
    ros::init(argc, argv, "multiple_walls");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, [](const sensor_msgs::LaserScan::ConstPtr& msg) {
        vector<red_fundamentals::Walls> detected_walls = sransac(msg);
        for (const auto& wall : detected_walls) {
            ROS_INFO("Detected wall at distance: %.7lf, angle: %.7lf", wall.dist, wall.angle);
        }
    });

    //ros::Publisher pub = n.advertise<red_fundamentals::Walls>("walls_topic", 1);

    while (ros::ok()) {
        ros::spinOnce();
        //pub.publish(wall);
        ros::Duration(0.3).sleep();
    }
    return 0;
}
