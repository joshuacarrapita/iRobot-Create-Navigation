/*#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "create_fundamentals/ResetEncoders.h"
#include <cstdlib>
#include "paradriver.hpp"

using namespace std;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("%f", msg->ranges[msg->ranges.size()/2]);
}

double calculateDistance(pair<double, double> p1, pair<double, double> p2){
    double x1 = p1.first;
    double y1 = p1.second;
    double x2 = p2.first;
    double y2 = p2.second;

    double distance = sqrt((x1 - x2)² + (y1 - y2)²);
    return distance;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "potential_field");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
    DiffDriving driver;

    pair<double, double> posRobot(driver.xcord, driver.ycord); // we need to update position
    pair<double, double> posGoal; // where do we get this from?
    pair<double, double> posObst; // we need to update laser

    pair<double, double> obstVector (posRobot.first - posObst.first, posRobot.second - posObst.second);
    pair<double, double> goalVector (posRobot.first - posGoal.first, posRobot.second - posGoal.second);

    double distGoal = calculateDistance(posRobot, posGoal);
    double distObst = calculateDistance(posRobot, posObst);

    double v = 1.1;
    double k = 1.1;


    // Abstoßende Kraft

    double maxDistToObst = 0.2;
    
    double F_repulsive = v * (1/distObst - 1/maxDistToObst) * 1/(distObst²) * obstVector/distObst;

    // Anziehendes Potential
    
    double F_attractiveLin = - k * goalVector/distGoal; //linear
    
    double F_attractiveQuad = - k * goalVector; //quadratic

   return 0;
    
} */

// Test

using namespace std;
#include <iostream>
#include <iostream>
#include <cmath>
#include <utility>

double calculateDistance(pair<double, double> p1, pair<double, double> p2){
    double x1 = p1.first;
    double y1 = p1.second;
    double x2 = p2.first;
    double y2 = p2.second;

    double distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    return distance;
}

int main(int argc, char **argv)
{
    pair<double, double> posRobot(2, 2); // we need to update position
    pair<double, double> posGoal(3, 6); // where do we get this from?
    pair<double, double> posObst(3, 4); // we need to update laser

    pair<double, double> obstVector (posRobot.first - posObst.first, posRobot.second - posObst.second);
    pair<double, double> goalVector (posRobot.first - posGoal.first, posRobot.second - posGoal.second);

    double distGoal = calculateDistance(posRobot, posGoal);
    double distObst = calculateDistance(posRobot, posObst);

    double v = 1.1;
    double k = 1.1;


    // Abstoßende Kraft

    double maxDistToObst = 0.2;

    double factor = v * (1/distObst - 1/maxDistToObst) * 1/pow(distObst,2) * 1/distObst;
    
    pair<double, double> F_repulsive (factor*obstVector.first, factor*obstVector.second);

    // Anziehendes Potential

    factor = -k * 1/distGoal;
    
    pair<double, double> F_attractiveLin (factor*goalVector.first, factor*goalVector.second); //linear

    factor = -k;
    
    pair<double, double> F_attractiveQuad (factor*goalVector.first, factor*goalVector.second); //quadratic

    cout << "obstVector: (" << obstVector.first << ", " << obstVector.second << ")" << endl;
    cout << "goalVector: (" << goalVector.first << ", " << goalVector.second << ")" << endl;
    cout << "distGoal: " << distGoal << endl;
    cout << "distObst: " << distObst << endl;
    cout << "F_repulsive: (" << F_repulsive.first << ", " << F_repulsive.second << ")" << endl;
    cout << "F_attractiveLin: (" << F_attractiveLin.first << ", " << F_attractiveLin.second << ")" << endl;
    cout << "F_attractiveQuad: (" << F_attractiveQuad.first << ", " << F_attractiveQuad.second << ")" << endl;

   return 0;

}



