#ifndef CENTER_PD_H
#define CENTER_PD_H
#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "paradriver.hpp"
#include "red_fundamentals/WallsCombined.h"


class Center
{
	public:
                enum Centphs
                {
                        LOOKINGFORFIRSTWALL = 0,
                        PERPENDICULARTOFIRSTWALL,
                        DISTANCEDTOFIRSTWALL,
                        LOOKINGFORSECONDWALL,
                        PERPENDICULARTOSECONDWALL,
                        DISTANCEDTOSECONDWALL,
                        CENTERED
                };

                Centphs centeringPhase;
                int heading;			//current heading of robot; 0 = north, 1 = east, 2 = south, 3 = west
                int turnCtr;                    //keeps track of how many times robot has turned to find a wall
                double tolerance;	        //tolerance value for angle to wall

                ros::NodeHandle n;
                ros::ServiceClient diffDrive;
                create_fundamentals::DiffDrive srv;
                ros::Subscriber sub;

                DiffDriving driver;
                
                void perpendicularise(const red_fundamentals::WallsCombined::ConstPtr& msg);
                void correctDist(const red_fundamentals::WallsCombined::ConstPtr& msg);
                void center(const red_fundamentals::WallsCombined::ConstPtr& msg);
                void centering();

                Center();
};

#endif
