#ifndef CENTER_H
#define CENTER_H
#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "red_fundamentals/Walls.h"
#include "red_fundamentals/WallsCombined.h"

class Center
{
	public:
        int isPerpendicular;
        int distCorrect;
        int turnCtr;
        int noPerp;
        int centeringPhase;		//0 if still orienting to first wall; 1 if orienting to second wall; 2 if centered
        int heading;			//current heading of robot; 0 = north, 1 = east, 2 = south, 3 = west

        double tolerance;	    //tolerance value for angle to wall
        double speed;		    //motor speed
        double moveIncrement;	//time to move each increment

        ros::NodeHandle n;
        ros::ServiceClient diffDrive;
        create_fundamentals::DiffDrive srv;
        ros::Subscriber sub;
        
        void stop();
        void moveDist(int direction, float distance);
        void turnRightAngle(int direction);
        void turnAngle(int direction, double angle);
        void perpendicularise(const red_fundamentals::Walls::ConstPtr& msg);
        void correctDist(const red_fundamentals::Walls::ConstPtr& msg);
        void center(const red_fundamentals::Walls::ConstPtr& msg);
        void centering();

        Center();
};

#endif
