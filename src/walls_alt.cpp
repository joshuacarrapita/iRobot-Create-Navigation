#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "red_fundamentals/Walls.h"
#include <cstdlib>

#define LARRSIZE 726
#define LARRRIGHTEND 272
#define LARRFRONTEND 498
#define LARROFFSET 44
#define LARRACTSIZE 682
#define ANGLESTEP 0.351906
#define LARRUSED 512
#define LARRUSEDOFFSET 85
#define ANGLESTEPUSED 0.351563
#define TOTALOFFSET 129

using namespace std;

double errThresh = 0.02;
int candidateDist = 5;
int winnerThresh = 70;

const int num_rows = 360; // 0 bis 2pi entspricht 360 Grad
const int num_cols = 1100;

vector<vector<int>> houghRaum(num_rows, vector<int>(num_cols)); // Hough Raum


ros::Publisher pub;
red_fundamentals::Walls wall;

typedef struct Coords
{
	double x;	//for polar = r
	double y;	//for polar = angle
} coords;

const coords robotpos = {.x = 0, .y = 0};


//converts polar to cart coordinates
coords polToCart(coords polar)
{
	//ROS_INFO("Converting polar coordinate with r = %f and angle = %f", polar.x, polar.y);
	coords cart;
	cart.x = polar.x * cos(polar.y);
	cart.y = polar.x * sin(polar.y);
	//ROS_INFO("To cartesian coordinate with (%f , %f)", cart.x, cart.y);
	return cart;
}

//calculates angle of distance measurement in radians
double calcSensAngle(int index)
{
	//return((index * ANGLESTEP/240)* ((double) M_PI));
	return((-30.0 + ((index * 240)/682)) * ((double) M_PI /180));
}

//calculates distance between point and line given through 2 points
double calcDist(coords pt_a, coords pt_b, coords pt_0)
{
	return (abs((pt_b.x-pt_a.x)*(pt_0.y-pt_a.y)-(pt_0.x-pt_a.x)*(pt_b.y-pt_a.y))/sqrt((pt_b.x-pt_a.x)*(pt_b.x-pt_a.x) + (pt_b.y-pt_a.y)*(pt_b.y-pt_a.y)));
}

//calculates slope and y-axis intercept point of line given 2 points
coords calcLineEquation(coords pt_a, coords pt_b)
{
	double slope = (pt_b.y - pt_a.y)/(pt_b.x-pt_a.x);
	double y_inter = pt_a.y - slope * pt_a.x;
	return {.x = slope, .y = y_inter};
}

//calculates angle relative to y axis via 2 winner points
double calcAngle(coords pt_a, coords pt_b)
{
	//double ydiff = max(pt_a.y,pt_b.y) - min(pt_a.y,pt_b.y);
	int signer = 1;
	//if (pt_a.x < 0) { signer = -1; }
	double ydiff = pt_a.y-pt_b.y;
	return signer*((acos(ydiff/sqrt((pt_a.x-pt_b.x)*(pt_a.x-pt_b.x) + ydiff*ydiff))) * (180/(double) M_PI));
}

//calculates angle relative to y axis via slope of line
double calcAngleAlt(double slope)
{
	return (cos(slope)/sin(slope));
}

void resetMSG()
{
	//ROS_INFO("Resetting msg vals");
	//reset bools
	wall.walls[0] = false;
	wall.walls[1] = false;
	wall.walls[2] = false;
	//reset coords
	wall.R_A[0] = 0.0;
	wall.R_A[1] = 0.0;
	wall.R_B[0] = 0.0;
	wall.R_B[1] = 0.0;
	wall.F_A[0] = 0.0;
	wall.F_A[1] = 0.0;
	wall.F_B[0] = 0.0;
	wall.F_B[1] = 0.0;
	wall.L_A[0] = 0.0;
	wall.L_A[1] = 0.0;
	wall.L_B[0] = 0.0;
	wall.L_B[1] = 0.0;
	// reset distance and angle to front wall
	wall.angle_f = 0.0;
	wall.dist_f = 0.0;
	return;
}

/*
//simplified RANSAC making use of environment constraints
void sransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	vector<coords> coordinates;
	coords pol;
	//int j = 0;
	//get all valid measurements and convert to cartesian
	for (int i = (TOTALOFFSET-1); i < LARRSIZE - (LARRUSEDOFFSET+1); i++) {
		if (isnan(msg->ranges[i]) && msg->ranges[i] < msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (TOTALOFFSET-1));
		coordinates.push_back(polToCart(pol));
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", j, coordinates[j].x, coordinates[j].y);
		//j++;
	}
	//actual SRANSAC start
	int maxInliers = 0;
	coords winner_a;
	coords winner_b;
	//take candidates with fixed interval instead of random
	for (int i = 0; i < coordinates.size()-(candidateDist+1); i+=candidateDist) {
		int curInliers = 0;
		coords candidate_a = coordinates[i];
		coords candidate_b = coordinates[i+candidateDist];
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
			//ROS_INFO("winner found with %i inliers", curInliers);
		}
	}
	wall.dist_f = calcDist(winner_a, winner_b, robotpos);
	wall.angle_f = calcAngle(winner_a, winner_b) - 20;
	//wall.angle = calcAngleAlt(calcLineEquation(winner_a, winner_b).x);
	ROS_INFO("Best wall with %i inliers has dist: %.7lf angle: %.7lf", maxInliers, wall.dist_f, wall.angle_f);
}*/


// Funktion, um Indizes des größten Wertes zu finden
pair<int, int> findMaxValueIndex(vector<vector<int>>& array) {
    int maxVal = array[0][0]; // Annahme: Das erste Element ist der vorläufige größte Wert
    pair<int, int> maxIndex = make_pair(0, 0); // Die Indizes des vorläufig größten Werts
    
    // Durchlaufe das gesamte Array
    for (size_t i = 0; i < array.size(); ++i) {
        for (size_t j = 0; j < array[i].size(); ++j) {
            // Überprüfe, ob das aktuelle Element größer als der bisher größte Wert ist
            if (array[i][j] > maxVal) {
                maxVal = array[i][j]; // Aktualisiere den größten Wert
                maxIndex = make_pair(i, j); // Aktualisiere die Indizes des größten Werts
            }
        }
    }
    
    return maxIndex; 
}


void houghTransform(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
  // Hough Raum auf 0 setzen
  for (int i = 0; i < num_rows; ++i) {
    for (int j = 0; j < num_cols; ++j) {
        houghRaum[i][j] = 0;
    }
  }
  
  for(int i = 0; i < msg->ranges.size(); i++){
    // Durch die Datenpunkte iterieren
    if (!isnan(msg->ranges[i]) && msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max){

      float angle = msg->angle_min + (msg->angle_increment * i);
      
      // Winkel positiv machen (z.B. -pi/2 (90) wird zu 3pi/2 (270))
      if(angle < 0){
        angle += 2 * M_PI;
      }
      
      // Wir drehen den Winkel um 90 Grad 
      angle += 0.5 * M_PI;
      
      float x = cos(angle) * msg->ranges[i];
      float y = sin(angle) * msg->ranges[i];

      for(int alpha = 0; alpha < num_rows; alpha ++){
        
        // Winkel umrechnen
        double radians = (alpha * M_PI) / 180.0;

        
        // Abstand berechnen
        double d = (x * cos(radians) + y * sin(radians)) * 1000;

        // Index berechnen
        int d_index = round(abs(d));
        
        houghRaum[alpha][d_index]++;
      }
    } else {
      //ROS_INFO("Nicht betrachteter Datenpunkt: %f", msg->ranges[i]);
    }
  }

  // Suche nach dem höchsten Wert im houghRaum
  
  pair<int, int> maxIndex = findMaxValueIndex(houghRaum);

  wall.angle_f = maxIndex.first;
  wall.dist_f =  maxIndex.second / 1000.0;
  
}


//simplified RANSAC making use of environment constraints
void extendedSransac(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//reset walls msg contents
	resetMSG();

	//array for holding valid coordinates
	vector<coords> coordinates;
	coordinates.reserve(683);
	coords pol;

	//get all valid measurements and convert to cartesian
	for (int i = LARROFFSET; i < LARRSIZE; i++) {
		if (isnan(msg->ranges[i]) | msg->ranges[i] <= msg->range_min) {
			continue;
		}
		pol.x = msg->ranges[i];
		pol.y = calcSensAngle(i - (LARROFFSET-1));
		coordinates.push_back(polToCart(pol));
		//ROS_INFO("Generated cartesian coordinate %i with (%f , %f)", lens[0], coordinates[0][(lens[0])].x, coordinates[0][(lens[0])].y);
	}

	//actual SRANSAC start
	int curInliers;
	int maxInliers;

	vector<coords> outliers;
	outliers.reserve(683);
	vector<coords> winneroutliers;
	winneroutliers.reserve(683);
	vector<coords> inliers;
	inliers.reserve(683);

	coords winners[3][2];
	int winnerInliers[3] = {};

	//look for 3 best walls
	for (int k = 0; k < 3; k++) {
		//if not enough candidates stop looking for walls
		if (coordinates.size() < winnerThresh) {
			//ROS_INFO("not enough candidates");
			break;
		}
		coords winner_a = robotpos;
		coords winner_b = robotpos;
		maxInliers = 0;
		//ROS_INFO("looking for wall %i with %d candidates", k,coordinates.size());
		//take candidates with fixed interval instead of random
		for (int i = 0; i < coordinates.size()-(candidateDist+1); i+=candidateDist) {
			//ROS_INFO("checking candidates %i", i);
			curInliers = 0;
			coords candidate_b = coordinates[i];
			coords candidate_a = coordinates[i+candidateDist];
			//count number of inliers
			for (int j = 0; j < coordinates.size(); j++) {
				if (calcDist(candidate_a, candidate_b, coordinates[j]) < errThresh) {
					curInliers++;
					inliers.push_back(coordinates[j]);
				} else {
					outliers.push_back(coordinates[j]);
				}
			}
			//update winner if current candidates have more inliers than previous best and threshold and remove inliers
			if (curInliers > maxInliers && curInliers > winnerThresh) {
				maxInliers = curInliers;
				winneroutliers = outliers;
				winner_a = inliers.back();
				winner_b = inliers.front();
			}
			outliers.clear();
			inliers.clear();
		}
		if (maxInliers > winnerThresh) {
			//ROS_INFO("Wall %d found",k);
			winners[k][0] = winner_a;
			winners[k][1] = winner_b;
			winnerInliers[k] = maxInliers;
		}
		coordinates = winneroutliers;
	}
	//classify walls
	for (int i = 0; i < 3; i++) {
		if (winnerInliers[i]) {
			double ang = calcAngle(winners[i][0],winners[i][1]);
			//ROS_INFO("Wall %d has angle: %lf",i,ang);
			if (ang > 120) {
				//ROS_INFO("Left wall found");
				if (winners[i][0].y < 0.09) {
					wall.walls[2] = true;
				}
				wall.L_A[0] = winners[i][0].x;
				wall.L_A[1] = winners[i][0].y;
				wall.L_B[0] = winners[i][1].x;
				wall.L_B[1] = winners[i][1].y;
			} else if (ang < 60) {
				//ROS_INFO("Right wall found");
				if (winners[i][1].y < 0.09) {
					wall.walls[0] = true;
				}
				wall.R_A[0] = winners[i][0].x;
				wall.R_A[1] = winners[i][0].y;
				wall.R_B[0] = winners[i][1].x;
				wall.R_B[1] = winners[i][1].y;
			} else {
				//ROS_INFO("Front wall found");
				if (winners[i][0].y < 0.5) {
					wall.walls[1] = true;
				}
				wall.F_A[0] = winners[i][0].x;
				wall.F_A[1] = winners[i][0].y;
				wall.F_B[0] = winners[i][1].x;
				wall.F_B[1] = winners[i][1].y;
			}

		}
	}
	//sransac(msg);
	houghTransform(msg);
	
	pub.publish(wall);
}



int main(int argc, char **argv)
{
	if (argc != 4) {
		ROS_ERROR("Usage: walls <candidateDist> <errThresh> <winnerThresh>");
		ROS_INFO("Setting default values...");
	} else {
		candidateDist = atoi(argv[1]);
		errThresh = atof(argv[2]);
		winnerThresh = atoi(argv[3]);
	}
	ros::init(argc, argv, "walls");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan_filtered", 1, extendedSransac);
	pub = n.advertise<red_fundamentals::Walls>("walls_topic", 1);
	ROS_INFO("start detecting walls");
	while(ros::ok()) {
		ros::spinOnce();
		//ros::Duration(0.5).sleep();
	}
	return 0;
}

