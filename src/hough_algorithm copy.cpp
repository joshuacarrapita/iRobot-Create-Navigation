#include "ros/ros.h"
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;
#include "sensor_msgs/LaserScan.h"

// Globale Variablen
const int num_rows = 360; // 0 bis 2pi entspricht 360 Grad
const int num_cols = 1100;

bool laserData = false;

vector<vector<int>> houghRaum(num_rows, vector<int>(num_cols)); // Hough Raum

// Funktion, um die Indizes der vier größten Werte zu finden
void findMaxValues(int maxValues[][2]) {
  // Array initialisieren
  for (int i = 0; i < 4; ++i) {
      maxValues[i][0] = -1;
      maxValues[i][1] = -1; 
  }

  // Durchlaufe den gesamten Hough-Raum
  for (int row = 0; row < num_rows; ++row) {
      for (int col = 0; col < num_cols; ++col) {
          // Überprüfe, ob der aktuelle Wert größer ist als der bisher größte gefundene Wert
          for (int i = 0; i < 4; ++i) {
              if (houghRaum[row][col] > houghRaum[maxValues[i][0]][maxValues[i][1]]) {
                  // Verschiebe die Indizes der anderen größten Werte nach unten
                  for (int j = 3; j > i; --j) {
                      maxValues[j][0] = maxValues[j - 1][0];
                      maxValues[j][1] = maxValues[j - 1][1];
                  }
                  // Speichere die Indizes des neuen größten Werts
                  maxValues[i][0] = row;
                  maxValues[i][1] = col;
                  break; // Beende die Schleife, nachdem der Wert eingefügt wurde
              }
          }
      }
  }
}

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

  laserData = true;
  // Hough Raum auf 0 setzen
  for (int i = 0; i < num_rows; ++i) {
    for (int j = 0; j < num_cols; ++j) {
        houghRaum[i][j] = 0;
    }
  }

  // wir gehen durch die Datenpunkte durch, die der Roboter vor sich sieht
  // Wir fangen bei -90 Grad an und gehen bis 90 Grad weiter 
  // msg->ranges[128] misst bei -90 Grad 
  // msg->ranges[384] misst bei 0 Grad (geradeaus)
  // msg->ranges[639] misst bei 90 Grad
  
  for(int i = 0; i < msg->ranges.size(); i++){
    // Durch die Datenpunkte iterieren
    if (!isnan(msg->ranges[i]) && msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max){

      float angle = msg->angle_min + (msg->angle_increment * i);
      ROS_INFO("Angle = %f", angle);
      
      // Winkel positiv machen (z.B. -pi/2 (90) wird zu 3pi/2 (270))
      if(angle < 0){
        angle += 2 * M_PI;
        ROS_INFO("Positiv machen: Angle = %f", angle);
      }
      
      // Wir drehen den Winkel um 90 Grad 
      angle += 0.5 * M_PI;
      ROS_INFO("Um 90 Grad drehen: Angle = %f", angle);
      
      float x = cos(angle) * msg->ranges[i];
      float y = sin(angle) * msg->ranges[i];

      /* if(x < 0){
        ROS_INFO("x kleiner als 0");
      }

      if(y < 0){
        ROS_INFO("y kleiner als 0");
      } */

      for(int alpha = 0; alpha < num_rows; alpha ++){
        
        // Winkel umrechnen
        double radians = (alpha * M_PI) / 180.0;

        
        // Abstand berechnen
        double d = (x * cos(radians) + y * sin(radians)) * 1000;

        /* if(d < 0){
          ROS_INFO("Abstand kleiner als 0");
        } */
        

        // Index berechnen
        int d_index = round(abs(d));
        
        houghRaum[alpha][d_index]++;
      }
    } else {
      ROS_INFO("Nicht betrachteter Datenpunkt: %f", msg->ranges[i]);
    }
  }

  // Suche nach dem höchsten Wert im houghRaum
  
  pair<int, int> maxIndex = findMaxValueIndex(houghRaum);

  int angle_act = maxIndex.first;
  int distance =  maxIndex.second;

  //ROS_INFO("Winkel = %d | Abstand =  %d", angle_act, distance);

  int angle_des = 90;

  if(angle_act > angle_des){
    // nach links drehen
  } else if(angle_act < angle_des){
    // nach rechts drehen
  }

  /* // Suche nach den 4 Höchsten werte im houghRaum

  int maxValues[4][2];

  findMaxValues(maxValues);

  // Vier größten Werte ausgeben
  cout << "Die vier größten Werte in der Matrix sind:\n";
  for (int i = 0; i < 4; ++i) {
      cout << "Wert " << i << ": " << houghRaum[maxValues[i][0]][maxValues[i][1]] << " an Index (" << maxValues[i][0] << ", " << maxValues[i][1] << ")\n";
  } */
  
}
    

int main(int argc, char **argv){
    
    ros::init(argc, argv, "hough_algorithm");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("scan_filtered", 1, houghTransform);

    while(!laserData){
      ros::spinOnce();
    }
    return 0;
}

