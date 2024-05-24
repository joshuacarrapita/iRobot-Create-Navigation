#include "ros/ros.h"
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <iostream>
using namespace std;
#include "sensor_msgs/LaserScan.h"

// Globale Variablen
const int num_rows = 180; // 0 bis pi entspricht 180 Grad
const int num_cols = 1100; 

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
    
    return maxIndex; // Gib die Indizes des größten Werts zurück
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

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
  
  for(int i = 128; i < 640; i++){
    // Durch die Datenpunkte iterieren
    if (!isnan(msg->ranges[i])){


      float angle = msg->angle_min + (msg->angle_increment * i);
      // Wir drehen den Winkel um 90 Grad 
      angle += 0.5 * M_PI;
      
      float x = cos(angle) * msg->ranges[i];
      float y = sin(angle) * msg->ranges[i];

      if(y < 0){
        ROS_INFO("kann nicht sein");
      }

      for(int alpha = 0; alpha < num_rows; alpha ++){
        
        // Winkel umrechnen
        double radians = (alpha * M_PI) / 180.0;

        
        // Abstand berechnen
        double d = (x * cos(radians) + y * sin(radians)) * 1000;
        

        // Index berechnen
        int d_index = round(abs(d));
        
        houghRaum[alpha][d_index]++;
      }
    }
  }

  // Suche nach dem höchsten Wert im houghRaum
  
  pair<int, int> maxIndex = findMaxValueIndex(houghRaum);

  int angle_act = maxIndex.first;
  int distance =  maxIndex.second;

  ROS_INFO("Winkel = %d | Abstand =  %d", angle_act, distance);

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
    
    ros::init(argc, argv, "align");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("scan_filtered", 1, laserCallback);

    ros::spin();
    return 0;
}

