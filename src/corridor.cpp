#include "ros/ros.h"
#include "moving.hpp"
#include <cstdlib>
#include <iostream>

using namespace std;

// Possible to define it up here?
Moving mover;

void drive(int move, int streak){
    switch (move) {
        case 0:
            cout << "Wir drehen uns nach rechts." << endl;
            //mover.turn(?);
            break;
        case 1:
            cout << "Wir drehen uns nach oben." << endl;
            //mover.turn(?);
            break;
        case 2:
            cout << "Wir drehen uns nach links." << endl;
            //mover.turn(?);
            break;
        case 3:
            cout << "Wir drehen uns nach unten." << endl;
            //mover.turn(?);
            break;
    }

    cout << "Und fahren " << streak << " Zelle(n) geradeaus." << endl;
    // Accelerating... ? 
    float dist = 0.8 * streak;
    mover.moveDist(dist);
}

void corridor(int plan[], int size){
    int streak = 1;

    for(int i = 1; i < size; i++){
        
        int predecessor = plan[i-1];
        int successor = plan[i];

        cout << "pre = " << predecessor << endl;
        cout << "suc = " << successor << endl;

        
        if(predecessor == successor){                                                                                    
            streak ++;
        } else{
            cout << "Jetzt wird " << streak << " Mal " << predecessor << " ausgeführt: " << endl;

            drive(predecessor, streak);
            
            // Streak wird erneuert
            streak = 1;
        }

        // Egal ob der letze Move Teil einer Streak ist oder nicht: Er muss nochmal seperat ausgeführt werden
        if(i == size - 1){
            cout << "Jetzt wird " << streak << " Mal " << successor << " ausgeführt: " << endl;
            drive(successor, streak);
        }

    }
}



int main(int argc, char **argv)
{   
    ros::init(argc, argv, "corridor");
	ros::NodeHandle n;
    
    int arr[] = {0,0,3,2,2,2,1,0,0,0,3,3,1};
    corridor(arr, 13);
    return 0;

}