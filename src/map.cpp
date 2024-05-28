#include "ros/ros.h"
#include <cstdlib>
#include "map.hpp"

Map::Map(){
  while(ros::ok()){
  	const red_fundamentals::Grid::ConstPtr& msg = ros::topic::waitForMessage<red_fundamentals::Grid>("map", ros::Duration(10));

  	if (msg != nullptr){
   		this->map = *msg;
   		this->num_rows = msg->rows.size();
   		this->num_columns = msg->rows[0].cells.size();
		ROS_INFO("Number of rows: %lu", msg->rows.size());
		for(size_t i=0; i<num_rows; i++){
			const red_fundamentals::Row& row = msg->rows[i];
			ROS_INFO("Row %lu: Number of cells: %lu", i, row.cells.size());
			for(size_t j=0; j<num_columns; j++){
				const red_fundamentals::Cell& cell = row.cells[j];
				ROS_INFO("Row %lu, Cell %lu: Number of walls: %lu", i, j, cell.walls.size());
				for(size_t k=0; k<cell.walls.size(); k++){
					ROS_INFO("Row %lu, Cell %lu, Wall %lu: %d", i, j, k, cell.walls[k]);
				}
			}
		}
    	break;
  	}
  	else{
    		ROS_INFO("Waiting for the map information");
  	}
  }
}


