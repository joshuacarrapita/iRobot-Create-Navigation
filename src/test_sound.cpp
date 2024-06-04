#include "ros/ros.h"
#include <create_fundamentals/StoreSong.h>
#include <create_fundamentals/PlaySong.h>

// define note constants
const int c4 = 60;  // Middle C
const int Q = 160 / 4;  // Quarter note length for a measure of 160

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_sound");
    ros::NodeHandle node;

    ROS_INFO("Waiting for services store_song and play_song ...");
    ros::service::waitForService("store_song");
    ros::ServiceClient store_song_client = node.serviceClient<create_fundamentals::StoreSong>("store_song");
    ros::service::waitForService("play_song");
    ros::ServiceClient play_song_client = node.serviceClient<create_fundamentals::PlaySong>("play_song");
    ROS_INFO("Ready!");

    create_fundamentals::StoreSong store_song_srv;
    create_fundamentals::PlaySong play_song_srv;

    // Upload a short tone 
    store_song_srv.request.number = 0;
    store_song_srv.request.song = {c4, Q};
    store_song_client.call(store_song_srv);

    ros::Duration(1).sleep();

    ROS_INFO("Playing short tone...");

    // Play the short tone
    play_song_srv.request.number = 0;
    play_song_client.call(play_song_srv);

    ros::Duration(Q / 1000.0).sleep();  // Wait for the tone to finish playing

    return 0;
}
