#include "ros/ros.h"
#include "create_fundamentals/StoreSong.h"
#include "create_fundamentals/PlaySong.h"
#include "song.hpp"

Song::Song(){
    ROS_INFO("Waiting for services store_song and play_song ...");
    
    ros::service::waitForService("store_song");
    store_song_client = n.serviceClient<create_fundamentals::StoreSong>("store_song");
    
    ros::service::waitForService("play_song");
    play_song_client = n.serviceClient<create_fundamentals::PlaySong>("play_song");
    
    ROS_INFO("Ready!");
}

bool Song::uploadSong(int number, std::vector<unsigned int> song){
    store_song_srv.request.number = number;
    store_song_srv.request.song = song;
    store_song_client.call(store_song_srv);

    ros::Duration(1).sleep();

    return store_song_srv.response.success;
}

bool Song::playSong(int number, double duration){
    play_song_srv.request.number = number;
    play_song_client.call(play_song_srv);

    ros::Duration(duration).sleep();

    return play_song_srv.response.success;
}
