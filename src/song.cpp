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

void Song::playLocalizedSound(){
    ROS_INFO("Play localized sound");
    Song s;

    // Define note constants
    unsigned int a4 = 69;
    unsigned int c5 = 72; 
    unsigned int e5 = 76;
    unsigned int g5 = 79; 
    unsigned int a5 = 81;  
    unsigned int c6 = 84;  

    // Define note durations
    unsigned int Q = 160 / 4;
    unsigned int E = Q / 2;
    unsigned int S = Q / 4;   

    // Upload the song 
    s.uploadSong(0, {a4, S, c5, S, e5, E, g5, S, a5, S, c6, Q});

    // Play the song 
    double duration = (S + S + E + S + S + Q) / 1000.0;
    s.playSong(0, duration);
}

void Song::playLostSound(){
    ROS_INFO("Play lost sound");
    Song s;

    // Define note constants
    unsigned int e4 = 64;
    unsigned int d4 = 62; 
    unsigned int c4 = 60; 
    unsigned int a3 = 57; 
    unsigned int g3 = 55;  

    // Define note durations
    unsigned int Q = 160 / 4;
    unsigned int H = Q * 2; 

    // Upload the song
    s.uploadSong(0, {e4, Q, d4, Q, c4, Q, a3, Q, g3, H});

    // Play the song
    double duration = (Q * 4 + H) / 1000.0;
    s.playSong(0, duration);
}
