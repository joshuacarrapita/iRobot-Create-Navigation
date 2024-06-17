#ifndef SONG_H
#define SONG_H
#include "ros/ros.h"
#include "create_fundamentals/StoreSong.h"
#include "create_fundamentals/PlaySong.h"

class Song
{
	public:
		bool uploadSong(int number, std::vector<unsigned int> song);
        bool playSong(int number, double duration);
        void playLocalizedSound();
        void playLostSound();
        void playSuccessSound();

        ros::ServiceClient store_song_client;
        ros::ServiceClient play_song_client;

        create_fundamentals::StoreSong store_song_srv;
        create_fundamentals::PlaySong play_song_srv;

		ros::NodeHandle n;
        
		Song(); // Constructor
};

#endif