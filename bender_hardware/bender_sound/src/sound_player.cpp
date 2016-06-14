
// C++
#include <fstream>
#include <string.h>
#include <unistd.h>

// ros
#include <ros/package.h>
#include <sound_play/sound_play.h>
#include <bender_srvs/play_sound.h>

 
// TODO: agregar funcionalidad para stop y rewind.

bool file_exists(const std::string &filename)
{
  std::ifstream ifile(filename.c_str());
  return ifile;
}

bool playSound(bender_srvs::play_sound::Request  &req, bender_srvs::play_sound::Response &res){ 

    sound_play::SoundClient sc; 
    std::string sound_path = req.sound_file;

    // stop current sounds!
    if (req.play == false) {
        ROS_INFO("Stopping sounds...");
        sc.stop(1);
        res.success = true;
        return true;
    }

    // try to play the sound
    res.success = false;
    if ( !file_exists(sound_path) ) {
        ROS_WARN_STREAM("Attempted to play an nonexistent sound file: " << sound_path);
        return false;
    }
    ROS_INFO_STREAM("Playing sound: " << sound_path);
	sc.playWave(sound_path);
	res.success = true;    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "player");
    ros::NodeHandle nh("~");
    sound_play::SoundClient sc; 
    ros::ServiceServer service = nh.advertiseService("play", playSound);

    ros::spin();
    return 0;
}



