#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <sensor_msgs/Joy.h>

ros::Publisher turtle_vel;

void startTrCallBack(const sensor_msgs::Joy::ConstPtr& joy){	
    if(joy->buttons[10] == 1){
        char *name[4];
        name[0] = "/bin/bash";
        name[1] = "-c";
        name[2] = "roslaunch rbx1_speech turtlebot_voice_nav.launch";
        name[3] = NULL;
        execvp(name[0], name);
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "start_voice");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("joy", 10, &startTrCallBack);
    
    ros::spin();

    return 0;

}