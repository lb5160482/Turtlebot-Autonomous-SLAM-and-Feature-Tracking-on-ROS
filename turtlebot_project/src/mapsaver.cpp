#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <sensor_msgs/Joy.h>

void mapSaveCallBack(const sensor_msgs::Joy::ConstPtr& joy){	
    if(joy->buttons[2] == 1){
	    char *name[4];
        ROS_INFO("map saved\n");
	    name[0] = "/bin/bash";
	    name[1] = "-c";
	    name[2] = "rosrun map_server map_saver -f /tmp/lab_map";
	    name[3] = NULL;
	    execvp(name[0], name);
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "map_saver");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("joy", 10, &mapSaveCallBack);
    
    ros::spin();

    return 0;

}