#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <sensor_msgs/Joy.h>

void killTrCallBack(const sensor_msgs::Joy::ConstPtr& joy){	
    if(joy->buttons[0] == 1){
        ROS_INFO("stop\n");
	    char *name[4];
	    name[0] = "/bin/bash";
	    name[1] = "-c";
	    name[2] = "rosnode kill /AR_tracker";
	    name[3] = NULL;
	    execvp(name[0], name);
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "kill_tracking");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("joy", 10, &killTrCallBack);
    
    ros::spin();

    return 0;

}