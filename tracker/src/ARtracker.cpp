#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <math.h>

int stopSPin = 0;
ros::Publisher turtle_vel;
ros::Publisher spin_pub;


void trackCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& ar){
	static double last_y;
    if(ar->markers.size() == 0) {
    	int flag = last_y>0?1:-1;
    	geometry_msgs::Twist spin;
	    spin.angular.z = flag*0.5;
	    spin_pub.publish(spin);
		ROS_INFO("SPINING\n");
    	return;

    }
    ROS_INFO("TRACKING\n");
    stopSPin = 1;
    double tar = 0.5;
    geometry_msgs::Twist cmd_vel;
    if(ar->markers[0].pose.pose.position.x < tar) return;
    double scalar_x = 0.5,scalar_y = 1;
    double x = ar->markers[0].pose.pose.position.x;
    double y = ar->markers[0].pose.pose.position.y;

    double input_x = x - tar;
    double input_y = y;
    last_y = y;


    cmd_vel.angular.z = scalar_y*input_y;
    cmd_vel.linear.x = scalar_x*input_x;
    if(abs(cmd_vel.angular.z)<0.5 && abs(y) > 0.01) cmd_vel.angular.z = cmd_vel.angular.z>0?0.5:-0.5;
    turtle_vel.publish(cmd_vel);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "AR_tracker");

    ros::NodeHandle node;

    ros::Rate loop_rate(10);

    turtle_vel = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    ros::Subscriber sub = node.subscribe("ar_pose_marker", 10, &trackCallback);
    
	spin_pub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    
    ros::spin();

    return 0;

};
