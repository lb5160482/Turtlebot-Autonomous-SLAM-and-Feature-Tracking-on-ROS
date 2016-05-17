#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmvision_3d/Blobs3d.h>
#include <math.h>

ros::Publisher turtle_vel;
ros::Publisher spin_pub;


void trackCallback(const cmvision_3d::Blobs3d::ConstPtr& color){
    static double last_x = 0;
    static int last_flag = 0;
    static double fct = 0;
    if(color->blob_count == 0) {
        int flag = last_x>0?-1:1;
        geometry_msgs::Twist spin;
        if(flag * last_flag < 0) fct = 0;
        spin.angular.z = flag*(0.5 + fct);
        fct += 0.005;
        if(abs(spin.angular.z) > 1) spin.angular.z = spin.angular.z > 0 ? 1:-1;
        spin_pub.publish(spin);
        ROS_INFO("SPINING\n");
        last_flag = flag;
        return;
    }
    ROS_INFO("Tracking\n");
    double tar = 0.5;
    geometry_msgs::Twist cmd_vel;
    if(color->blobs[0].center.z < tar+0.1) return;
    double scalar_depth = 0.3,scalar_x = 1;
    double x = color->blobs[0].center.x;
    double depth = color->blobs[0].center.z;
    double input_x = x;
    double input_depth = depth - tar;
    last_x = x;


    cmd_vel.angular.z = -scalar_x*input_x;
    if(abs(cmd_vel.angular.z) < 0.3) cmd_vel.angular.z = cmd_vel.angular.z > 0 ? 0.3:-0.3;
    cmd_vel.linear.x = scalar_depth*input_depth;
    turtle_vel.publish(cmd_vel);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "color_tracker");

    ros::NodeHandle node;

    ros::Rate loop_rate(10);

    turtle_vel = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    ros::Subscriber sub = node.subscribe("blobs_3d", 3, &trackCallback);
    
    spin_pub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 3);
    
    ros::spin();

    return 0;

};

