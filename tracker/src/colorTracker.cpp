#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmvision_3d/Blobs3d.h>
#include <math.h>

int stopSPin = 0;
ros::Publisher turtle_vel;


void trackCallback(const cmvision_3d::Blobs3d::ConstPtr& color){
    if(color->blob_count == 0) return;
    stopSPin = 1;
    ROS_INFO("Tracking\n");
    double tar = 0.5;
    geometry_msgs::Twist cmd_vel;
    if(color->blobs[0].center.z < tar) return;
    double scalar_depth = 0.3,scalar_x = 1;
    double x = color->blobs[0].center.x;
    double depth = color->blobs[0].center.z;
    double input_x = x;
    double input_depth = depth - tar;


    cmd_vel.angular.z = -scalar_x*input_x;
    if(abs(cmd_vel.angular.z) < 0.3) cmd_vel.angular.z = cmd_vel.angular.z > 0 ? 0.3:-0.3;
    cmd_vel.linear.x = scalar_depth*input_depth;
    turtle_vel.publish(cmd_vel);
    ros::Duration(0.2).sleep();
    turtle_vel.publish(cmd_vel);
    ros::Duration(0.2).sleep();
    turtle_vel.publish(cmd_vel);



}

int main(int argc, char** argv){
    ros::init(argc, argv, "color_tracker");

    ros::NodeHandle node;

    ros::Rate loop_rate(10);

    turtle_vel = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    ros::Subscriber sub = node.subscribe("blobs_3d", 3, &trackCallback);
    
    ros::Publisher spin_pub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 3);
    
    geometry_msgs::Twist spin;
    
    spin.angular.z = 0.3;
    
    while(!stopSPin){
        spin_pub.publish(spin);
        ROS_INFO("SPINING\n");
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;

};



// header: 
//   seq: 412
//   stamp: 
//     secs: 1461016554
//     nsecs: 999655961
//   frame_id: camera_rgb_optical_frame
// blob_count: 1
// blobs: 
//   - 
//     name: green
//     red: 39
//     green: 188
//     blue: 233
//     area: 136
//     center: 
//       x: 0.0369000184766
//       y: -0.0657000328974
//       z: 0.948
//     top_left: 
//       x: 0.0226504774097
//       y: -0.0806356995784
//       z: 0.955
//     bottom_right: 
//       x: 0.0470508290851
//       y: -0.0559283440069
//       z: 0.935
