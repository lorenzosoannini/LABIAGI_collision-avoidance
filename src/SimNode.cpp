#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"

#define T 100

geometry_msgs::Twist *vel_;

void cmdCallback(const geometry_msgs::Twist msg){
    ROS_INFO("\nlinear speed: %f\nangular_speed: %f", msg.linear.x, msg.angular.z);

    *vel_ = msg; //save current input velocity
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    //TO DO - need to create a PointCloud structure from LaserScan
    return;
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "SimNode");
    ros::NodeHandle n;

    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    vel_ = &vel;

    ros::Subscriber scan_sub = n.subscribe("/base_scan", 1000, scanCallback);
    ros::Subscriber cmd_sub = n.subscribe("/cmd_vel", 1000, cmdCallback);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Rate r(T);

    while(ros::ok()){

        geometry_msgs::Twist vel_msg = vel; //initialize msg to be published

        //do nothing if input velocity is 0
        if(vel.linear.x == 0 && vel.linear.y == 0 && vel.linear.z == 0 &&
            vel.angular.x == 0 && vel.angular.y == 0 && vel.angular.z == 0){
            ros::spinOnce();
            r.sleep();
            continue;
            
        }

        /* DO STUFF WITH LASER SCAN TO AVOID OBSTACLES */

        cmd_pub.publish(vel_msg);
        ROS_INFO("\nnew linear speed: %f\nnew angular_speed: %f", vel_msg.linear.x, vel_msg.angular.z);

        ros::spinOnce();

        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        r.sleep();
    }

    return 0;
}