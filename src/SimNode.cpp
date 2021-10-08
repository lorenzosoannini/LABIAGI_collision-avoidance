#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
#include <math.h>

#define T 100

laser_geometry::LaserProjection *projector_;
tf::TransformListener *listener_;
sensor_msgs::PointCloud *cloud_;
geometry_msgs::Twist *vel_;

void cmdCallback(const geometry_msgs::Twist msg){
    ROS_INFO("\nlinear speed: %f\nangular_speed: %f", msg.linear.x, msg.angular.z);

    *vel_ = msg; //save current input velocity
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    if(!(*listener_).waitForTransform(scan_in->header.frame_id, "/base_link", scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0))){
        return;
    }

    (*projector_).transformLaserScanToPointCloud("/base_link", *scan_in, *cloud_, *listener_); // populate a PointCloud structure from LaserScan
    
    //ROS_INFO("\npoints[0] = (%f, %f, %f)", cloud_->points[0].x, cloud_->points[0].y, cloud_->points[0].z);
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "SimNode");
    ros::NodeHandle n;

    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    sensor_msgs::PointCloud cloud;
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
    projector_ = &projector;
    listener_ = &listener;
    cloud_ = &cloud;
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

        /* TO BE TESTED

        float pi_dist = pow(cloud.points[0].x, 2.0) + pow(cloud.points[i].y, 2.0);

        float theta = atan2(-cloud.points[i].y, -cloud.points[i].x);
        float magnitude = 1/(pi_dist);

        vel_msg.linear.x += magnitude * cos(theta); //move back/forward
        vel_msg.angular.z += magnitude * sin(theta); //rotation in (x,y) plane

        */

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