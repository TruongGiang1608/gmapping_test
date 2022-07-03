#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
using namespace std;

double odom_s[3], odom_t[3];
void odomCallback(const nav_msgs::Odometry& msg) {
	odom_t[0] = odom_s[0];
	odom_t[1] = odom_s[1];
	odom_t[2] = odom_s[2];
	
	odom_s[0] = msg.pose.pose.position.x;
	odom_s[1] = msg.pose.pose.position.y;
	odom_s[2] = tf::getYaw(msg.pose.pose.orientation);
}
const int num_beam = 720;
double angle[3], range[num_beam];
void scanCallback(const sensor_msgs::LaserScan& scan) {
    for(int i = 0; i < num_beam; i++) {
        range[i] = scan.ranges[i];
    }
    angle[0] = scan.angle_min;
    angle[1] = scan.angle_max;
    angle[2] = scan.angle_increment;
}
const int M = 1;
void SampleInitial(double (&initial_pose)[3][M]) {
	random_device s0;
	mt19937 rng0(s0());
	uniform_real_distribution<double> uni(0.0,1.0);
	for (int i = 0; i <= 2; i++) {
		for(int j = 0; j < M; j++) {
			initial_pose[i][j] = uni(rng0);
		}
	}
}
void poseGuess(double odom_pose_suc[3], double odom_pose_pre[3], double pose_est_pre[3][M], double (&pose_t)[3][M]) {
    for (int i = 0; i < M; i++) {
        pose_t[0][i] = pose_est_pre[0][i] + odom_pose_suc[0] - odom_pose_pre[0];
        pose_t[1][i] = pose_est_pre[1][i] + odom_pose_suc[1] - odom_pose_pre[1];
        pose_t[2][i] = pose_est_pre[2][i] + odom_pose_suc[2] - odom_pose_pre[2];
    }
}
const double map_width = 500, map_height = 500;
const double map_resolution = 0.05;
double x_offset = -map_width*map_resolution/2;
double y_offset = -map_height*map_resolution/2;

int main (int argc, char **argv) {
    ros::init(argc, argv, "scan_matcher_vasco");
    ros::NodeHandle nh;
    // ros::Publisher pose_guess_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_guess", 1000);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_test", 1000);

    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber laser_scan_sub = nh.subscribe("lidar_1/scan", 1000, scanCallback);

    ROS_INFO("Running scan matching node !");

    double pose_t[3][M] = {0, 0, 0};
	// SampleInitial(pose_t);
    nav_msgs::OccupancyGrid map_test;
    map_test.data.resize(map_width*map_height);
    map_test.header.frame_id = "map";
    map_test.info.map_load_time = ros::Time::now();
    map_test.info.resolution = map_resolution;
    map_test.info.width = map_width;
    map_test.info.height = map_height;
    map_test.info.origin.position.x = x_offset;
    map_test.info.origin.position.y = y_offset;
    map_test.info.origin.position.z = 0.0;
    map_test.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
    for(int i = 0; i < map_width*map_height; i++) {
        map_test.data.push_back(0);
    }
    ros::Rate rate(50);
    while(ros::ok()) {
        ros::spinOnce();
        poseGuess(odom_s, odom_t, pose_t, pose_t);
        geometry_msgs::PoseStamped pose_laser;
        pose_laser.header.stamp = ros::Time::now();
        pose_laser.header.frame_id = "map";
        pose_laser.pose.position.x = pose_t[0][0] + 0.28*cos(pose_t[2][0]);
        pose_laser.pose.position.y = pose_t[1][0] + 0.28*sin(pose_t[2][0]);
        pose_laser.pose.position.z = 0.095;
        pose_laser.pose.orientation = tf::createQuaternionMsgFromYaw(pose_t[2][0]);

        int index_x, index_y;
        int index_num[num_beam];
        for(int i = 0; i < num_beam; i++) {
            index_x = ((pose_laser.pose.position.x + range[i]*cos(i*angle[2])*cos(pose_t[2][0]) - range[i]*sin(i*angle[2])*sin(pose_t[2][0])) - x_offset)/map_resolution + 1;
            index_y = ((pose_laser.pose.position.y + range[i]*cos(i*angle[2])*sin(pose_t[2][0]) + range[i]*sin(i*angle[2])*cos(pose_t[2][0])) - y_offset)/map_resolution + 1;
            index_num[i] = (index_y - 1)*map_width + index_x - 1;
        }
        for(int i = 0; i < num_beam; i++) {
            map_test.data[index_num[i]] = 100;
        }
        map_pub.publish(map_test);
        rate.sleep();
        
    }

}
