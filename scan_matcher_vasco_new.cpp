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
#include <math.h>
using namespace std;

double odom_new[3], odom_t[3];
double odom_old[3];
const int M = 1; // Number of partiles
const int num_beam = 720; // Number of beams
const int Z_MAX = 15; // Range maximum of beam [m]
double angle[3];
double range[num_beam];
const int map_width = 1200, map_height = 1200; // Size of map [cells]
const double map_resolution = 0.01; // Resolution of map [m]
double x_offset = -map_width*map_resolution/2;
double y_offset = -map_height*map_resolution/2;
const double PI = acos(-1);
const double z_hit = 0.95, z_rand = 0.05, sigma_hit = 0.1;
const double epsilon_trans = 0.001;
const double epsilon_rot = 0.001;
const double eta = 0.1;
double x_endpoint, y_endpoint;
double x_occupied, y_occupied;
double x_occupied_, y_occupied_;
double dist, dist_min;
double p_hit, p_rand;
double m[map_width*map_height];
double gra_x, gra_y, gra_phi;

// Call message from topic odom.
void odomCallback(const nav_msgs::Odometry& msg) {
    odom_t[0] = msg.pose.pose.position.x;
	odom_t[1] = msg.pose.pose.position.y;
	odom_t[2] = tf::getYaw(msg.pose.pose.orientation);
}
// Call message from topic laser scan.
void scanCallback(const sensor_msgs::LaserScan& scan) {
    for(int i = 0; i < num_beam; i++) {
        range[i] = scan.ranges[i];
    }
    angle[0] = scan.angle_min;
    angle[1] = scan.angle_max;
    angle[2] = scan.angle_increment;
}
//Sample M particles with uniform distribution.
void SampleInitial(double (&initial_pose)[3][M]) {
	random_device s0;
	mt19937 rng0(s0());
	uniform_real_distribution<double> uni(0.0,1.0);
	for (int i = 0; i < 3; i++) {
		for(int j = 0; j < M; j++) {
			initial_pose[i][j] = uni(rng0);
		}
	}
}
// Compute initial guess pose.
void PoseGuess(double odom_pose_new[3], double odom_pose_old[3], double pose_est[3][M], double (&pose_guess)[3][M])
{ 
	double del_rot1, del_trans, del_rot2;
	del_rot1 = atan2(odom_pose_new[1] - odom_pose_old[1],odom_pose_new[0] - odom_pose_old[0])- odom_pose_old[2];
	del_trans = sqrt(pow(odom_pose_new[0] - odom_pose_old[0], 2) + pow(odom_pose_new[1] - odom_pose_old[1], 2));
	del_rot2 = odom_pose_new[2] - odom_pose_old[2] - del_rot1;
	for (int i = 0; i < M; i++) {
		pose_guess[0][i] = pose_est[0][i] + del_trans*cos(pose_est[2][i] + del_rot1);
		pose_guess[1][i] = pose_est[1][i] + del_trans*cos(pose_est[2][i] + del_rot1);
		pose_guess[2][i] = pose_est[2][i] + del_rot1 + del_rot2;
	}
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "scan_matcher_vasco");
    ros::NodeHandle nh;
    // ros::Publisher pose_guess_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_guess", 1000);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_test", 1000);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber laser_scan_sub = nh.subscribe("lidar_1/scan", 1000, scanCallback);
    ROS_INFO("Running scan matching node !");
    
    int index_x, index_y, index_xy;
    int index_num[num_beam];
    double pose_t[3][M] = {0, 0, 0};
    double range_old[num_beam];
    
	// // SampleInitial(pose_t);
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
    for(int i = 0; i < num_beam; i++) {
            index_x = ((pose_t[0][0] + 0.28*cos(pose_t[2][0]) + range[i]*cos(i*angle[2] + pose_t[2][0])) - x_offset)/map_resolution + 1;
            index_y = ((pose_t[1][0] + 0.28*sin(pose_t[2][0]) + range[i]*sin(i*angle[2] + pose_t[2][0])) - y_offset)/map_resolution + 1;
            index_num[i] = (index_y - 1)*map_width + index_x - 1;
    }
    for(int i = 0; i < num_beam; i++) {
            map_test.data[index_num[i]] = 100;
    }
    ros::Rate rate(50);
	while(ros::ok()) {
        ros::spinOnce();
        for(int i = 0; i < 3; i++) {
            odom_new[i] = odom_t[i];
        } 
        // cout << odom_new[2] << " " << odom_old[2] << endl;
        PoseGuess(odom_new, odom_old, pose_t, pose_t);
        // for(int i = 0; i < map_width*map_height; i++) {
        //     m[i] = map_test.data[i];
        // }
        // for(int i = 0; i < num_beam; i++) {
        //     range_old[i] = range[i];
        // }
        do{
            gra_x = 0; gra_y = 0; gra_phi = 0;
            for(int i = 0; i < num_beam; i++) {
            x_endpoint = pose_t[0][0] + 0.28*cos(pose_t[2][0]) + range[i]*cos(i*angle[2] + pose_t[2][0]);
            y_endpoint = pose_t[1][0] + 0.28*sin(pose_t[2][0]) + range[i]*sin(i*angle[2] + pose_t[2][0]);
            index_x = (x_endpoint - x_offset)/map_resolution + 1;
            index_y = (y_endpoint - y_offset)/map_resolution + 1;
            dist_min = UINT16_MAX;
            if((index_x - 2) >= 0 && (index_x + 2) <= map_width && (index_y - 2) >= 0 && (index_y + 2) <= map_height) {
                for(int j = index_x - 2; j <= index_x + 2; j++) {
                    for(int k = index_y - 2; k <= index_y + 2; k++) {
                        index_xy = (k - 1)*map_width + (j - 1);
                        if(map_test.data[index_xy] > 0) {
                            x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                            y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                            dist = (sqrt(pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2)));
                            if(dist <= dist_min) {
                                x_occupied = x_occupied_;
                                y_occupied = y_occupied_;
                                dist_min = dist;
                            }
                        }
                    }
                }
            } else {
                for(int j = 0; j < map_width*map_height; j++) {
                    if(map_test.data[j] > 0) {
                    x_occupied_ = ((j + 1 - (j/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                    y_occupied_ = ((j/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                    dist = (sqrt(pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2)));
                    if(dist <= dist_min) {
                    dist_min = dist;
                    x_occupied = x_occupied_;
                    y_occupied = y_occupied_;
                    }
                }
            }     
            }
	        p_hit = (z_hit/(sqrt(2*PI)*sigma_hit))*exp(-0.5*pow((dist_min/sigma_hit), 2));
	        p_rand = z_rand/Z_MAX;
	        gra_x += pow(sigma_hit, -2)*(x_endpoint - x_occupied)/(1 + p_rand/p_hit);
	        gra_y += pow(sigma_hit, -2)*(y_endpoint - y_occupied)/(1 + p_rand/p_hit);
	        gra_phi += pow(sigma_hit, -2)*((pose_t[0][0] - x_occupied)*(-0.28*sin(pose_t[2][0]) - range[i]*sin(pose_t[2][0]+i*angle[2])) + (pose_t[1][0] - y_occupied)*(0.28*cos(pose_t[2][0]) + range[i]*cos(pose_t[2][0]+i*angle[2])))/(1 + p_rand/p_hit);
            cout << "gra:" << gra_x << " " << gra_y << " " << gra_phi << endl;
            }   
            pose_t[0][0] = pose_t[0][0] - eta*gra_x;
            pose_t[1][0] = pose_t[1][0] - eta*gra_y;
            pose_t[2][0] = pose_t[2][0] - eta*gra_phi;
         
        } while(fabs(gra_x) > epsilon_trans && fabs(gra_y) > epsilon_trans && fabs(gra_phi) > epsilon_rot);
        // cout << "gra:" << gra_x << " " << gra_y << " " << gra_phi << endl;
        // cout << pose_t[2][0] << " " << pose_t[0][0] << " " << pose_t[1][0] << endl;
        for(int i = 0; i < num_beam; i++) {
            index_x = ((pose_t[0][0] + 0.28*cos(pose_t[2][0]) + range[i]*cos(i*angle[2] + pose_t[2][0])) - x_offset)/map_resolution + 1;
            index_y = ((pose_t[1][0] + 0.28*sin(pose_t[2][0]) + range[i]*sin(i*angle[2] + pose_t[2][0])) - y_offset)/map_resolution + 1;
            index_num[i] = (index_y - 1)*map_width + index_x - 1;
    }
        for(int i = 0; i < num_beam; i++) {
            map_test.data[index_num[i]] = 100;
        }
        map_pub.publish(map_test);
        for(int i = 0; i < 3; i++) {
            odom_old[i] = odom_new[i];
        }   
        rate.sleep();
	}
    return 0;
}
