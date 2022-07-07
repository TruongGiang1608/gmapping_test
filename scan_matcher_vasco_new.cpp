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

double odom_new[3];
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
double anpha1 = 0.1; 
double anpha2 = 0.1; 
double anpha3 = 0.2; 
double anpha4 = 0.2;
// Call message from topic odom.
void odomCallback(const nav_msgs::Odometry& msg) {
	odom_old[0] = odom_new[0];
	odom_old[1] = odom_new[1];
	odom_old[2] = odom_new[2];
	
	odom_new[0] = msg.pose.pose.position.x;
	odom_new[1] = msg.pose.pose.position.y;
	odom_new[2] = tf::getYaw(msg.pose.pose.orientation);
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
	for (int i = 0; i <= 2; i++) {
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
		pose_guess[0][i] = pose_est[0][i] + del_trans[i]*cos(pose_est[2][i] + del_rot1[i]);
		pose_guess[1][i] = pose_est[1][i] + del_trans[i]*cos(pose_est[2][i] + del_rot1[i]);
		pose_guess[2][i] = pose_est[2][i] + del_rot1[i] + del_rot2[i];
	}
}
// void SampleMotionModel(double odom_pose_new[3], double odom_pose_old[3], double pose_est_old[3][M], double (&pose_guess)[3][M])
// { 
// 	double del_rot1, del_trans, del_rot2;
// 	double sig_rot1, sig_trans, sig_rot2;

// 	del_rot1 = atan2(odom_pose_new[1] - odom_pose_old[1],odom_pose_new[0] - odom_pose_old[0])- odom_pose_old[2];
// 	del_trans = sqrt(pow(odom_pose_new[0] - odom_pose_old[0], 2) + pow(odom_pose_new[1] - odom_pose_old[1], 2));
// 	del_rot2 = odom_pose_new[2] - odom_pose_old[2] - del_rot1;

// 	sig_rot1 = sqrt(anpha1*pow(del_rot1, 2) + anpha2*pow(del_trans, 2));
// 	sig_trans = sqrt(anpha3*pow(del_trans, 2) + anpha4*pow(del_rot1, 2) + anpha4*pow(del_rot2, 2));
// 	sig_rot2 = sqrt(anpha1*pow(del_rot2, 2) + anpha2*pow(del_trans, 2));

// 	random_device s;
// 	mt19937 rng(s());
// 	normal_distribution<double> nor_dist_rot1(0, sig_rot1);
// 	normal_distribution<double> nor_dist_trans(0, sig_trans);
// 	normal_distribution<double> nor_dist_rot2(0, sig_rot2);

//     double del_rot1_hat[M], del_trans_hat[M], del_rot2_hat[M];

// 	for (int i = 0; i < M; i++) {
// 		del_rot1_hat[i] = del_rot1 - nor_dist_rot1(rng);
// 		del_trans_hat[i] = del_trans - nor_dist_trans(rng);
// 		del_rot2_hat[i] = del_rot2 - nor_dist_rot2(rng);	

// 		pose_guess[0][i] = pose_est_old[0][i] + del_trans_hat[i]*cos(pose_est_old[2][i] + del_rot1_hat[i]);
// 		pose_guess[1][i] = pose_est_old[1][i] + del_trans_hat[i]*cos(pose_est_old[2][i] + del_rot1_hat[i]);
// 		pose_guess[2][i] = pose_est_old[2][i] + del_rot1_hat[i] + del_rot2_hat[i];
// 	}
// }
// Compute likelihood
void likelihood_gra(double x_t[3][M], double z_t[num_beam], double angle_beam[3], double m[map_width*map_height], double (&gra)[3]) {
    double gra_x = 0, gra_y = 0, gra_z = 0;
    double x_endpoint, y_endpoint;
    double x_occupied, y_occupied;
    double x_occupied_, y_occupied_;
    double dist, dist_min;
	double p_hit, p_rand;
    for(int i = 0; i < num_beam; i++) {
        x_endpoint = x_t[0][0] + 0.28*cos(x_t[2][0]) + z_t[i]*cos(i*angle_beam[2] + x_t[2][0]);
        y_endpoint = x_t[1][0] + 0.28*sin(x_t[2][0]) + z_t[i]*sin(i*angle_beam[2] + x_t[2][0]);
        dist_min = UINT16_MAX;
        for(int j = 0; j < map_width*map_height; j++) {
            if(m[j] > 0) {
                x_occupied_ = ((j + 1 - (j/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                y_occupied_ = ((j/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                dist = (sqrt(pow((x_endpoint - x_occupied), 2) + pow((y_endpoint - y_occupied), 2)));
                if(dist <= dist_min) {
                    dist_min = dist;
                    x_occupied = x_occupied_;
                    y_occupied = y_occupied_;
                }
            }
        }
	   p_hit = (z_hit/(sqrt(2*PI)*sigma_hit))*exp(-0.5*pow((dist_min/sigma_hit), 2));
	   p_rand = z_rand/Z_MAX;
	   gra_x += pow(sigma_hit, -2)*(x_endpoint - x_occupied)/(1 + p_rand/p_hit);
	   gra_y += pow(sigma_hit, -2)*(y_endpoint - y_occupied)/(1 + p_rand/p_hit);
	   gra_phi += pow(sigma_hit, -2)*((x_t[0][0] - x_occupied)*(-0.28*sin(x_t[2][0]) - z_t[i]*sin(x_t[2][0]+i*angle_beam[2])) + (x_t[1][0] - y_occupied)*(0.28*cos(x_t[2][0]) + z_t[i]*cos(x_t[2][0]+i*angle_beam[2])))/(1 + p_rand/p_hit);
    }
	gra[0] = gra_x;
	gra[1] = gra_y;
	gra[2] = gra_phi;
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "scan_matcher_vasco");
    ros::NodeHandle nh;
    // ros::Publisher pose_guess_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_guess", 1000);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_test", 1000);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber laser_scan_sub = nh.subscribe("lidar_1/scan", 1000, scanCallback);
    ROS_INFO("Running scan matching node !");
    
    int index_x, index_y;
    int index_num[num_beam];
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
    for(int i = 0; i < num_beam; i++) {
            index_x = ((pose_t[0][0] + 0.28*cos(pose_t[2][0]) + range[i]*cos(i*angle[2])*cos(pose_t[2][0]) - range[i]*sin(i*angle[2])*sin(pose_t[2][0])) - x_offset)/map_resolution + 1;
            index_y = ((pose_t[1][0] + 0.28*sin(pose_t[2][0]) + range[i]*cos(i*angle[2])*sin(pose_t[2][0]) + range[i]*sin(i*angle[2])*cos(pose_t[2][0])) - y_offset)/map_resolution + 1;
            index_num[i] = (index_y - 1)*map_width + index_x - 1;
    }
    for(int i = 0; i < num_beam; i++) {
            map_test.data[index_num[i]] = 100;
    }
	while(ros::ok()) {
		
	}
}
