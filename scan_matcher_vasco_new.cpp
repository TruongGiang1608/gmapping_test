#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
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
#include <algorithm>
using namespace std;

const int M = 1; // Number of partiles
const int num_beam = 540; // Number of beams
const int Z_MAX = 25; // Range maximum of beam [m]
const int N = 50;
const int kernel_size = 15; // Kernel size [cells]
const int r = (kernel_size-1)/2;
const int map_width = 1000, map_height = 1000; // Size of map [cells]
const double map_resolution = 0.01; // Resolution of map [m]
const double x_offset = -map_width*map_resolution/2;
const double y_offset = -map_height*map_resolution/2;
const double PI = acos(-1);
const double z_hit = 0.95, sigma_hit = 0.05;
const double a = z_hit/sqrt(2*PI*pow(sigma_hit, 2));
const double c = (1 - z_hit)/Z_MAX;
const double epsilon_trans = 0.001;
const double epsilon_rot = 0.001;
const double eta = 0.001;
double angle[3];
double range[num_beam];
double odom_t[3];

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

int main (int argc, char **argv) {
    ros::init(argc, argv, "scan_matcher_vasco");
    ros::NodeHandle nh;
    ros::Publisher pose_guess_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_guess", 1000);
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map_test", 1000);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);
    ros::Subscriber laser_scan_sub = nh.subscribe("scan1", 1000, scanCallback);
    ROS_INFO("Running scan matching node !");
    
    double odom_new[3], odom_old[3];
    double x_sens, y_sens;
    double x_endpoint, y_endpoint;
    double x_occupied, y_occupied;
    double x_occupied_, y_occupied_;
    double dist2, dist2_min;
    double dist2_dot_x, dist2_dot_y, dist2_dot_phi;
    double db_dx, db_dy, db_dphi;
    double dlogq_dx, dlogq_dy, dlogq_dphi;
    double gra_x, gra_y, gra_phi;
    double b;
    double x_endpoint_dot_phi, y_endpoint_dot_phi;
    double sum_gra_x, sum_gra_y, sum_gra_phi;
    int nu[num_beam];
    int index_x, index_y, index_xy;
    int index_num[num_beam];
    double pose_t[3][M] = {0.0, 0.0, 0.0};
    double range_old[num_beam];
    double del_x, del_y, del_phi;
    
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
    x_sens = 0.289;
    y_sens = -0.000;
    for(int i = 0; i < map_width*map_height; i++) {
        map_test.data.push_back(0);
    }
    for(int i = 0; i < num_beam; i++) {
            x_endpoint = pose_t[0][0] + x_sens*cos(pose_t[2][0]) - y_sens*sin(pose_t[2][0]) + range[i]*cos(PI/2 - i*angle[2] + pose_t[2][0]);
            y_endpoint = pose_t[1][0] + x_sens*sin(pose_t[2][0]) + y_sens*cos(pose_t[2][0]) + range[i]*sin(PI/2 -i*angle[2] + pose_t[2][0]);
            index_x = (x_endpoint - x_offset)/map_resolution + 1;
            index_y = (y_endpoint - y_offset)/map_resolution + 1;
            index_num[i] = (index_y - 1)*map_width + index_x - 1;
            map_test.data[index_num[i]] = 100;
            nu[i] = i;
    }
    for(int i = 0; i < 3; i++) {
            odom_old[i] = odom_t[i];
            odom_new[i] = odom_old[i];
        } 
    ros::Rate rate(1);
	while(ros::ok()) {
        ros::spinOnce();
        for(int i = 0; i < 3; i++) {
            odom_new[i] = odom_t[i];
        } 
        del_x = (odom_new[0] - odom_old[0])*cos(odom_old[2]) + (odom_new[1] - odom_old[1])*sin(odom_old[2]);
        del_y = -(odom_new[0] - odom_old[0])*sin(odom_old[2]) + (odom_new[1] - odom_old[1])*cos(odom_old[2]);
        del_phi = odom_new[2] - odom_old[2];
        for (int i = 0; i < M; i++) {
		    pose_t[0][i] = pose_t[0][i] + del_x*cos(pose_t[2][i]) - del_y*sin(pose_t[2][i]);
		    pose_t[1][i] = pose_t[1][i] + del_x*sin(pose_t[2][i]) + del_y*cos(pose_t[2][i]);
		    pose_t[2][i] = pose_t[2][i] + del_phi;
	    }
        cout << pose_t[0][0] << " " << pose_t[1][0] << endl;
        for(int i = 0; i < num_beam; i++) {
            range_old[i] = range[i];
        }
        sum_gra_x = 0;
        sum_gra_y = 0;
        sum_gra_phi = 0;
        random_shuffle(&nu[0],&nu[num_beam]);
        do{
            // random_shuffle(&nu[0],&nu[num_beam]);
            gra_x = 0;
            gra_y = 0;
            gra_phi = 0;
            for(int i = 0; i < N; i++) {
                x_endpoint = pose_t[0][0] + x_sens*cos(pose_t[2][0]) - y_sens*sin(pose_t[2][0]) + range[i]*cos(PI/2 - i*angle[2] + pose_t[2][0]);
                y_endpoint = pose_t[1][0] + x_sens*sin(pose_t[2][0]) + y_sens*cos(pose_t[2][0]) + range[i]*sin(PI/2 -i*angle[2] + pose_t[2][0]);
                x_endpoint_dot_phi = -x_sens*sin(pose_t[2][0]) - y_sens*cos(pose_t[2][0]) - range[i]*sin(PI/2 - i*angle[2] + pose_t[2][0]);
                y_endpoint_dot_phi = x_sens*cos(pose_t[2][0]) - y_sens*sin(pose_t[2][0]) + range[i]*cos(PI/2 - i*angle[2] + pose_t[2][0]);
                index_x = (x_endpoint - x_offset)/map_resolution + 1;
                index_y = (y_endpoint - y_offset)/map_resolution + 1;
                dist2_min = UINT16_MAX;
                if((index_x - r) > 0 && (index_x + r) <= map_width && (index_y - r) > 0 && (index_y + r) <= map_height) { // no.1
                    for(int j = index_x - r; j <= index_x + r; j++) {
                        for(int k = index_y - r; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) <= 0 && (index_x + r) <= map_width && (index_y - r) > 0 && (index_y + r) <= map_height){ // no.2
                    for(int j = 1; j <= index_x + r; j++) {
                        for(int k = index_y - r; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) > map_width && (index_y - r) > 0 && (index_y + r) <= map_height){ // no.3
                    for(int j = index_x - r; j <= map_width; j++) {
                        for(int k = index_y - r; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) <= map_width && (index_y - r) <= 0 && (index_y + r) <= map_height){ // no.4
                    for(int j = index_x - r; j <= index_x + r; j++) {
                        for(int k = 1; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) <= map_width && (index_y - r) > 0 && (index_y + r) > map_height){ // no.5
                    for(int j = index_x - r; j <= index_x + r; j++) {
                        for(int k = index_y - r; k <= map_height; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) <= map_width && (index_y - r) > 0 && (index_y + r) <= map_height){ // no.6
                    for(int j = index_x - r; j <= map_width; j++) {
                        for(int k = index_y - r; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) <= 0 && (index_x + r) > map_width && (index_y - r) <= 0 && (index_y + r) > map_height){ // no.7
                    for(int j = 1; j <= map_width; j++) {
                        for(int k = 1; k <= map_height; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) > map_width && (index_y - r) <= 0 && (index_y + r) > map_height){ // no.8
                    for(int j = index_x - r; j <= map_width; j++) {
                        for(int k = 1; k <= map_height; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) <= map_width && (index_y - r) <= 0 && (index_y + r) > map_height){ // no.9
                    for(int j = index_x - r; j <= index_x + r; j++) {
                        for(int k = 1; k <= map_height; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) <= map_width && (index_y - r) > 0 && (index_y + r) > map_height){ // no.10
                    for(int j = index_x - r; j <= index_x + r; j++) {
                        for(int k = index_y - r; k <= map_height; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) <= 0 && (index_x + r) > map_width && (index_y - r) > 0 && (index_y + r) <= map_height){ // no.11
                    for(int j = 1; j <= map_width; j++) {
                        for(int k = index_y - r; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) <= 0 && (index_x + r) > map_width && (index_y - r) <= 0 && (index_y + r) <= map_height){ // no.12
                    for(int j = 1; j <= map_width; j++) {
                        for(int k = 1; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) <= 0 && (index_x + r) <= map_width && (index_y - r) <= 0 && (index_y + r) <= map_height){ // no.13
                    for(int j = 1; j <= index_x + r; j++) {
                        for(int k = 1; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) > map_width && (index_y - r) > 0 && (index_y + r) > map_height){ // no.14
                    for(int j = index_x - r; j <= map_width; j++) {
                        for(int k = index_y - r; k <= map_height; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else if((index_x - r) > 0 && (index_x + r) > map_width && (index_y - r) <= 0 && (index_y + r) <= map_height){ // no.15
                    for(int j = index_x - r; j <= map_width; j++) {
                        for(int k = 1; k <= index_y + r; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                } else { // no.16
                    for(int j = 1; j <= index_x + r; j++) {
                        for(int k = index_y - r; k <= map_height; k++) {
                            index_xy = (k - 1)*map_width + (j - 1);
                            if(map_test.data[index_xy] > 0) {
                                x_occupied_ = ((index_xy + 1 - (index_xy/map_width)*map_width)*map_resolution - map_resolution/2 + x_offset);
                                y_occupied_ = ((index_xy/map_width + 1)*map_resolution - map_resolution/2 + y_offset);
                                dist2 = pow((x_endpoint - x_occupied_), 2) + pow((y_endpoint - y_occupied_), 2);
                                if(dist2 <= dist2_min) {
                                    x_occupied = x_occupied_;
                                    y_occupied = y_occupied_;
                                    dist2_min = dist2;
                                }
                            }
                        }
                    }
                }
                if(dist2_min == UINT16_MAX) {
                    gra_x += 0;
                    gra_y += 0;
                    gra_phi += 0;
                } else {
                    dist2_dot_x = 2*(x_endpoint - x_occupied);
                    dist2_dot_y = 2*(y_endpoint - y_occupied);
                    dist2_dot_phi = 2*((x_endpoint - x_occupied)*x_endpoint_dot_phi + (y_endpoint - y_occupied)*y_endpoint_dot_phi);
                    b = -0.5*dist2_min/pow(sigma_hit, 2);

                    db_dx = -0.5*dist2_dot_x/pow(sigma_hit, 2);
                    db_dy = -0.5*dist2_dot_y/pow(sigma_hit, 2);
                    db_dphi = -0.5*dist2_dot_phi/pow(sigma_hit, 2);

                    dlogq_dx = a*exp(b)*db_dx/(c+a*exp(b));
                    dlogq_dy = a*exp(b)*db_dy/(c+a*exp(b));
                    dlogq_dphi = a*exp(b)*db_dphi/(c+a*exp(b));

	                gra_x += -dlogq_dx;
	                gra_y += -dlogq_dy;
	                gra_phi += -dlogq_dphi;
                }
            }
            sum_gra_x += pow(gra_x, 2);
            sum_gra_y += pow(gra_y, 2);
            sum_gra_phi += pow(gra_x, 2);

            pose_t[0][0] = pose_t[0][0] - eta*gra_x/sqrt(sum_gra_x + 1e-8);
            pose_t[1][0] = pose_t[1][0] - eta*gra_y/sqrt(sum_gra_y + 1e-8);
            pose_t[2][0] = pose_t[2][0] - eta*gra_phi/sqrt(sum_gra_phi + 1e-8);

            // pose_t[0][0] = pose_t[0][0] - eta*gra_x;
            // pose_t[1][0] = pose_t[1][0] - eta*gra_y;
            // pose_t[2][0] = pose_t[2][0] - eta*gra_phi;
        } while((fabs(gra_x) > epsilon_trans) || (fabs(gra_y) > epsilon_trans) || (fabs(gra_phi) > epsilon_rot));
        // cout << "Gradient_descent:" << gra_x << " " << gra_y << " " << gra_phi << endl;
        for(int i = 0; i < num_beam; i++) {
            x_endpoint = pose_t[0][0] + x_sens*cos(pose_t[2][0]) - y_sens*sin(pose_t[2][0]) + range_old[i]*cos(PI/2 - i*angle[2] + pose_t[2][0]);
            y_endpoint = pose_t[1][0] + x_sens*sin(pose_t[2][0]) + y_sens*cos(pose_t[2][0]) + range_old[i]*sin(PI/2 -i*angle[2] + pose_t[2][0]);
            index_x = (x_endpoint - x_offset)/map_resolution + 1;
            index_y = (y_endpoint - y_offset)/map_resolution + 1;
            index_num[i] = (index_y - 1)*map_width + index_x - 1;
            map_test.data[index_num[i]] = 100;
        }
        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "map";
        p.pose.position.x = pose_t[0][0];
        p.pose.position.y = pose_t[1][0];
        p.pose.position.z = 0;
        p.pose.orientation = tf::createQuaternionMsgFromYaw(pose_t[2][0]);

        pose_guess_pub.publish(p);
        map_pub.publish(map_test);
        for(int i = 0; i < 3; i++) {
            odom_old[i] = odom_new[i];
        }   
        rate.sleep();
	}
    return 0;
}
