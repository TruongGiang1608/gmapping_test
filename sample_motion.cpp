#include <iostream>
#include <math.h>
#include <random>
#include <unistd.h>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
using namespace std;
double odom_pose_s[3], odom_pose_tr[3];
void odomCallback(const nav_msgs::Odometry& msg) {
	odom_pose_tr[0] = odom_pose_s[0];
	odom_pose_tr[1] = odom_pose_s[1];
	odom_pose_tr[2] = odom_pose_s[2];
	
	odom_pose_s[0] = msg.pose.pose.position.x;
	odom_pose_s[1] = msg.pose.pose.position.y;
	odom_pose_s[2] = tf::getYaw(msg.pose.pose.orientation);
}
/* Sampling x(i)_t ~ p(x(i)_t | x(i)_t-1, u_t) */
const int M = 30;
void SampleInitial(double (&initial_pose)[3][M]) {
	random_device s0;
	mt19937 rng0(s0());
	uniform_real_distribution<double> uni(0.0,1.0);
	for (int i = 0; i <= 2; i++) {
		for(int j = 0; j <= M-1; j++) {
			initial_pose[i][j] = uni(rng0);
		}
	}
}
		   
void SampleMotionModel(double odom_pose_suc[3], double odom_pose_pre[3], double pose_est_pre[3][M], double (&pose_est_suc)[3][M])
{ 
    // Parameters of the noise model
	const double anpha1 = 0.1; 
	const double anpha2 = 0.1; 
	const double anpha3 = 0.2; 
	const double anpha4 = 0.2;

	double del_rot1, del_trans, del_rot2;
	double sig_rot1, sig_trans, sig_rot2;

	del_rot1 = atan2(odom_pose_suc[1] - odom_pose_pre[1],odom_pose_suc[0] - odom_pose_pre[0]);
	del_trans = sqrt(pow(odom_pose_suc[0] - odom_pose_pre[0], 2) + pow(odom_pose_suc[1] - odom_pose_pre[1], 2));
	del_rot2 = odom_pose_suc[2] - odom_pose_pre[2] - del_rot1;

	sig_rot1 = sqrt(anpha1*pow(del_rot1, 2) + anpha2*pow(del_trans, 2));
	sig_trans = sqrt(anpha3*pow(del_trans, 2) + anpha4*pow(del_rot1, 2) + anpha4*pow(del_rot2, 2));
	sig_rot2 = sqrt(anpha1*pow(del_rot2, 2) + anpha2*pow(del_trans, 2));

	random_device s;
	mt19937 rng(s());
	normal_distribution<double> nor_dist_rot1(0, sig_rot1);
	normal_distribution<double> nor_dist_trans(0, sig_trans);
	normal_distribution<double> nor_dist_rot2(0, sig_rot2);

    double del_rot1_hat[M], del_trans_hat[M], del_rot2_hat[M];

	for (int i = 0; i <= M-1; i++) {
		del_rot1_hat[i] = del_rot1 - nor_dist_rot1(rng);
		del_trans_hat[i] = del_trans - nor_dist_trans(rng);
		del_rot2_hat[i] = del_rot2 - nor_dist_rot2(rng);	

		pose_est_suc[0][i] = pose_est_pre[0][i] + del_trans_hat[i]*cos(pose_est_pre[2][i] + del_rot1_hat[i]);
		pose_est_suc[1][i] = pose_est_pre[1][i] + del_trans_hat[i]*cos(pose_est_pre[2][i] + del_rot1_hat[i]);
		pose_est_suc[2][i] = pose_est_pre[2][i] + del_rot1_hat[i] + del_rot2_hat[i];
	}
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "sample_motion");
	ros::NodeHandle nh;
	ros::Publisher pose_pub = nh.advertise<std_msgs::Float64MultiArray>("pose_sample", 1000);
	ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback); 
	
	ROS_INFO("Publishing pose samples over ROS");
	ros::Rate rate(50);
	double pose_t[3][M];
	SampleInitial(pose_t);
	
	while (ros::ok()) {
	ros::spinOnce();
	SampleMotionModel(odom_pose_s, odom_pose_tr, pose_t, pose_t);
		
	std_msgs::Float64MultiArray pose_sample;
	//pose_sample.data.resize(3*M);
	for (int i = 0; i <= 2; i++) {
		for(int j = 0; j <= M-1; j++) {
	pose_sample.data.push_back(pose_t[i][j]);
		}
	}
		double sum1 = 0;
	double sum2 = 0;
	for (int i = 0; i <= M-1; i++) {
	 	sum1 += pose_t[0][i];
	}
	double muy = sum1/M;
	for (int i = 0; i <= M-1; i++) {
	 	sum2 += pow(pose_t[0][i] - muy, 2);
	}
	double var = sum2/M;
	cout << "Expect:" << muy << endl << "Variance" << var << endl;
	pose_pub.publish(pose_sample);
	rate.sleep();	
	}
		
    return 0;
}
