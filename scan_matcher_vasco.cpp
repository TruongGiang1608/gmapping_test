#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

// struct pose{
//     vector<double> x;
//     vector<double> y;
//     vector<double> phi;

// };
double odom_s[3], odom_t[3];
void odomCallback(const nav_msgs::Odometry& msg) {
	odom_t[0] = odom_s[0];
	odom_t[1] = odom_s[1];
	odom_t[2] = odom_s[2];
	
	odom_s[0] = msg.pose.pose.position.x;
	odom_s[1] = msg.pose.pose.position.y;
	odom_s[2] = tf::getYaw(msg.pose.pose.orientation);
}
const int M = 1;
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
void poseGuess(double odom_pose_suc[3], double odom_pose_pre[3], double pose_est_pre[3][M], double (&pose_t)[3][M]) {
    for (int i = 0; i < M; i++) {
        pose_t[0][i] = pose_est_pre[0][i] + odom_pose_suc[0] - odom_pose_pre[0];
        pose_t[1][i] = pose_est_pre[1][i] + odom_pose_suc[1] - odom_pose_pre[1];
        pose_t[2][i] = pose_est_pre[2][i] + odom_pose_suc[2] - odom_pose_pre[2];
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "scan_matcher_vasco");
    ros::NodeHandle nh;
    ros::Publisher pose_guess_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_guess", 1000);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);

    tf2_ros::TransformBroadcaster pose_broadcaster;
    ROS_INFO("Publishing pose samples over ROS");

    double pose_t[3][M];
	SampleInitial(pose_t);
    ros::Rate rate(50);
    while(ros::ok()) {
        ros::spinOnce();
        poseGuess(odom_s, odom_t, pose_t, pose_t);
        //for(int i = 0; i < M; i++) {
            geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(pose_t[2][0]);

            geometry_msgs::TransformStamped pose_trans;
            pose_trans.header.stamp = ros::Time::now();
            pose_trans.header.frame_id = "map";
            pose_trans.child_frame_id = "base_footprint";
    
            pose_trans.transform.translation.x = pose_t[0][0];
            pose_trans.transform.translation.y = pose_t[1][0];
            pose_trans.transform.translation.z = 0.0;
            pose_trans.transform.rotation = pose_quat;
    
            pose_broadcaster.sendTransform(pose_trans);

            geometry_msgs::PoseStamped pose_guess;
            pose_guess.header.stamp = ros::Time::now();
            pose_guess.header.frame_id = "map";

            geometry_msgs::Pose p;
            pose_guess.pose.position.x = pose_t[0][0];
            pose_guess.pose.position.y = pose_t[1][0];
            pose_guess.pose.position.z = 0.0;
            pose_guess.pose.orientation = pose_quat;
        //}
        pose_guess_pub.publish(pose_guess);
        rate.sleep();
        
    }

}
