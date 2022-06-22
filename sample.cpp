#include <iostream>
#include <math.h>
#include <random>
#include <vector>
using namespace std;
/* Sampling x(i)_t ~ p(x(i)_t | x(i)_t-1, u_t) */
const int M = 3;
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
int main() {
    double p;
    double a[] = {1, 2, 1};
    double b[] = {0.2, 1, 0.6};
    double c[3][3] = {{1,1,1}, {1,0.8,0.8}, {0,0,0}};
    double x[3][M];

    SampleMotionModel(a, b, c, x);
	cout << x[0][1] << endl << x[0][2] << endl << x[0][0];
	//cout << "VScode";

    return 0;
}
