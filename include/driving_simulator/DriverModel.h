#ifndef DRIVERMODEL_H_
#define DRIVERMODEL_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <driving_simulator_msgs/Belief.h>
#include <driving_simulator_msgs/Traj.h>
#include <driving_simulator_msgs/Waypoint.h>
#include <driving_simulator_msgs/State.h>
#include <driving_simulator_msgs/Action.h>
#include "/home/bdebrito/code/ProbabilisticMotionPlanning-MasterThesis/POMDP/POMDP/src/spline.h"
#include "/home/bdebrito/code/ProbabilisticMotionPlanning-MasterThesis/POMDP/POMDP/src/Clothoid.h"

#include <vector>
#include <cmath>

#include <typeinfo>
const double PI = 3.1415926;

using namespace std;
class DriverModel {
private:
    vector<tk::spline> REF_PATH_A_0, REF_PATH_A_1;
    double dist_spline_pts;

    ros::NodeHandle n_;
    ros::Publisher stateA_pub = n_.advertise<driving_simulator_msgs::State>("state_A", 100);
    ros::Publisher action_pub = n_.advertise<driving_simulator_msgs::Action>("action", 100);

    driving_simulator_msgs::State state_A_ros;
    driving_simulator_msgs::Action action;

    ros::Subscriber poseR_sub;
public:
	DriverModel();
	~DriverModel(){};

     double obs_veh_num;
     double radius_disks;
     double Length;
     double Width;
     double initi_ego_x;
     double initi_ego_y;
     double initi_ego_v;
     double initi_obs_x;
     double initi_obs_y;
     double initi_obs_v;
     int n_points_spline;
     int N_SPLINE_POINTS;
     double Acc;

	// Rfe path parameters
	vector<double> x_A_0;
	vector<double> y_A_0;
	vector<double> theta_A_0;
	vector<double> x_A_1;
	vector<double> y_A_1;
	vector<double> theta_A_1;

    vector<double> state;

    vector<double> pose_R;

    void ConstSpeed(double speed);

    void FriendlyDriver(vector<double>& state_A, vector<double> state_R);
    void ConservativeDriver();

    inline vector<tk::spline> Ref_path(vector<double> x, vector<double> y, vector<double> theta);

    void ConstructRefPath();

    void Callback(const driving_simulator_msgs::State::ConstPtr& msg);

};












#endif
