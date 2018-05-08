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


using namespace std;
class DriverModel {
private:
    vector<tk::spline> REF_PATH_A_0, REF_PATH_A_1;
    double dist_spline_pts;

    ros::NodeHandle n_;
    ros::Publisher stateA_pub = n_.advertise<driving_simulator_msgs::State>("state_A_2", 100);
    ros::Publisher action_pub = n_.advertise<driving_simulator_msgs::Action>("action_2", 100);
    driving_simulator_msgs::State state_A_ros;
    driving_simulator_msgs::Action action;
public:
    DriverModel(){};
    ~DriverModel(){};


void ConstSpeed(double speed, vector<double>& state);

void FriendlyDriver(vector<double>& state_A, vector<double> state_R);
void ConservativeDriver(vector<double>& state_A, vector<double> state_R);

inline vector<tk::spline> Ref_path(vector<double> x, vector<double> y, vector<double> theta);

void ConstructRefPath();



};












#endif
