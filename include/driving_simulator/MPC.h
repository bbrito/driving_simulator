
#ifndef MPC_H_
#define MPC_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <driving_simulator_msgs/Belief.h>
#include <driving_simulator_msgs/Traj.h>
#include <driving_simulator_msgs/Waypoint.h>
#include <driving_simulator_msgs/State.h>
#include <driving_simulator_msgs/Action.h>
#include <driving_simulator/spline.h>
#include <driving_simulator/Clothoid.h>
#include "FORCESNLPsolver.h"
#include "/home/bdebrito/catkin_ws/src/driving_simulator/src/FORCESNLPsolver_casadi2forces.c"
#include <vector>
#include <cmath>

#include <typeinfo>



using namespace std;
class MPMPC {
private:
    ros::NodeHandle n_;
    ros::Publisher trajR_pub = n_.advertise<driving_simulator_msgs::Traj>("traj_R", 100);
    ros::Publisher pose_R_pub = n_.advertise<driving_simulator_msgs::State>("pose_R", 100);
	driving_simulator_msgs::Traj traj_R;
	driving_simulator_msgs::State pose_R;
    ros::Publisher marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    visualization_msgs::Marker line, cylinder;




	FORCESNLPsolver_params mpcparams;
	FORCESNLPsolver_output mpcoutput;
	FORCESNLPsolver_info mpcinfo;
	FORCESNLPsolver_extfunc pt2Function = &FORCESNLPsolver_casadi2forces;

	bool Flag = false;

	double time[50], dt[50];


public:
	MPMPC();
    vector<tk::spline> ref_path_R;
    double dist_spline_pts;


	void MPCSolver(vector<double>& state, double& solvetime);

	void MPCUpdateParams(vector<double> state_R_, vector< vector<driving_simulator_msgs::Waypoint> > est_traj_A_0, vector< vector<driving_simulator_msgs::Waypoint> > est_traj_A_1, vector< vector<double> > beliefs);

	vector<double> Uncertainty(double time, double weight);

	inline vector<tk::spline> Ref_path(vector<double> x, vector<double> y, vector<double> theta, double& dist_spline_pts );

	inline vector<double> Rotation(double angle){
		vector<double> rotation(4);
		rotation[0] = cos(angle);
		rotation[1] = sin(angle);
		rotation[2] = -sin(angle);
		rotation[3] = cos(angle);
		return rotation;
	};



};




#endif
