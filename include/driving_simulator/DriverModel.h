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
#include <driving_simulator/spline.h>
#include <driving_simulator/Clothoid.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <driving_simulator_msgs/Sensor.h>
#include <vector>
#include <cmath>

#include <typeinfo>

#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
const double PI = 3.1415926;

using namespace std;
class DriverModel {
private:
    vector<tk::spline> REF_PATH_A_0, REF_PATH_A_1;
    double dist_spline_pts;

    ros::NodeHandle n_;
    ros::Publisher stateA_pub;
    ros::Publisher action_pub;
	ros::Publisher veh_pub;
	ros::Publisher sensor_pub_;

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

	string root_frame;
	string robot_frame_id,agent_frame_id;
	int driver_id;
	string driving_style;

	//Topic names
	string state_topic;
	string action_topic;
	string vis_topic;
	string sensor_data_topic;

	//TF
	tf2_ros::TransformBroadcaster state_pub_;

	//Visualization parameters
	vector<double> color;

    void ConstSpeed(double speed);

    void FriendlyDriver(vector<double>& state_A, vector<double> state_R);
    void ConservativeDriver();

    inline vector<tk::spline> Ref_path(vector<double> x, vector<double> y, vector<double> theta);

    void ConstructRefPath();

    void RobotPose(const driving_simulator_msgs::State::ConstPtr& msg);

	void visualize();

	void broadcastTF();

	void SimulateSensorMeasurements();

};












#endif
