#include <ros/ros.h>
#include <driving_simulator/OnPOMDP.h>

const double PI = 3.1415926;

vector<driving_simulator_msgs::Waypoint> traj_R;
vector<double> pose_R= {0, 0, 0, 0, 0, 0}, state_A;
int action;
void Callback1(const driving_simulator_msgs::Traj::ConstPtr& msg){
	traj_R = msg->Traj;
}
void Callback2(const driving_simulator_msgs::State::ConstPtr& msg){
	pose_R = msg->pose;
}
void Callback3(const driving_simulator_msgs::State::ConstPtr& msg){
	state_A = msg->pose;
}
void Callback4(const driving_simulator_msgs::Action::ConstPtr& msg){
	action = msg->action;
}

int main(int argc, char **argv)
{
	// ROS connection
	ros::init(argc, argv, "DESPOT");
	ros::NodeHandle n;
	ros::Rate r(2.5);
	ros::Subscriber trajR_sub = n.subscribe("traj_R", 100, Callback1);
	ros::Subscriber pose_R_sub = n.subscribe("pose_R", 100, Callback2);
	string listen_state;

	if (!n.getParam(ros::this_node::getName()+"/listen_state", listen_state))
	{
		ROS_ERROR_STREAM("Parameter " << ros::this_node::getName()+"/listen_state not set");
		return 0;
	}

	string listen_action;

	if (!n.getParam(ros::this_node::getName()+"/listen_action", listen_action))
	{
		ROS_ERROR("Parameter 'listen_action' not set");
		return 0;
	}

	ros::Subscriber state_A_sub = n.subscribe(listen_state, 100, Callback3);
	ros::Subscriber action_sub = n.subscribe(listen_action, 100, Callback4);
	ROS_WARN_STREAM("POMDP Started");

	int count = 0;
	despot::POMDP POMDP_Planner;

	state_A = {POMDP_Planner.initi_obs_x, POMDP_Planner.initi_obs_y, PI, POMDP_Planner.initi_obs_v, 1, 0};//x,y,theta,v,g,s

	double start = ros::Time::now().toSec();
	POMDP_Planner.Initialization(argc, argv);
	cout << "time for initialization:" << ros::Time::now().toSec()-start << endl;

	//Initial trajectory of ego-vehicle

	double dx, dy, s;
	driving_simulator_msgs::Waypoint point;

	vector<double> x_R = {POMDP_Planner.initi_ego_x, 27, 12, -50};
	vector<double> y_R = {POMDP_Planner.initi_ego_y, -2, 8, 8};
	vector<double> theta_R = {PI/2, PI/2, PI, PI};
	vector<tk::spline> ref_path_R = POMDP_Planner.Ref_path(x_R, y_R, theta_R);

	for (int i=0; i<50; i++){
		s = POMDP_Planner.initi_ego_v*0.1*i;
		point.x = ref_path_R[0](s);
		point.y = ref_path_R[1](s);
		dx = ref_path_R[0].deriv(1,s);
		dy = ref_path_R[1].deriv(1,s);
		point.theta = atan2(dy, dx);
		traj_R.push_back(point);
	}

	//ofstream outfile;
	//outfile.open("/home/bingyu/ProMotionPlan/ICRA/scaling/DESPOT_solvetime3.txt");
	double solvetime;
	ROS_WARN_STREAM("POMDP Started");
	while (count <= 220)
	{
		cout << "--------------------Round " << count << "----------------" << endl;

		/**
		* Update the DESPOT solver
		**/
		if (count>0){

			POMDP_Planner.POMDP_Update(action, pose_R, state_A);

		}

		/**
		 * Anticipation of obstacle vehicles' motion
		 */
		start = ros::Time::now().toSec();
		int a = POMDP_Planner.POMDP_Solver(state_A, traj_R);
		solvetime = ros::Time::now().toSec() - start;


		/** record data **/

		//outfile << solvetime << endl;

		count++;
		ros::spinOnce();
		r.sleep();

	}
	//outfile.close();
	return 0;
}

