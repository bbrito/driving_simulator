#include <ros/ros.h>
#include <driving_simulator/DriverModel.h>

int main(int argc, char** argv){

	// ROS connection
	ros::init(argc, argv, "DriverModel");
	ros::NodeHandle n;
	ros::Rate r(2.5);
	ros::Publisher veh_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 100);

	uint32_t shape = visualization_msgs::Marker::CUBE;

	visualization_msgs::Marker Obs_veh;
	Obs_veh.header.frame_id = "/my_frame";

	Obs_veh.ns = n.getNamespace();

	if (!n.getParam(n.getNamespace()+"/DriverModel/Id", Obs_veh.id))
	{
		ROS_ERROR("Parameter 'Obs_veh.id' not set");
		return 0;
	}
	string driving_style;
	if (!n.getParam(n.getNamespace()+"/DriverModel/driving_style", driving_style))
	{
		ROS_ERROR("Parameter 'driving_style' not set");
		return 0;
	}

	vector<double> color;
	if (!n.getParam(n.getNamespace()+"/DriverModel/color", color))
	{
		ROS_ERROR("Parameter 'color' not set");
		return 0;
	}

	Obs_veh.type = shape;

	Obs_veh.action = visualization_msgs::Marker::ADD;


	Obs_veh.scale.x = 4.0;
	Obs_veh.scale.y = 2.0;
	Obs_veh.scale.z = 1;


	Obs_veh.color.r = color[0];
	Obs_veh.color.g = color[1];
	Obs_veh.color.b = color[2];
	Obs_veh.color.a = color[3];

	Obs_veh.lifetime = ros::Duration();

	int count = 0;
	DriverModel Driver;

//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<double> randgm(0.0, 1.0);
//    if (randgm(gen) < 0.5)
//        state_A[4]  = 1;
//    else
//        state_A[4]  = 0;
	ROS_INFO_STREAM("Initial condition " << Obs_veh.ns << "  " << Driver.state[0] << "  " << Driver.state[1] << "  " << Driver.state[2]
						<< "  " << Driver.state[3]<< "  " << Driver.state[4] << "  " << Driver.state[5]);

	double dis;
	double min_dis = 999;

	//ofstream outfile;
	//outfile.open("/home/bingyu/ProMotionPlan/ICRA/scaling/clearance3.txt");

	ros::Duration(130.2).sleep();
	while (count <= 220)
	{

		if(driving_style=="Constant")
			Driver.ConstSpeed(3.0);

		if(driving_style=="Conservative")
			Driver.ConservativeDriver();
		//Driver.FriendlyDriver(state_A, pose_R);


		/**
		* Update Rviz
		**/
		Obs_veh.header.stamp = ros::Time::now();

		Obs_veh.pose.position.x = Driver.state[0];
		Obs_veh.pose.position.y = Driver.state[1];
		Obs_veh.pose.position.z = 0;
		Obs_veh.pose.orientation.x = 0.0;
		Obs_veh.pose.orientation.y = 0.0;
		Obs_veh.pose.orientation.z = sin(Driver.state[2]/2);
		Obs_veh.pose.orientation.w = cos(Driver.state[2]/2);

		/** record data **/
		dis = sqrt(std::pow((Driver.state[0] - Driver.pose_R[0]),2) + std::pow((Driver.state[1] - Driver.pose_R[1]),2));

		//outfile << dis << endl;


		veh_pub.publish(Obs_veh);
		count++;
		ros::spinOnce();
		r.sleep();
	}


	//outfile.close();

	return 0;
}

