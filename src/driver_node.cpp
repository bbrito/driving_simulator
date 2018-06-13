#include <ros/ros.h>
#include <driving_simulator/DriverModel.h>

int main(int argc, char** argv){

	// ROS connection
	ros::init(argc, argv, "DriverModel");
	ros::NodeHandle n;
	ros::Rate r(2.5);

	int count = 0;
	DriverModel Driver;

	ROS_INFO_STREAM("Initial condition " << "  " << Driver.state[0] << "  " << Driver.state[1] << "  " << Driver.state[2]
						<< "  " << Driver.state[3]<< "  " << Driver.state[4] << "  " << Driver.state[5]);

	ROS_INFO_STREAM("Waiting for /solver_is_on");
	ros::service::waitForService("/solver_is_on", -1);

	while (count <= 220)
	{

		if(Driver.driving_style=="Constant")
			Driver.ConstSpeed(3.0);

		if(Driver.driving_style=="Conservative")
			Driver.ConservativeDriver();
			//Driver.FriendlyDriver(state_A, pose_R);
		/**
		* Update Rviz
		**/
		Driver.broadcastTF();
		Driver.visualize();

		count++;
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

