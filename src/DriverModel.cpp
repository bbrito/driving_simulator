
#include <driving_simulator/DriverModel.h>
#include <fstream>

const double PI = 3.1415926;
const double obs_veh_num = 4;
const double radius_disks = sqrt(4*4/pow(4*2,2)+2*2*0.25);
const double Length = 4;
const double Width = 2;
const double initi_ego_x = 27;
const double initi_ego_y = -15;
const double initi_ego_v = 3;
const double initi_obs_x = 40 + (0.5-0.5)*20;
const double initi_obs_y = 8;
const double initi_obs_v = 3;
const int n_points_spline = 100;
const int N_SPLINE_POINTS = 30;
const double Acc = 2;


vector<tk::spline> DriverModel::Ref_path(vector<double> x, vector<double> y, vector<double> theta) {

	double k, dk, L;
	vector<double> X(10), Y(10);
	vector<double> X_all, Y_all, S_all;
	double total_length=0;
	int n_clothoid = 20;
	S_all.push_back(0);



	for (int i = 0; i < x.size()-1; i++){
		Clothoid::buildClothoid(x[i], y[i], theta[i], x[i+1], y[i+1], theta[i+1], k, dk, L);

		Clothoid::pointsOnClothoid(x[i], y[i], theta[i], k, dk, L, n_clothoid, X, Y);
		if (i==0){
			X_all.insert(X_all.end(), X.begin(), X.end());
			Y_all.insert(Y_all.end(), Y.begin(), Y.end());
		}
		else{
			X.erase(X.begin()+0);
			Y.erase(Y.begin()+0);
			X_all.insert(X_all.end(), X.begin(), X.end());
			Y_all.insert(Y_all.end(), Y.begin(), Y.end());
		}
		total_length = total_length + L;
		for (int j=1; j< n_clothoid; j++){
				S_all.push_back(S_all[j-1+i*(n_clothoid-1)]+L/(n_clothoid-1));
		}

	}

	tk::spline ref_path_x, ref_path_y;
	ref_path_x.set_points(S_all, X_all);
	ref_path_y.set_points(S_all, Y_all);


	dist_spline_pts = total_length / n_points_spline;
	vector<double> ss(n_points_spline),xx(n_points_spline),yy(n_points_spline);

	for (int i=0; i<n_points_spline; i++){
		ss[i] = dist_spline_pts *i;
		xx[i] = ref_path_x(ss[i]);
		yy[i] = ref_path_y(ss[i]);
	}
	ref_path_x.set_points(ss,xx);
	ref_path_y.set_points(ss,yy);

	vector<tk::spline> ref_path_(2);
	ref_path_[0] = ref_path_x;
	ref_path_[1] = ref_path_y;

	return ref_path_;
}

void DriverModel::ConstructRefPath(){
    vector<double> x_A_0 = {initi_obs_x, 0};
	vector<double> y_A_0 = {initi_obs_y, 8};
	vector<double> theta_A_0 = {PI, PI};
	vector<double> x_A_1 = {initi_obs_x, 30, 23, 23};
	vector<double> y_A_1 = {initi_obs_y, 8, 0, -50};
	vector<double> theta_A_1 = {PI, PI, -PI/2, -PI/2};


	REF_PATH_A_0 = Ref_path(x_A_0, y_A_0, theta_A_0);

	REF_PATH_A_1 = Ref_path(x_A_1, y_A_1, theta_A_1);

}

void DriverModel::ConstSpeed(double speed, vector<double>& state){

    ConstructRefPath();
    // state={x,y.theta,v,g,s}
    double prev_progress = state[5];
    state[3] = speed;

    double t = 0.1;
    double progress = prev_progress + speed * t;
    double dx, dy;
    if (state[4] == 1){
        state[0] = REF_PATH_A_1[0](progress);
        state[1] = REF_PATH_A_1[1](progress);
        dx = REF_PATH_A_1[0].deriv(1,progress);
        dy = REF_PATH_A_1[1].deriv(1,progress);
        state[2] = atan2(dy, dx);

        // // constant acceleration
        //state[3] = std::min(4.0, state[3] + 0.5* t);
        //state[5] += state[3] * t;

    }
    else{
        state[0] = REF_PATH_A_0[0](progress);
        state[1] = REF_PATH_A_0[1](progress);
        dx = REF_PATH_A_0[0].deriv(1,progress);
        dy = REF_PATH_A_0[1].deriv(1,progress);
        state[2] = atan2(dy, dx);

        // // constant acceleration
        //state[3] = std::min(4.0,state[3] + 0.5* t);
        //state[5] += state[3] * t;

    }

    state[5] = progress;

    state_A_ros.pose.clear();
    state_A_ros.pose = state;
    stateA_pub.publish(state_A_ros);

    action.action = 2;
    action_pub.publish(action);


}

void DriverModel::FriendlyDriver(vector<double>& state_A, vector<double> state_R){
    ConstructRefPath();

    double x_R = state_R[0];
    double y_R = state_R[1];
    double x_A = state_A[0];
    double y_A = state_A[1];
    double v_A = state_A[3];
    double t = 0.1;

    double dist_RA = sqrt(pow((x_R-x_A),2) +pow((y_R-y_A),2));

    action.action = 2;
    if (1.5*Length < dist_RA && dist_RA < 2.5*Length){
        if (v_A > 2){
            state_A[3] = std::max(0.0, v_A - Acc * t);
            action.action = 1;
        }
    }
    else{
        if (dist_RA <= 1.5*Length && v_A!=0){
            state_A[3] = std::max(0.0, v_A - Acc * t);
            action.action = 1;
        }
    }

   if (dist_RA >=2.5 * Length){
    if (v_A < 3.0){
        state_A[3] = std::max(0.0, v_A + t);
        action.action = 0;
    }
   }


    double progress;
    progress = state_A[5] + state_A[3] * t;

    double dx, dy;
    if (state_A[4] == 1.0){
        state_A[0] = REF_PATH_A_1[0](progress);
        state_A[1] = REF_PATH_A_1[1](progress);
        dx = REF_PATH_A_1[0].deriv(1,progress);
        dy = REF_PATH_A_1[1].deriv(1,progress);
        state_A[2] = atan2(dy, dx);
        state_A[5] = progress;

    }
    else{
        state_A[0] = REF_PATH_A_0[0](progress);
        state_A[1] = REF_PATH_A_0[1](progress);
        dx = REF_PATH_A_0[0].deriv(1,progress);
        dy = REF_PATH_A_0[1].deriv(1,progress);
        state_A[2] = atan2(dy, dx);
        state_A[5] = progress;

    }

    state_A_ros.pose.clear();
    state_A_ros.pose = state_A;
    stateA_pub.publish(state_A_ros);
    action_pub.publish(action);

}

void DriverModel::ConservativeDriver(vector<double>& state_A, vector<double> state_R){
    double t = 0.1;
    ConstructRefPath();

    if (state_R[0] < 25){
        if (state_A[3] < initi_obs_v){
            action.action = 0;
            state_A[3] += Acc * t;
        }
        else{
            action.action = 2;
        }

        double dx, dy;
        if (state_A[4] == 1.0){
            state_A[5] += state_A[3] *t;
            state_A[0] = REF_PATH_A_1[0](state_A[5]);
            state_A[1] = REF_PATH_A_1[1](state_A[5]);
            dx = REF_PATH_A_1[0].deriv(1,state_A[5]);
            dy = REF_PATH_A_1[1].deriv(1,state_A[5]);
            state_A[2] = atan2(dy, dx);

        }
        else{
            state_A[5] += state_A[3] *t;
            state_A[0] = REF_PATH_A_0[0](state_A[5]);
            state_A[1] = REF_PATH_A_0[1](state_A[5]);
            dx = REF_PATH_A_0[0].deriv(1,state_A[5]);
            dy = REF_PATH_A_0[1].deriv(1,state_A[5]);
            state_A[2] = atan2(dy, dx);
        }

    }
    else{
        if (state_A[0] <= 32){
            action.action = 1;
            state_A[3] = max(0.0, state_A[3]-Acc*t);
            double dx, dy;
            if (state_A[4] == 1.0){
                state_A[5] += state_A[3] *t;
                state_A[0] = REF_PATH_A_1[0](state_A[5]);
                state_A[1] = REF_PATH_A_1[1](state_A[5]);
                dx = REF_PATH_A_1[0].deriv(1,state_A[5]);
                dy = REF_PATH_A_1[1].deriv(1,state_A[5]);
                state_A[2] = atan2(dy, dx);

            }
            else{
                state_A[5] += state_A[3] *t;
                state_A[0] = REF_PATH_A_0[0](state_A[5]);
                state_A[1] = REF_PATH_A_0[1](state_A[5]);
                dx = REF_PATH_A_0[0].deriv(1,state_A[5]);
                dy = REF_PATH_A_0[1].deriv(1,state_A[5]);
                state_A[2] = atan2(dy, dx);
            }

        }

        else{
            action.action = 2;
            double dx, dy;
            if (state_A[4] == 1.0){
                state_A[5] += state_A[3] *t;
                state_A[0] = REF_PATH_A_1[0](state_A[5]);
                state_A[1] = REF_PATH_A_1[1](state_A[5]);
                dx = REF_PATH_A_1[0].deriv(1,state_A[5]);
                dy = REF_PATH_A_1[1].deriv(1,state_A[5]);
                state_A[2] = atan2(dy, dx);

            }
            else{
                state_A[5] += state_A[3] *t;
                state_A[0] = REF_PATH_A_0[0](state_A[5]);
                state_A[1] = REF_PATH_A_0[1](state_A[5]);
                dx = REF_PATH_A_0[0].deriv(1,state_A[5]);
                dy = REF_PATH_A_0[1].deriv(1,state_A[5]);
                state_A[2] = atan2(dy, dx);
            }


        }
    }

    state_A_ros.pose.clear();
    state_A_ros.pose = state_A;
    stateA_pub.publish(state_A_ros);
    action_pub.publish(action);


}

vector<double> pose_R = {initi_ego_x, initi_ego_y, PI/2};
void Callback(const driving_simulator_msgs::State::ConstPtr& msg){
    pose_R = msg->pose;
}

int main(int argc, char** argv){

    // ROS connection
    ros::init(argc, argv, "DriverModel");
    ros::NodeHandle n;
    ros::Rate r(2.5);
    ros::Publisher veh_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    ros::Subscriber poseR_sub = n.subscribe("pose_R", 100, Callback);


    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker Obs_veh;
    Obs_veh.header.frame_id = "/my_frame";

    Obs_veh.ns = "obs_vehicle";

    Obs_veh.id = 1;

    Obs_veh.type = shape;

    Obs_veh.action = visualization_msgs::Marker::ADD;


    Obs_veh.scale.x = 4.0;
    Obs_veh.scale.y = 2.0;
    Obs_veh.scale.z = 1;


    Obs_veh.color.r = 0.0f;
    Obs_veh.color.g = 0.0f;
    Obs_veh.color.b = 1.0f;
    Obs_veh.color.a = 1.0;



    Obs_veh.lifetime = ros::Duration();


    Obs_veh.pose.position.x = initi_obs_x;
    Obs_veh.pose.position.y = initi_obs_y;
    Obs_veh.pose.position.z = 0;
    Obs_veh.pose.orientation.x = 0.0;
    Obs_veh.pose.orientation.y = 0.0;
    Obs_veh.pose.orientation.z = sin(PI/2);
    Obs_veh.pose.orientation.w = cos(PI/2);


    veh_pub.publish(Obs_veh);


    int count = 0;
    DriverModel Driver;


    vector<double> state_A = {initi_obs_x, initi_obs_y, PI, initi_obs_v, 1, 0};//x,y,theta,v,g,s
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<double> randgm(0.0, 1.0);
//    if (randgm(gen) < 0.5)
//        state_A[4]  = 1;
//    else
//        state_A[4]  = 0;

    double dis;
    double min_dis = 999;

    //ofstream outfile;
    //outfile.open("/home/bingyu/ProMotionPlan/ICRA/scaling/clearance3.txt");

    ros::Duration(6.2).sleep();
    while (count <= 220)
    {
        Driver.ConstSpeed(3.0, state_A);
        //Driver.ConservativeDriver(state_A, pose_R);
        //Driver.FriendlyDriver(state_A, pose_R);


        /**
        * Update Rviz
        **/
        Obs_veh.header.stamp = ros::Time::now();

        Obs_veh.pose.position.x = state_A[0];
        Obs_veh.pose.position.y = state_A[1];
        Obs_veh.pose.position.z = 0;
        Obs_veh.pose.orientation.x = 0.0;
        Obs_veh.pose.orientation.y = 0.0;
        Obs_veh.pose.orientation.z = sin(state_A[2]/2);
        Obs_veh.pose.orientation.w = cos(state_A[2]/2);

        /** record data **/
        dis = sqrt(std::pow((state_A[0] - pose_R[0]),2) + std::pow((state_A[1] - pose_R[1]),2));

        //outfile << dis << endl;


        veh_pub.publish(Obs_veh);
        count++;
        ros::spinOnce();
        r.sleep();
    }


    //outfile.close();

    return 0;
}
