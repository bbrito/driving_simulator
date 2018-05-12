
#include <driving_simulator/DriverModel.h>
#include <fstream>

DriverModel::DriverModel(){

    if (!n_.getParam("obs_veh_num", obs_veh_num))
    {
        ROS_ERROR("Parameter 'obs_veh_num' not set");
        return;
    }

    if (!n_.getParam("radius_disks", radius_disks))
    {
        ROS_ERROR("Parameter 'radius_disks' not set");
        return;
    }

    if (!n_.getParam("Length", Length))
    {
        ROS_ERROR("Parameter 'Length' not set");
        return;
    }

    if (!n_.getParam("Width", Width))
    {
        ROS_ERROR("Parameter 'Width' not set");
        return;
    }
    vector<double> init_state;
    if (!n_.getParam("init_state", init_state))
    {
        ROS_ERROR("Parameter 'radius_disks' not set");
        return;
    }
    else{
        initi_ego_x = init_state[0];
        initi_ego_y = init_state[1];
        initi_ego_v = init_state[2];
    }
    vector<double> init_obs;
    if (!n_.getParam("init_obs", init_obs))
    {
        ROS_ERROR("Parameter 'radius_disks' not set");
        return;
    }
    else{
        initi_obs_x = init_state[0];
        initi_obs_y = init_state[1];
        initi_obs_v = init_state[2];
    }

    if (!n_.getParam("n_points_spline", n_points_spline))
    {
        ROS_ERROR("Parameter 'n_points_spline' not set");
        return;
    }

    if (!n_.getParam("N_SPLINE_POINTS", N_SPLINE_POINTS))
    {
        ROS_ERROR("Parameter 'N_SPLINE_POINTS' not set");
        return;
    }

    if (!n_.getParam("x_A_0", x_A_0))
    {
        ROS_ERROR("Parameter 'x_A_0' not set");
        return;
    }

    if (!n_.getParam("y_A_0", y_A_0))
    {
        ROS_ERROR("Parameter 'y_A_0' not set");
        return;
    }

    if (!n_.getParam("theta_A_0", theta_A_0))
    {
        ROS_ERROR("Parameter 'theta_A_0' not set");
        return;
    }

    if (!n_.getParam("x_A_1", x_A_1))
    {
        ROS_ERROR("Parameter 'x_A_1' not set");
        return;
    }

    if (!n_.getParam("y_A_1", y_A_1))
    {
        ROS_ERROR("Parameter 'y_A_1' not set");
        return;
    }

    if (!n_.getParam("theta_A_1", theta_A_1))
    {
        ROS_ERROR("Parameter 'theta_A_1' not set");
        return;
    }

    //pose_R = {initi_ego_x, initi_ego_y, PI/2};
    pose_R = {0, 0, 0, 0, 0, 0};
    state = {initi_obs_x, initi_obs_y, PI, initi_obs_v, 1, 0};

    poseR_sub = n_.subscribe("pose_R", 100, &DriverModel::Callback, this);

    ROS_INFO("Driver started");
}

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

	REF_PATH_A_0 = Ref_path(x_A_0, y_A_0, theta_A_0);

	REF_PATH_A_1 = Ref_path(x_A_1, y_A_1, theta_A_1);

}

void DriverModel::ConstSpeed(double speed){

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

void DriverModel::FriendlyDriver(vector<double>& state, vector<double> pos_R){
    ConstructRefPath();

    double x_R = pos_R[0];
    double y_R = pos_R[1];
    double x_A = state[0];
    double y_A = state[1];
    double v_A = state[3];
    double t = 0.1;

    double dist_RA = sqrt(pow((x_R-x_A),2) +pow((y_R-y_A),2));

    action.action = 2;
    if (1.5*Length < dist_RA && dist_RA < 2.5*Length){
        if (v_A > 2){
            state[3] = std::max(0.0, v_A - Acc * t);
            action.action = 1;
        }
    }
    else{
        if (dist_RA <= 1.5*Length && v_A!=0){
            state[3] = std::max(0.0, v_A - Acc * t);
            action.action = 1;
        }
    }

   if (dist_RA >=2.5 * Length){
    if (v_A < 3.0){
        state[3] = std::max(0.0, v_A + t);
        action.action = 0;
    }
   }


    double progress;
    progress = state[5] + state[3] * t;

    double dx, dy;
    if (state[4] == 1.0){
        state[0] = REF_PATH_A_1[0](progress);
        state[1] = REF_PATH_A_1[1](progress);
        dx = REF_PATH_A_1[0].deriv(1,progress);
        dy = REF_PATH_A_1[1].deriv(1,progress);
        state[2] = atan2(dy, dx);
        state[5] = progress;

    }
    else{
        state[0] = REF_PATH_A_0[0](progress);
        state[1] = REF_PATH_A_0[1](progress);
        dx = REF_PATH_A_0[0].deriv(1,progress);
        dy = REF_PATH_A_0[1].deriv(1,progress);
        state[2] = atan2(dy, dx);
        state[5] = progress;

    }

    state_A_ros.pose.clear();
    state_A_ros.pose = state;
    stateA_pub.publish(state_A_ros);
    action_pub.publish(action);

}

void DriverModel::ConservativeDriver(){
    double t = 0.1;
    ConstructRefPath();

    if (pose_R[0] < 25){
        if (state[3] < initi_obs_v){
            action.action = 0;
            state[3] += Acc * t;
        }
        else{
            action.action = 2;
        }

        double dx, dy;
        if (state[4] == 1.0){
            state[5] += state[3] *t;
            state[0] = REF_PATH_A_1[0](state[5]);
            state[1] = REF_PATH_A_1[1](state[5]);
            dx = REF_PATH_A_1[0].deriv(1,state[5]);
            dy = REF_PATH_A_1[1].deriv(1,state[5]);
            state[2] = atan2(dy, dx);

        }
        else{
            state[5] += state[3] *t;
            state[0] = REF_PATH_A_0[0](state[5]);
            state[1] = REF_PATH_A_0[1](state[5]);
            dx = REF_PATH_A_0[0].deriv(1,state[5]);
            dy = REF_PATH_A_0[1].deriv(1,state[5]);
            state[2] = atan2(dy, dx);
        }

    }
    else{
        if (state[0] <= 32){
            action.action = 1;
            state[3] = max(0.0, state[3]-Acc*t);
            double dx, dy;
            if (state[4] == 1.0){
                state[5] += state[3] *t;
                state[0] = REF_PATH_A_1[0](state[5]);
                state[1] = REF_PATH_A_1[1](state[5]);
                dx = REF_PATH_A_1[0].deriv(1,state[5]);
                dy = REF_PATH_A_1[1].deriv(1,state[5]);
                state[2] = atan2(dy, dx);

            }
            else{
                state[5] += state[3] *t;
                state[0] = REF_PATH_A_0[0](state[5]);
                state[1] = REF_PATH_A_0[1](state[5]);
                dx = REF_PATH_A_0[0].deriv(1,state[5]);
                dy = REF_PATH_A_0[1].deriv(1,state[5]);
                state[2] = atan2(dy, dx);
            }

        }

        else{
            action.action = 2;
            double dx, dy;
            if (state[4] == 1.0){
                state[5] += state[3] *t;
                state[0] = REF_PATH_A_1[0](state[5]);
                state[1] = REF_PATH_A_1[1](state[5]);
                dx = REF_PATH_A_1[0].deriv(1,state[5]);
                dy = REF_PATH_A_1[1].deriv(1,state[5]);
                state[2] = atan2(dy, dx);

            }
            else{
                state[5] += state[3] *t;
                state[0] = REF_PATH_A_0[0](state[5]);
                state[1] = REF_PATH_A_0[1](state[5]);
                dx = REF_PATH_A_0[0].deriv(1,state[5]);
                dy = REF_PATH_A_0[1].deriv(1,state[5]);
                state[2] = atan2(dy, dx);
            }


        }
    }

    state_A_ros.pose.clear();
    state_A_ros.pose = state;
    stateA_pub.publish(state_A_ros);
    action_pub.publish(action);


}

void DriverModel::Callback(const driving_simulator_msgs::State::ConstPtr& msg){
    pose_R = msg->pose;
}
