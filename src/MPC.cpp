

#include <driving_simulator/MPC.h>
#include <algorithm>
#include <std_srvs/Empty.h>
#include <sstream>
#include <stdio.h>
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

const double initi_obs_x_1 = -5 + (0.65-0.5)*20;
const double initi_obs_y_1 = 4;

const double initi_obs_x_2 = 50 + (0.65-0.5)*20;
const double initi_obs_y_2 = 8;

const double initi_obs_x_3 = -20 + (0.65-0.5)*20;
const double initi_obs_y_3 = 4;

const double cmd_steer_angle_max     = 0.4;
const double cmd_acc_max             = 2.0;
const double cmd_steer_vel_max       = 2.0;
const double max_turn_rate           = 50.0;
const double theta_max               = 1.5*PI;
const double vel_max                 = 3.5;
const double vel_min                 = 0.3;
const double delta_theta_max         = 0.4;


vector<tk::spline> MPMPC::Ref_path(vector<double> x, vector<double> y, vector<double> theta, double& dist_spline_pts) {

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



MPMPC::MPMPC(){

    /*=====================================
	Build spline path for the ego-vehicle
	=======================================*/
	vector<double> x_R = {initi_ego_x, 27, 12, -50};
	vector<double> y_R = {initi_ego_y, -2, 8, 8};
	vector<double> theta_R = {PI/2, PI/2, PI, PI};
	ref_path_R = Ref_path(x_R, y_R, theta_R, dist_spline_pts);


	/*============================================
	Initialization of parameters
	=============================================*/
	mpcparams.x0[0] = 0;
	mpcparams.x0[1] = 0;
	mpcparams.x0[2] = 0; //initial value of slack
	mpcparams.x0[3] = initi_ego_x; // initial x position of ego-vehicle
	mpcparams.x0[4] = initi_ego_y; //initial y position of ego-vehicel
	mpcparams.x0[5] = PI/2; // initial orientation of ego-vehicle
	mpcparams.x0[6] = 0;
	mpcparams.x0[7] = 0;
	mpcparams.x0[8] = 0;

	for (int i=9; i<442; i=i+9){
		mpcparams.x0[i] = mpcparams.x0[0];
		mpcparams.x0[i+1] = mpcparams.x0[1];
		mpcparams.x0[i+2] = mpcparams.x0[2];
		mpcparams.x0[i+3] = mpcparams.x0[3];
		mpcparams.x0[i+4] = mpcparams.x0[4];
		mpcparams.x0[i+5] = mpcparams.x0[5];
		mpcparams.x0[i+6] = mpcparams.x0[6];
		mpcparams.x0[i+7] = mpcparams.x0[7];
		mpcparams.x0[i+8] = mpcparams.x0[8];
	}


	mpcparams.xinit[0] = initi_ego_x;
	mpcparams.xinit[1] = initi_ego_y;
	mpcparams.xinit[2] = PI/2;
	mpcparams.xinit[3] = 0;
	mpcparams.xinit[4] = initi_ego_v;
	mpcparams.xinit[5] = 0;


	for (int i=0; i<75000;i++)
		mpcparams.all_parameters[i] = 0;


	for (int i=0; i<50; i++){
		if (i<40)
			dt[i] = 0.1;
		else
			dt[i] = 0.2;
	}


	time[0] = 0;
	for (int i=1; i<50; i++){
		time[i] = time[i-1] + dt[i-1];
	}
	for (int i=0; i<50; i++){

		int k = i*1500;
		double end_fac;
		if (i > 39)
            end_fac = 1;
        else
            end_fac = 0;



		mpcparams.all_parameters[k] = dt[i];
		mpcparams.all_parameters[k+1] = n_points_spline;
		mpcparams.all_parameters[k+2] = dist_spline_pts;
		mpcparams.all_parameters[k+3] = 0;
		mpcparams.all_parameters[k+19] = cmd_steer_angle_max;
		mpcparams.all_parameters[k+20] = cmd_acc_max;
		mpcparams.all_parameters[k+21] = cmd_steer_vel_max;
		mpcparams.all_parameters[k+22] = max_turn_rate;
		mpcparams.all_parameters[k+23] = theta_max;
		mpcparams.all_parameters[k+24] = delta_theta_max;
		mpcparams.all_parameters[k+25] = vel_max;
		mpcparams.all_parameters[k+26] = vel_min;

		mpcparams.all_parameters[k+41] = 0;
		mpcparams.all_parameters[k+42] = 1;
		mpcparams.all_parameters[k+43] = end_fac;

		mpcparams.all_parameters[k+59] = Length;
		mpcparams.all_parameters[k+60] = Width;
		mpcparams.all_parameters[k+61] = radius_disks;

	}


	/*==============================
	Initialization of Marker
	================================*/
	line.type = visualization_msgs::Marker::LINE_STRIP;
    line.id = 5;
    line.scale.x = 0.1;
	line.scale.y = 0.1;
	line.scale.z = 0.1;
    line.color.r = 1.0f;
    line.color.a = 1.0;
    line.header.frame_id = "/my_frame";
    line.ns = "trajectory";
    line.action = visualization_msgs::Marker::ADD;
    line.lifetime = ros::Duration(0.4);

    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.id = 9;
    cylinder.scale.x = 1;
    cylinder.scale.y = 1;
    cylinder.scale.z = 0.5;
    cylinder.color.a = 1.0;
    cylinder.header.frame_id = "/my_frame";
    cylinder.ns = "trajectory";
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.lifetime = ros::Duration(0.4);


}

vector<double> MPMPC::Uncertainty(double time, double weight){
	double sigma_s0 = 0.01;
	double sigma_s_delta = 0.05 ;
	double sigma_s_max = 3;
	double sigma_d0 = 0.001;
	double sigma_d_delta = 0.01;
	double sigma_d_max = 0.4;
	double p_thresh = 0.01;
	vector<double> uncertainty(2);
	double a_uncertainty, b_uncertainty;

	double sigma_s = min(sigma_s0 + sigma_s_delta*time, sigma_s_max);
	double sigma_d = min(sigma_d0 + sigma_d_delta*time, sigma_d_max);



	if (1.0/(2*PI*sigma_s*sigma_d) >  p_thresh / weight && sigma_s*sigma_d > sqrt(0.002)){
		a_uncertainty = sigma_s* sqrt((log(p_thresh/weight) + log(2*PI*sigma_s*sigma_d))*-2);
        b_uncertainty = sigma_d* sqrt((log(p_thresh/weight) + log(2*PI*sigma_s*sigma_d))*-2);
    }
    else{
        a_uncertainty = 0;
        b_uncertainty = 0;
    }
    uncertainty[0] = a_uncertainty;
    uncertainty[1] = b_uncertainty;
    return uncertainty;
}


void MPMPC::MPCUpdateParams(vector<double> state_R_, vector< vector<driving_simulator_msgs::Waypoint> > est_traj_A_0, vector< vector<driving_simulator_msgs::Waypoint> > est_traj_A_1, vector< vector<double> > beliefs){
    // update initial guess for the solver
    if (Flag){
    std::copy(mpcoutput.x02, mpcoutput.x02+9, mpcparams.x0);
    std::copy(mpcoutput.x03, mpcoutput.x03+9, mpcparams.x0+9);
    std::copy(mpcoutput.x04, mpcoutput.x04+9, mpcparams.x0+18);
    std::copy(mpcoutput.x05, mpcoutput.x05+9, mpcparams.x0+27);
    std::copy(mpcoutput.x06, mpcoutput.x06+9, mpcparams.x0+36);
    std::copy(mpcoutput.x07, mpcoutput.x07+9, mpcparams.x0+45);
    std::copy(mpcoutput.x08, mpcoutput.x08+9, mpcparams.x0+54);
    std::copy(mpcoutput.x09, mpcoutput.x09+9, mpcparams.x0+63);
    std::copy(mpcoutput.x10, mpcoutput.x10+9, mpcparams.x0+72);
    std::copy(mpcoutput.x11, mpcoutput.x11+9, mpcparams.x0+81);
    std::copy(mpcoutput.x12, mpcoutput.x12+9, mpcparams.x0+90);
    std::copy(mpcoutput.x13, mpcoutput.x13+9, mpcparams.x0+99);
    std::copy(mpcoutput.x14, mpcoutput.x14+9, mpcparams.x0+108);
    std::copy(mpcoutput.x15, mpcoutput.x15+9, mpcparams.x0+9*13);
    std::copy(mpcoutput.x16, mpcoutput.x16+9, mpcparams.x0+9*14);
    std::copy(mpcoutput.x17, mpcoutput.x17+9, mpcparams.x0+9*15);
    std::copy(mpcoutput.x18, mpcoutput.x18+9, mpcparams.x0+9*16);
    std::copy(mpcoutput.x19, mpcoutput.x19+9, mpcparams.x0+9*17);
    std::copy(mpcoutput.x20, mpcoutput.x20+9, mpcparams.x0+9*18);
    std::copy(mpcoutput.x21, mpcoutput.x21+9, mpcparams.x0+9*19);
    std::copy(mpcoutput.x22, mpcoutput.x22+9, mpcparams.x0+9*20);
    std::copy(mpcoutput.x23, mpcoutput.x23+9, mpcparams.x0+9*21);
    std::copy(mpcoutput.x24, mpcoutput.x24+9, mpcparams.x0+9*22);
    std::copy(mpcoutput.x25, mpcoutput.x25+9, mpcparams.x0+9*23);
    std::copy(mpcoutput.x26, mpcoutput.x26+9, mpcparams.x0+9*24);
    std::copy(mpcoutput.x27, mpcoutput.x27+9, mpcparams.x0+9*25);
    std::copy(mpcoutput.x28, mpcoutput.x28+9, mpcparams.x0+9*26);
    std::copy(mpcoutput.x29, mpcoutput.x29+9, mpcparams.x0+9*27);
    std::copy(mpcoutput.x30, mpcoutput.x30+9, mpcparams.x0+9*28);
    std::copy(mpcoutput.x31, mpcoutput.x31+9, mpcparams.x0+9*29);
    std::copy(mpcoutput.x32, mpcoutput.x32+9, mpcparams.x0+9*30);
    std::copy(mpcoutput.x33, mpcoutput.x33+9, mpcparams.x0+9*31);
    std::copy(mpcoutput.x34, mpcoutput.x34+9, mpcparams.x0+9*32);
    std::copy(mpcoutput.x35, mpcoutput.x35+9, mpcparams.x0+9*33);
    std::copy(mpcoutput.x36, mpcoutput.x36+9, mpcparams.x0+9*34);
    std::copy(mpcoutput.x37, mpcoutput.x37+9, mpcparams.x0+9*35);
    std::copy(mpcoutput.x38, mpcoutput.x38+9, mpcparams.x0+9*36);
    std::copy(mpcoutput.x39, mpcoutput.x39+9, mpcparams.x0+9*37);
    std::copy(mpcoutput.x40, mpcoutput.x40+9, mpcparams.x0+9*38);
    std::copy(mpcoutput.x41, mpcoutput.x41+9, mpcparams.x0+9*39);
    std::copy(mpcoutput.x42, mpcoutput.x42+9, mpcparams.x0+9*40);
    std::copy(mpcoutput.x43, mpcoutput.x43+9, mpcparams.x0+9*41);
    std::copy(mpcoutput.x44, mpcoutput.x44+9, mpcparams.x0+9*42);
    std::copy(mpcoutput.x45, mpcoutput.x45+9, mpcparams.x0+9*43);
    std::copy(mpcoutput.x46, mpcoutput.x46+9, mpcparams.x0+9*44);
    std::copy(mpcoutput.x47, mpcoutput.x47+9, mpcparams.x0+9*45);
    std::copy(mpcoutput.x48, mpcoutput.x48+9, mpcparams.x0+9*46);
    std::copy(mpcoutput.x49, mpcoutput.x49+9, mpcparams.x0+9*47);
    std::copy(mpcoutput.x50, mpcoutput.x50+9, mpcparams.x0+9*48);
    std::copy(mpcoutput.x50, mpcoutput.x50+9, mpcparams.x0+9*49);




    }


    double break_index = floor(mpcoutput.x02[8] / dist_spline_pts);
   // cout << "break index: " << break_index << endl;
    int spline_index = 500;
    int k;
    for (int i=0; i<50; i++){
        k = i*1500;
        // estimated obstacle vehicles' future trajectories
        mpcparams.all_parameters[k+3] = break_index;
        for (int q=1; q<=obs_veh_num; q++){

            vector<double> uncertainty = Uncertainty(time[i], 0.5);
            //cout << "Pass unc" << endl;
            mpcparams.all_parameters[k+98+20*q-19] = sqrt(2)/2*Length + radius_disks + uncertainty[0];
            mpcparams.all_parameters[k+98+20*q-18] = sqrt(2)/2*Width + radius_disks + uncertainty[1];
            //cout << "Pass 2"<< endl;
            mpcparams.all_parameters[k+98+20*q-17] = est_traj_A_0[q-1][i].x;
            //cout << "Pass 3" << endl;
            mpcparams.all_parameters[k+98+20*q-16] = est_traj_A_0[q-1][i].y;
            vector<double> orientation = Rotation(est_traj_A_0[q-1][i].theta);
            mpcparams.all_parameters[k+98+20*q-15] = orientation[0];
            mpcparams.all_parameters[k+98+20*q-14] = orientation[1];
            mpcparams.all_parameters[k+98+20*q-13] = orientation[2];
            mpcparams.all_parameters[k+98+20*q-12] = orientation[3];

            mpcparams.all_parameters[k+98+20*q-19 + 8] = sqrt(2)/2*Length + radius_disks + uncertainty[0];
            mpcparams.all_parameters[k+98+20*q-18 + 8] = sqrt(2)/2*Width + radius_disks + uncertainty[1];
            //cout << "Pass 2"<< endl;
            mpcparams.all_parameters[k+98+20*q-17 + 8] = est_traj_A_1[q-1][i].x;
            //cout << "Pass 3" << endl;
            mpcparams.all_parameters[k+98+20*q-16 + 8] = est_traj_A_1[q-1][i].y;
            orientation = Rotation(est_traj_A_1[q-1][i].theta);
            mpcparams.all_parameters[k+98+20*q-15 + 8] = orientation[0];
            mpcparams.all_parameters[k+98+20*q-14 + 8] = orientation[1];
            mpcparams.all_parameters[k+98+20*q-13 + 8] = orientation[2];
            mpcparams.all_parameters[k+98+20*q-12 + 8] = orientation[3];

           // cout << "trajectories of obs" << endl;
        }
        // Reference path for the ego-vehicle
        for (int j=1; j<=N_SPLINE_POINTS-1;j++){
            mpcparams.all_parameters[k + spline_index + (j-1)*8] = ref_path_R[0].m_a[break_index+j-1];
            mpcparams.all_parameters[k + spline_index + (j-1)*8 + 1] = ref_path_R[0].m_b[break_index+j-1];
            mpcparams.all_parameters[k + spline_index + (j-1)*8 + 2] = ref_path_R[0].m_c[break_index+j-1];
            mpcparams.all_parameters[k + spline_index + (j-1)*8 + 3] = ref_path_R[0].m_d[break_index+j-1];
            mpcparams.all_parameters[k + spline_index + (j-1)*8 + 4] = ref_path_R[1].m_a[break_index+j-1];
            mpcparams.all_parameters[k + spline_index + (j-1)*8 + 5] = ref_path_R[1].m_b[break_index+j-1];
            mpcparams.all_parameters[k + spline_index + (j-1)*8 + 6] = ref_path_R[1].m_c[break_index+j-1];
            mpcparams.all_parameters[k + spline_index + (j-1)*8 + 7] = ref_path_R[1].m_d[break_index+j-1];
        }
    }


    // Update initial state of ego-vehicle
    mpcparams.xinit[0] = state_R_[0];
	mpcparams.xinit[1] = state_R_[1];
	mpcparams.xinit[2] = state_R_[2];
	mpcparams.xinit[3] = state_R_[3];
	mpcparams.xinit[4] = state_R_[4];
	mpcparams.xinit[5] = state_R_[5];

	Flag = true;

	//for (int i=0; i<500; i++)
      //  cout << mpcparams.all_parameters[i] << endl;

}

void MPMPC::MPCSolver(vector<double>& state, double& solvetime){
    // Calling the FORCESPro solver
    int exitflag = FORCESNLPsolver_solve(&mpcparams, &mpcoutput, &mpcinfo, stdout, pt2Function);
    if (exitflag == 1){
		//cout << "Successful Planning :)"<< endl;
        // Next state
        state.clear();
        state.push_back(mpcoutput.x02[3]);
        state.push_back(mpcoutput.x02[4]);
        state.push_back(mpcoutput.x02[5]);
        state.push_back(mpcoutput.x02[6]);
        state.push_back(mpcoutput.x02[7]);
        state.push_back(mpcoutput.x02[8]);

        solvetime = mpcinfo.solvetime;

        pose_R.pose.clear();
        pose_R.pose.push_back(state[0]);
        pose_R.pose.push_back(state[1]);
        pose_R.pose.push_back(state[2]);
        pose_R.speed = state[4];
        pose_R.acc = mpcoutput.x02[1];
        pose_R.steer_vel = mpcoutput.x02[0];
        pose_R.steer = mpcoutput.x02[6];



        if (mpcoutput.x02[1] >= 0.1){
            cylinder.color.g = 1.0f;
            cylinder.color.r = 0;
            cylinder.color.b = 0;
        }
        if (mpcoutput.x02[1] <= -0.1){
            cylinder.color.r = 1.0f;
            cylinder.color.g = 0;
            cylinder.color.b = 0;
        }

        if (mpcoutput.x02[1] < 0.1 && mpcoutput.x02[1] >-0.1){
            cylinder.color.r = 0;
            cylinder.color.g = 0;
            cylinder.color.b = 1.0f;
        }

        cylinder.pose.position.x = state[0]+1.5;
        cylinder.pose.position.y = state[1]+2;
        cylinder.pose.position.z = 0;



        //Policy of ego vehicle
        traj_R.Traj.clear();
        line.points.clear();
		driving_simulator_msgs::Waypoint point;
        geometry_msgs::Point p;
        point.x = mpcoutput.x01[3]; point.y = mpcoutput.x01[4]; point.theta = mpcoutput.x01[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x02[3]; point.y = mpcoutput.x02[4]; point.theta = mpcoutput.x02[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x03[3]; point.y = mpcoutput.x03[4]; point.theta = mpcoutput.x03[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x04[3]; point.y = mpcoutput.x04[4]; point.theta = mpcoutput.x04[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x05[3]; point.y = mpcoutput.x05[4]; point.theta = mpcoutput.x05[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x06[3]; point.y = mpcoutput.x06[4]; point.theta = mpcoutput.x06[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x07[3]; point.y = mpcoutput.x07[4]; point.theta = mpcoutput.x07[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x08[3]; point.y = mpcoutput.x08[4]; point.theta = mpcoutput.x08[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x09[3]; point.y = mpcoutput.x09[4]; point.theta = mpcoutput.x09[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x10[3]; point.y = mpcoutput.x10[4]; point.theta = mpcoutput.x10[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x11[3]; point.y = mpcoutput.x11[4]; point.theta = mpcoutput.x11[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x12[3]; point.y = mpcoutput.x12[4]; point.theta = mpcoutput.x12[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x13[3]; point.y = mpcoutput.x13[4]; point.theta = mpcoutput.x13[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x14[3]; point.y = mpcoutput.x14[4]; point.theta = mpcoutput.x14[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x15[3]; point.y = mpcoutput.x15[4]; point.theta = mpcoutput.x15[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x16[3]; point.y = mpcoutput.x16[4]; point.theta = mpcoutput.x16[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x17[3]; point.y = mpcoutput.x17[4]; point.theta = mpcoutput.x17[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x18[3]; point.y = mpcoutput.x18[4]; point.theta = mpcoutput.x18[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x19[3]; point.y = mpcoutput.x19[4]; point.theta = mpcoutput.x19[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x20[3]; point.y = mpcoutput.x20[4]; point.theta = mpcoutput.x20[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x21[3]; point.y = mpcoutput.x21[4]; point.theta = mpcoutput.x21[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x22[3]; point.y = mpcoutput.x22[4]; point.theta = mpcoutput.x22[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x23[3]; point.y = mpcoutput.x23[4]; point.theta = mpcoutput.x23[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x24[3]; point.y = mpcoutput.x24[4]; point.theta = mpcoutput.x24[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x25[3]; point.y = mpcoutput.x25[4]; point.theta = mpcoutput.x25[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x26[3]; point.y = mpcoutput.x26[4]; point.theta = mpcoutput.x26[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x27[3]; point.y = mpcoutput.x27[4]; point.theta = mpcoutput.x27[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x28[3]; point.y = mpcoutput.x28[4]; point.theta = mpcoutput.x28[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x29[3]; point.y = mpcoutput.x29[4]; point.theta = mpcoutput.x29[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x30[3]; point.y = mpcoutput.x30[4]; point.theta = mpcoutput.x30[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x31[3]; point.y = mpcoutput.x31[4]; point.theta = mpcoutput.x31[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x32[3]; point.y = mpcoutput.x32[4]; point.theta = mpcoutput.x32[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x33[3]; point.y = mpcoutput.x33[4]; point.theta = mpcoutput.x33[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x34[3]; point.y = mpcoutput.x34[4]; point.theta = mpcoutput.x34[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x35[3]; point.y = mpcoutput.x35[4]; point.theta = mpcoutput.x35[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x36[3]; point.y = mpcoutput.x36[4]; point.theta = mpcoutput.x36[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x37[3]; point.y = mpcoutput.x37[4]; point.theta = mpcoutput.x37[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x38[3]; point.y = mpcoutput.x38[4]; point.theta = mpcoutput.x38[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x39[3]; point.y = mpcoutput.x39[4]; point.theta = mpcoutput.x39[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x40[3]; point.y = mpcoutput.x40[4]; point.theta = mpcoutput.x40[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x41[3]; point.y = mpcoutput.x41[4]; point.theta = mpcoutput.x41[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x42[3]; point.y = mpcoutput.x42[4]; point.theta = mpcoutput.x42[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x43[3]; point.y = mpcoutput.x43[4]; point.theta = mpcoutput.x43[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x44[3]; point.y = mpcoutput.x44[4]; point.theta = mpcoutput.x44[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x45[3]; point.y = mpcoutput.x45[4]; point.theta = mpcoutput.x45[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x46[3]; point.y = mpcoutput.x46[4]; point.theta = mpcoutput.x46[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x47[3]; point.y = mpcoutput.x47[4]; point.theta = mpcoutput.x47[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x48[3]; point.y = mpcoutput.x48[4]; point.theta = mpcoutput.x48[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x49[3]; point.y = mpcoutput.x49[4]; point.theta = mpcoutput.x49[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);
        point.x = mpcoutput.x50[3]; point.y = mpcoutput.x50[4]; point.theta = mpcoutput.x50[5];
        traj_R.Traj.push_back(point);
        p.x = point.x; p.y = point.y; p.z = 0;
        line.points.push_back(p);

        line.header.stamp = ros::Time::now();
        cylinder.header.stamp = ros::Time::now();
        trajR_pub.publish(traj_R);
        pose_R_pub.publish(pose_R);
        marker_pub.publish(line);
        marker_pub.publish(cylinder);

	}
	else
		cout << "Failed Planning :(" << endl;
}


 vector<driving_simulator_msgs::Waypoint> traj_A0, traj_A1, traj_A0_1, traj_A1_1, traj_A0_2, traj_A1_2, traj_A0_3, traj_A1_3;
 vector<double> belief={0.5, 0.5}, belief_1={0.5, 0.5}, belief_2={0.5, 0.5}, belief_3={0.5, 0.5};
void Callback1(const driving_simulator_msgs::Traj::ConstPtr& msg){
    traj_A0 = msg->Traj;
}
void Callback2(const driving_simulator_msgs::Traj::ConstPtr& msg){
    traj_A1 = msg->Traj;
}
void Callback3(const driving_simulator_msgs::Belief::ConstPtr& msg){
     belief = msg->belief;
}

void Callback4(const driving_simulator_msgs::Traj::ConstPtr& msg){
    traj_A0_1 = msg->Traj;
}
void Callback5(const driving_simulator_msgs::Traj::ConstPtr& msg){
    traj_A1_1 = msg->Traj;
}
void Callback6(const driving_simulator_msgs::Belief::ConstPtr& msg){
     belief_1 = msg->belief;
}

void Callback7(const driving_simulator_msgs::Traj::ConstPtr& msg){
    traj_A0_2 = msg->Traj;
}
void Callback8(const driving_simulator_msgs::Traj::ConstPtr& msg){
    traj_A1_2 = msg->Traj;
}
void Callback9(const driving_simulator_msgs::Belief::ConstPtr& msg){
     belief_2 = msg->belief;
}

void Callback10(const driving_simulator_msgs::Traj::ConstPtr& msg){
    traj_A0_3 = msg->Traj;
}
void Callback11(const driving_simulator_msgs::Traj::ConstPtr& msg){
    traj_A1_3 = msg->Traj;
}
void Callback12(const driving_simulator_msgs::Belief::ConstPtr& msg){
     belief_3 = msg->belief;
}

bool HelloFromMPC(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

	return true;
}

int main(int argc, char **argv)
{
    // ROS connection
    ros::init(argc, argv, "MultipolicyMPC");
    ros::NodeHandle n;
    ros::Rate r(2.5);
	ROS_WARN_STREAM("MPC Started");
    ros::Publisher veh_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Pose>("cmd", 100);
    ros::Subscriber trajA0_sub = n.subscribe("traj_A0", 100, Callback1);
    ros::Subscriber trajA1_sub = n.subscribe("traj_A1", 100, Callback2);
    ros::Subscriber belief_sub = n.subscribe("belief", 100, Callback3);

    ros::Subscriber trajA0_1_sub = n.subscribe("traj_A0_1", 100, Callback4);
    ros::Subscriber trajA1_1_sub = n.subscribe("traj_A1_1", 100, Callback5);
    ros::Subscriber belief_1_sub = n.subscribe("belief_1", 100, Callback6);

    ros::Subscriber trajA0_2_sub = n.subscribe("traj_A0_2", 100, Callback7);
    ros::Subscriber trajA1_2_sub = n.subscribe("traj_A1_2", 100, Callback8);
    ros::Subscriber belief_2_sub = n.subscribe("belief_2", 100, Callback9);

    ros::Subscriber trajA0_3_sub = n.subscribe("traj_A0_3", 100, Callback10);
    ros::Subscriber trajA1_3_sub = n.subscribe("traj_A1_3", 100, Callback11);
    ros::Subscriber belief_3_sub = n.subscribe("belief_3", 100, Callback12);

    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker Ego_veh;
    Ego_veh.header.frame_id = "/my_frame";

    Ego_veh.ns = "ego_vehicle";

    Ego_veh.id = 0;

    Ego_veh.type = shape;

    Ego_veh.action = visualization_msgs::Marker::ADD;


    Ego_veh.scale.x = 4.0;
    Ego_veh.scale.y = 2.0;
    Ego_veh.scale.z = 1;


    Ego_veh.color.r = 1.0f;
    Ego_veh.color.g = 0.0f;
    Ego_veh.color.b = 0.0f;
    Ego_veh.color.a = 1.0;



    Ego_veh.lifetime = ros::Duration();


    Ego_veh.pose.position.x = initi_ego_x;
    Ego_veh.pose.position.y = initi_ego_y;
    Ego_veh.pose.position.z = 0;
    Ego_veh.pose.orientation.x = 0.0;
    Ego_veh.pose.orientation.y = 0.0;
    Ego_veh.pose.orientation.z = sin(PI/4);
    Ego_veh.pose.orientation.w = cos(PI/4);


    veh_pub.publish(Ego_veh);


    int count = 0;
    MPMPC MPC_Planner;

    vector<double> state_R = {initi_ego_x, initi_ego_y, PI/2, 0, initi_ego_v, 0};//x,y,theta,steer angle,v,s

    vector<double> x_A_0 = {initi_obs_x, 0, -100};
	vector<double> y_A_0 = {initi_obs_y, 8, 8};
	vector<double> theta_A_0 = {PI, PI, PI};
	vector<double> x_A_1 = {initi_obs_x, 30, 23, 23};
	vector<double> y_A_1 = {initi_obs_y, 8, 0, -50};
	vector<double> theta_A_1 = {PI, PI, -PI/2, -PI/2};


	vector<double> x_A_0_1 = {initi_obs_x_1, 0, 50};
	vector<double> y_A_0_1 = {initi_obs_y_1, 4, 4};
	vector<double> theta_A_0_1 = {0, 0, 0};
	vector<double> x_A_1_1 = {initi_obs_x_1, 17, 23, 23};
	vector<double> y_A_1_1 = {initi_obs_y_1, 4, 0, -50};
	vector<double> theta_A_1_1 = {0, 0, -PI/2, -PI/2};

	vector<double> x_A_0_2 = {initi_obs_x_2, 0, 30};
	vector<double> y_A_0_2 = {initi_obs_y_2, 8, 8};
	vector<double> theta_A_0_2 = {PI, PI, PI};
	vector<double> x_A_1_2 = {initi_obs_x_2, 30, 23, 23};
	vector<double> y_A_1_2 = {initi_obs_y_2, 8, 0, -50};
	vector<double> theta_A_1_2 = {PI, PI, -PI/2, -PI/2};

	vector<double> x_A_0_3 = {initi_obs_x_3, 0, 30};
	vector<double> y_A_0_3 = {initi_obs_y_3, 4, 4};
	vector<double> theta_A_0_3 = {0, 0, 0};
	vector<double> x_A_1_3 = {initi_obs_x_3, 20, 23, 23};
	vector<double> y_A_1_3 = {initi_obs_y_3, 4, 0, -50};
	vector<double> theta_A_1_3 = {0, 0, -PI/2, -PI/2};

	double dist_spline;

	vector<tk::spline> REF_PATH_A_0 = MPC_Planner.Ref_path(x_A_0, y_A_0, theta_A_0, dist_spline);
	vector<tk::spline> REF_PATH_A_1 = MPC_Planner.Ref_path(x_A_1, y_A_1, theta_A_1, dist_spline);

	vector<tk::spline> REF_PATH_A_0_1 = MPC_Planner.Ref_path(x_A_0_1, y_A_0_1, theta_A_0_1, dist_spline);
	vector<tk::spline> REF_PATH_A_1_1 = MPC_Planner.Ref_path(x_A_1_1, y_A_1_1, theta_A_1_1, dist_spline);

	vector<tk::spline> REF_PATH_A_0_2 = MPC_Planner.Ref_path(x_A_0_2, y_A_0_2, theta_A_0_2, dist_spline);
	vector<tk::spline> REF_PATH_A_1_2 = MPC_Planner.Ref_path(x_A_1_2, y_A_1_2, theta_A_1_2, dist_spline);

	vector<tk::spline> REF_PATH_A_0_3 = MPC_Planner.Ref_path(x_A_0_3, y_A_0_3, theta_A_0_3, dist_spline);
	vector<tk::spline> REF_PATH_A_1_3 = MPC_Planner.Ref_path(x_A_1_3, y_A_1_3, theta_A_1_3, dist_spline);
	double dx, dy, s;
	driving_simulator_msgs::Waypoint point;

    /** obs 0 **/
	for (int i=0; i<50; i++){
        s = initi_obs_v*0.1*i;
        point.x = REF_PATH_A_0[0](s);
        point.y = REF_PATH_A_0[1](s);
        dx = REF_PATH_A_0[0].deriv(1,s);
        dy = REF_PATH_A_0[1].deriv(1,s);
        point.theta = atan2(dy, dx);
        traj_A0.push_back(point);
    }

    for (int i=0; i<50; i++){
        s = initi_obs_v*0.1*i;
        point.x = REF_PATH_A_1[0](s);
        point.y = REF_PATH_A_1[1](s);
        dx = REF_PATH_A_1[0].deriv(1,s);
        dy = REF_PATH_A_1[1].deriv(1,s);
        point.theta = atan2(dy, dx);
        traj_A1.push_back(point);
    }
    /** obs1 **/
    for (int i=0; i<50; i++){
        s = initi_obs_v*0.1*i;
        point.x = REF_PATH_A_0_1[0](s);
        point.y = REF_PATH_A_0_1[1](s);
        dx = REF_PATH_A_0_1[0].deriv(1,s);
        dy = REF_PATH_A_0_1[1].deriv(1,s);
        point.theta = atan2(dy, dx);
        traj_A0_1.push_back(point);
    }

    for (int i=0; i<50; i++){
        s = initi_obs_v*0.1*i;
        point.x = REF_PATH_A_1_1[0](s);
        point.y = REF_PATH_A_1_1[1](s);
        dx = REF_PATH_A_1_1[0].deriv(1,s);
        dy = REF_PATH_A_1_1[1].deriv(1,s);
        point.theta = atan2(dy, dx);
        traj_A1_1.push_back(point);
    }
    /** obs2 **/
    for (int i=0; i<50; i++){
        s = initi_obs_v*0.1*i;
        point.x = REF_PATH_A_0_2[0](s);
        point.y = REF_PATH_A_0_2[1](s);
        dx = REF_PATH_A_0_2[0].deriv(1,s);
        dy = REF_PATH_A_0_2[1].deriv(1,s);
        point.theta = atan2(dy, dx);
        traj_A0_2.push_back(point);
    }

    for (int i=0; i<50; i++){
        s = initi_obs_v*0.1*i;
        point.x = REF_PATH_A_1_2[0](s);
        point.y = REF_PATH_A_1_2[1](s);
        dx = REF_PATH_A_1_2[0].deriv(1,s);
        dy = REF_PATH_A_1_2[1].deriv(1,s);
        point.theta = atan2(dy, dx);
        traj_A1_2.push_back(point);
    }
    /** obs3 **/
    for (int i=0; i<50; i++){
        s = initi_obs_v*0.1*i;
        point.x = REF_PATH_A_0_3[0](s);
        point.y = REF_PATH_A_0_3[1](s);
        dx = REF_PATH_A_0_3[0].deriv(1,s);
        dy = REF_PATH_A_0_3[1].deriv(1,s);
        point.theta = atan2(dy, dx);
        traj_A0_3.push_back(point);
    }

    for (int i=0; i<50; i++){
        s = initi_obs_v*0.1*i;
        point.x = REF_PATH_A_1_3[0](s);
        point.y = REF_PATH_A_1_3[1](s);
        dx = REF_PATH_A_1_3[0].deriv(1,s);
        dy = REF_PATH_A_1_3[1].deriv(1,s);
        point.theta = atan2(dy, dx);
        traj_A1_3.push_back(point);
    }


	ROS_INFO_STREAM("Waiting for /solver_is_on");
	ros::service::waitForService("/solver_is_on", -1);
	ROS_INFO_STREAM("Waiting for /solver_is_on2");
	ros::service::waitForService("/solver_is_on2", -1);

	ros::ServiceServer service;
	service = n.advertiseService("mpc_is_on", &HelloFromMPC); //allows to trigger the MPC because initialization takes an hug and random time

    vector< vector<driving_simulator_msgs::Waypoint> > est_traj_A_0(4), est_traj_A_1(4);
    vector< vector<double> > belief_all(4);
    double solvetime = 0;
	ROS_INFO_STREAM("MPC Started");
	geometry_msgs::Pose pose;
    while (count <= 220)
    {
        //listening on traj_A_0, traj_A_1, beliefs
        //est_traj_A_0.clear(); est_traj_A_1.clear(); belief_all.clear();
        est_traj_A_0[0] = traj_A0;
        est_traj_A_0[1] = traj_A0_1;
        est_traj_A_0[2] = traj_A0_2;
        est_traj_A_0[3] = traj_A0_3;

        est_traj_A_1[0] = traj_A1;
        est_traj_A_1[1] = traj_A1_1;
        est_traj_A_1[2] = traj_A1_2;
        est_traj_A_1[3] = traj_A1_3;

        belief_all[0] = belief;
        belief_all[1] = belief_1;
        belief_all[2] = belief_2;
        belief_all[3] = belief_3;



    /**
    * Motion planning for ego-vehicle using MPC
    **/
        MPC_Planner.MPCUpdateParams(state_R, est_traj_A_0, est_traj_A_1, belief_all);



        MPC_Planner.MPCSolver(state_R, solvetime); // update state_R and compute the future policy


        /**
        * Update Rviz
        **/
        Ego_veh.header.stamp = ros::Time::now();

        Ego_veh.pose.position.x = state_R[0];
        Ego_veh.pose.position.y = state_R[1];
        Ego_veh.pose.position.z = 0;
        Ego_veh.pose.orientation.x = 0.0;
        Ego_veh.pose.orientation.y = 0.0;
        Ego_veh.pose.orientation.z = sin(state_R[2]/2);
        Ego_veh.pose.orientation.w = cos(state_R[2]/2);
		pose.position.x = state_R[0];
		pose.position.y = state_R[1];
		pose.position.z = 0;
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = sin(state_R[2]/2);
		pose.orientation.w = cos(state_R[2]/2);
		cmd_pub.publish(pose);
        veh_pub.publish(Ego_veh);


        /** record data**/

        //outfile << solvetime << endl;

        count++;
        ros::spinOnce();
        r.sleep();
    }
    //outfile.close();
  return 0;
}
