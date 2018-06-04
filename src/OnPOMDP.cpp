

#include <driving_simulator/OnPOMDP.h>
#include <algorithm>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <fstream>

const double PI = 3.1415926;

namespace despot{
POMDP::POMDP(){

	if (!n_.getParam(ros::this_node::getName()+"/Length", Length))
	{
		ROS_ERROR("Parameter 'Length' not set");
		return;
	}

	if (!n_.getParam(ros::this_node::getName()+"/Width", Width))
	{
		ROS_ERROR("Parameter 'Width' not set");
		return;
	}

	if (!n_.getParam(ros::this_node::getName()+"/obs_veh_num", obs_veh_num))
	{
		ROS_ERROR("Parameter 'obs_veh_num' not set");
		return;
	}

	if (!n_.getParam(ros::this_node::getName()+"/radius_disks", radius_disks))
	{
		ROS_ERROR("Parameter 'radius_disks' not set");
		return;
	}

	if (!n_.getParam(ros::this_node::getName()+"/N_SPLINE_POINTS", N_SPLINE_POINTS))
	{
		ROS_ERROR("Parameter 'N_SPLINE_POINTS' not set");
		return;
	}

}


vector<tk::spline> POMDP::Ref_path(vector<double> x, vector<double> y, vector<double> theta) {

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



void POMDP::InitializeDefaultParameters(){
    Globals::config.num_scenarios = 100;
    Globals::config.time_per_move = 1.0/3;
    Globals::config.search_depth = 10;
}

DSPOMDP* POMDP::InitializeModel(option::Option* options){
	POMDP_Plan* model  = new POMDP_Plan();

	initi_ego_x = model->initi_ego_x;
	initi_ego_y = model->initi_ego_y;
	initi_ego_v = model->initi_ego_v;
	initi_obs_x = model->initi_obs_x;
	initi_obs_y = model->initi_obs_y;
	initi_obs_v = model->initi_obs_v;
	n_points_spline = model->n_points_spline;

    return model;
}

void POMDP::OptionParse(option::Option* options, int& num_runs,
                   std::string& simulator_type, std::string& belief_type, int& time_limit,
                   std::string& solver_type, bool& search_solver){
  if (options[E_SILENCE])
    Globals::config.silence = true;

  if (options[E_DEPTH])
    Globals::config.search_depth = atoi(options[E_DEPTH].arg);

  if (options[E_DISCOUNT])
    Globals::config.discount = atof(options[E_DISCOUNT].arg);

  if (options[E_SEED])
    Globals::config.root_seed = atoi(options[E_SEED].arg);
  else { // last 9 digits of current time in milli second
    long millis = (long)get_time_second() * 1000;
    long range = (long)pow((double)10, (int)9);
    Globals::config.root_seed =
        (unsigned int)(millis - (millis / range) * range);
  }

  if (options[E_TIMEOUT])
    Globals::config.time_per_move = atof(options[E_TIMEOUT].arg);

  if (options[E_NUMPARTICLES])
    Globals::config.num_scenarios = atoi(options[E_NUMPARTICLES].arg);

  if (options[E_PRUNE])
    Globals::config.pruning_constant = atof(options[E_PRUNE].arg);

  if (options[E_GAP])
    Globals::config.xi = atof(options[E_GAP].arg);

  if (options[E_SIM_LEN])
    Globals::config.sim_len = atoi(options[E_SIM_LEN].arg);

  if (options[E_EVALUATOR])
    simulator_type = options[E_EVALUATOR].arg;

  if (options[E_MAX_POLICY_SIM_LEN])
    Globals::config.max_policy_sim_len =
        atoi(options[E_MAX_POLICY_SIM_LEN].arg);

  if (options[E_DEFAULT_ACTION])
    Globals::config.default_action = options[E_DEFAULT_ACTION].arg;

  if (options[E_RUNS])
    num_runs = atoi(options[E_RUNS].arg);

  if (options[E_BELIEF])
    belief_type = options[E_BELIEF].arg;

  if (options[E_TIME_LIMIT])
    time_limit = atoi(options[E_TIME_LIMIT].arg);

  if (options[E_NOISE])
    Globals::config.noise = atof(options[E_NOISE].arg);

  search_solver = options[E_SEARCH_SOLVER];

  if (options[E_SOLVER])
    solver_type = options[E_SOLVER].arg;

  int verbosity = 0;
  if (options[E_VERBOSITY])
    verbosity = atoi(options[E_VERBOSITY].arg);
  logging::level(verbosity);

}


Solver* POMDP::InitializeSolver(DSPOMDP *model, string solver_type,
                                    option::Option *options) {

  // DESPOT or its default policy
  if (solver_type == "DESPOT" ||
      solver_type == "PLB") // PLB: particle lower bound
  {
    string blbtype = options[E_BLBTYPE] ? options[E_BLBTYPE].arg : "DEFAULT";
    string lbtype = options[E_LBTYPE] ? options[E_LBTYPE].arg : "DEFAULT";
	  ROS_INFO_STREAM("POMDP::InitializeSolver: ScenarioLowerBound");
    ScenarioLowerBound *lower_bound =
        model->CreateScenarioLowerBound(lbtype, blbtype);

    logi << "Created lower bound " << typeid(*lower_bound).name() << endl;

    if (solver_type == "DESPOT") {
      string bubtype = options[E_BUBTYPE] ? options[E_BUBTYPE].arg : "DEFAULT";
      string ubtype = options[E_UBTYPE] ? options[E_UBTYPE].arg : "DEFAULT";
		ROS_INFO_STREAM("POMDP::InitializeSolver: ScenarioUpperBound");
      ScenarioUpperBound *upper_bound =
          model->CreateScenarioUpperBound(ubtype, bubtype);
		//here it initializes again the plan and it takes too long...
      logi << "Created upper bound " << typeid(*upper_bound).name() << endl;
		ROS_INFO_STREAM("POMDP::InitializeSolver: New DESPOT...");
      solver = new DESPOT(model, lower_bound, upper_bound);
    } else
      solver = lower_bound;
  } // AEMS or its default policy
  else if (solver_type == "AEMS" || solver_type == "BLB") {
    string lbtype = options[E_LBTYPE] ? options[E_LBTYPE].arg : "DEFAULT";
    BeliefLowerBound *lower_bound =
        static_cast<BeliefMDP *>(model)->CreateBeliefLowerBound(lbtype);

    logi << "Created lower bound " << typeid(*lower_bound).name() << endl;

    if (solver_type == "AEMS") {
      string ubtype = options[E_UBTYPE] ? options[E_UBTYPE].arg : "DEFAULT";
      BeliefUpperBound *upper_bound =
          static_cast<BeliefMDP *>(model)->CreateBeliefUpperBound(ubtype);

      logi << "Created upper bound " << typeid(*upper_bound).name() << endl;

      solver = new AEMS(model, lower_bound, upper_bound);
    } else
      solver = lower_bound;
  } // POMCP or DPOMCP
  else if (solver_type == "POMCP" || solver_type == "DPOMCP") {
    string ptype = options[E_PRIOR] ? options[E_PRIOR].arg : "DEFAULT";
    POMCPPrior *prior = model->CreatePOMCPPrior(ptype);

    logi << "Created POMCP prior " << typeid(*prior).name() << endl;

    if (options[E_PRUNE]) {
      prior->exploration_constant(Globals::config.pruning_constant);
    }

    if (solver_type == "POMCP")
      solver = new POMCP(model, prior);
    else
      solver = new DPOMCP(model, prior);
  } else { // Unsupported solver
    cerr << "ERROR: Unsupported solver type: " << solver_type << endl;
    exit(1);
  }
  return solver;
}

void POMDP::DisplayParameters(option::Option *options, DSPOMDP *model) {

  string lbtype = options[E_LBTYPE] ? options[E_LBTYPE].arg : "DEFAULT";
  string ubtype = options[E_UBTYPE] ? options[E_UBTYPE].arg : "DEFAULT";
  default_out << "Model = " << typeid(*model).name() << endl
              << "Random root seed = " << Globals::config.root_seed << endl
              << "Search depth = " << Globals::config.search_depth << endl
              << "Discount = " << Globals::config.discount << endl
              << "driving_simulator steps = " << Globals::config.sim_len << endl
              << "Number of scenarios = " << Globals::config.num_scenarios
              << endl
              << "Search time per step = " << Globals::config.time_per_move
              << endl
              << "Regularization constant = "
              << Globals::config.pruning_constant << endl
              << "Lower bound = " << lbtype << endl
              << "Upper bound = " << ubtype << endl
              << "Policy simulation depth = "
              << Globals::config.max_policy_sim_len << endl
              << "Target gap ratio = " << Globals::config.xi << endl;
  // << "Solver = " << typeid(*solver).name() << endl << endl;
}

void POMDP::Initialization(int argc, char* argv[]){
  clock_t main_clock_start = clock();
  EvalLog::curr_inst_start_time = get_time_second();

  const char *program = (argc > 0) ? argv[0] : "despot";

  argc -= (argc > 0);
  argv += (argc > 0); // skip program name argv[0] if present

  option::Stats stats(usage, argc, argv);
  option::Option *options = new option::Option[stats.options_max];
  option::Option *buffer = new option::Option[stats.buffer_max];
  option::Parser parse(usage, argc, argv, options, buffer);

  string solver_type = "DESPOT";
  bool search_solver;

  /* =========================
   * Parse required parameters
   * =========================*/
  int num_runs = 1;
  string simulator_type = "pomdp";
  string belief_type = "DEFAULT";
  int time_limit = -1;

  /* =========================================
   * Problem specific default parameter values
*=========================================*/
  ROS_WARN_STREAM("POMDP InitializeDefaultParameters");
  InitializeDefaultParameters();

  /* =========================
   * Parse optional parameters
   * =========================*/
  if (options[E_HELP]) {
    cout << "Usage: " << program << " [options]" << endl;
    option::printUsage(std::cout, usage);
  }
  OptionParse(options, num_runs, simulator_type, belief_type, time_limit,
              solver_type, search_solver);

  /* =========================
   * Global random generator
   * =========================*/
  Seeds::root_seed(Globals::config.root_seed);
  unsigned world_seed = Seeds::Next();
  unsigned seed = Seeds::Next();
  Random::RANDOM = Random(seed);

  /* =========================
   * initialize model
   * =========================*/
	ROS_WARN_STREAM("POMDP InitializeModel");
  model = InitializeModel(options);

   /* initialize solver
   * =========================*/
	ROS_WARN_STREAM("POMDP InitializeSolver");
  solver = InitializeSolver(model, solver_type, options);
  assert(solver != NULL);


    // Initial state
	State* state = model->CreateStartState();

   /* =========================
      Initial belief
      =========================*/

	double start_t = get_time_second();
	delete solver->belief();
	double end_t = get_time_second();

	Belief* belief = model->InitialBelief(state, belief_type);

	solver->belief(belief);


	/*=========================
	   Reference path for obstacle vehicle
	   ========================*/
	if (!n_.getParam(ros::this_node::getName()+"/x_A_0", x_A_0))
	{
		ROS_ERROR("Parameter 'x_A_0' not set");
		return;
	}
	if (!n_.getParam(ros::this_node::getName()+"/y_A_0", y_A_0))
	{
		ROS_ERROR("Parameter 'y_A_0' not set");
		return;
	}

	if (!n_.getParam(ros::this_node::getName()+"/theta_A_0", theta_A_0))
	{
		ROS_ERROR("Parameter 'theta_A_0' not set");
		return;
	}

	if (!n_.getParam(ros::this_node::getName()+"/x_A_1", x_A_1))
	{
		ROS_ERROR("Parameter 'x_A_1' not set");
		return;
	}

	if (!n_.getParam(ros::this_node::getName()+"/y_A_1", y_A_1))
	{
		ROS_ERROR("Parameter 'y_A_1' not set");
		return;
	}

	if (!n_.getParam(ros::this_node::getName()+"/theta_A_1", theta_A_1))
	{
		ROS_ERROR("Parameter 'theta_A_1' not set");
		return;
	}

	REF_PATH_A_0 = Ref_path(x_A_0, y_A_0, theta_A_0);
	//cout << REF_PATH_A_0.size() << endl;
	REF_PATH_A_1 = Ref_path(x_A_1, y_A_1, theta_A_1);

	ROS_INFO("POMDP::Initialization: POMDP Initialized");
	/*===============================
	Initialization of Line Marker
	=================================*/
    line1.type = visualization_msgs::Marker::LINE_STRIP;
    line1.id = 3;
    line1.scale.x = 0.1;
    //line1.color.g = 1.0f;
    line1.color.b = 0.6f;
    line1.color.a = 1.0;
    line1.header.frame_id = "/my_frame";
    line1.ns = "trajectory";
    line1.action = visualization_msgs::Marker::ADD;
    line1.lifetime = ros::Duration(0.4);

    line2.type = visualization_msgs::Marker::LINE_STRIP;
    line2.id = 4;
    line2.scale.x = 0.1;
    //line2.color.g = 1.0f;
    line2.color.b = 0.6f;
    line2.color.a = 1.0;
    line2.header.frame_id = "/my_frame";
    line2.ns = "trajectory";
    line2.action = visualization_msgs::Marker::ADD;
    line2.lifetime = ros::Duration(0.4);

    hist1.type = visualization_msgs::Marker::ARROW;
    hist1.id = 6;
    hist1.scale.y = 0.5;
    hist1.scale.z = 1;
    hist1.color.g = 1.0f;
    hist1.color.a = 1.0;
    hist1.pose.orientation.x = 0;
    hist1.pose.orientation.y = 0;
    hist1.pose.orientation.z = sin(PI/2);
    hist1.pose.orientation.w = cos(PI/2);
    hist1.header.frame_id = "/my_frame";
    hist1.ns = "trajectory";
    hist1.action = visualization_msgs::Marker::ADD;
    hist1.lifetime = ros::Duration(0.4);

    hist2.type = visualization_msgs::Marker::ARROW;
    hist2.id = 7;
    hist2.scale.y = 0.5;
    hist2.scale.z = 1;
    hist2.color.g = 1.0f;
    hist2.color.a = 1.0;
    hist2.pose.orientation.x = 0;
    hist2.pose.orientation.y = 0;
    hist2.pose.orientation.z = sin(-PI/4);
    hist2.pose.orientation.w = cos(-PI/4);
    hist2.header.frame_id = "/my_frame";
    hist2.ns = "trajectory";
    hist2.action = visualization_msgs::Marker::ADD;
    hist2.lifetime = ros::Duration(0.4);

    hist3.type = visualization_msgs::Marker::CYLINDER;
    hist3.id = 8;
    hist3.color.r = 1.0f;
    hist3.color.a = 1.0;
    hist3.header.frame_id = "/my_frame";
    hist3.ns = "trajectory";
    hist3.action = visualization_msgs::Marker::ADD;
    hist3.lifetime = ros::Duration(0.4);

    ellips1.type = visualization_msgs::Marker::CYLINDER;
    ellips1.id = 60;
    ellips1.color.b = 0.6f;
    ellips1.color.a = 1.0;
    ellips1.header.frame_id = "/my_frame";
    ellips1.ns = "trajectory";
    ellips1.action = visualization_msgs::Marker::ADD;
    ellips1.lifetime = ros::Duration(0.4);

    ellips2.type = visualization_msgs::Marker::CYLINDER;
    ellips2.id = 100;
    ellips2.color.b = 0.6f;
    ellips2.color.a = 1.0;
    ellips2.header.frame_id = "/my_frame";
    ellips2.ns = "trajectory";
    ellips2.action = visualization_msgs::Marker::ADD;
    ellips2.lifetime = ros::Duration(0.4);



}

void POMDP::ReconstructPolicy(vector<int> policyStar, vector<int> depthOrder, vector<int>& policy0, vector<int>& policy1){

    policy0.push_back(policyStar[0]);
    int i;
    for (i=1; i<depthOrder.size(); i++){
        if (depthOrder[i] > depthOrder[i-1]){
            policy0.push_back(policyStar[i]);
        }
        else
            break;
    }



    policy1 = policy0;
    for (;i < depthOrder.size(); i++){
        if (depthOrder[i] < policy1.size()){
            policy1[depthOrder[i]] = policyStar[i];
        }
        else{
            policy1.push_back(policyStar[i]);
            policy0.push_back(policyStar[i]);
        }
    }

    policy0.insert(policy0.end(), 50-policy0.size(), 2);
    policy1.insert(policy1.end(), 50-policy1.size(), 2);
}


vector<double> POMDP::Uncertainty(double time, double weight){
	double sigma_s0 = 0.01;
	double sigma_s_delta = 0.1 ;
	double sigma_s_max = 5;
	double sigma_d0 = 0.001;
	double sigma_d_delta = 0.05;
	double sigma_d_max = 0.9;
	double p_thresh = 0.01;
	vector<double> uncertainty(2);
	double a_uncertainty, b_uncertainty;

	double sigma_s = min(sigma_s0 + sigma_s_delta*time, sigma_s_max);
	double sigma_d = min(sigma_d0 + sigma_d_delta*time, sigma_d_max);



	if (1.0/(2*PI*sigma_s*sigma_d) >  p_thresh / weight ){
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




int POMDP::POMDP_Solver(vector<double> state_A, vector<driving_simulator_msgs::Waypoint> traj_R){
    // Update the policy of ego-vehicle
    model->traj_R = traj_R;

    int action = solver->Search().action;
    policyStar = solver->policyStar;
    depthOrder = solver->depthOrder;
    goal_probs = solver->goal_probs;
    model->goal_prob_ = goal_probs;

    beliefs.belief.clear();
    beliefs.belief = goal_probs;


    // Transition from optimal policy to state space
    vector<int> policy0, policy1;

    ReconstructPolicy(policyStar, depthOrder, policy0, policy1);

    vector<double> state(6), next_state, state_sim;

    state[0] = state_A[0];
    state[1] = state_A[1];
    state[2] = state_A[2];
    state[3] = state_A[3];
    state[5] = state_A[5];

    traj0.Traj.clear();
    traj1.Traj.clear();
    line1.points.clear();
    line2.points.clear();
    POMDP_Plan* pomdp_plan = static_cast<POMDP_Plan*>(model);

    vector<double> goal_probs_order = goal_probs;//ascending order
    std::sort(goal_probs_order.begin(), goal_probs_order.end());
    // only need to consider one policy
    if ((goal_probs_order[2]-goal_probs_order[1])/goal_probs_order[2] > 0.8){
        double index = std::distance(goal_probs.begin(), std::find(goal_probs.begin(), goal_probs.end(), goal_probs_order[2]));
        state[4] = index;
        state_sim = state;
        for (int i=0; i<policy0.size(); i++){
            geometry_msgs::Point p;
            driving_simulator_msgs::Waypoint point;
            next_state = pomdp_plan->Dynamics_A(state_sim, policy0[i]);
            point.x = next_state[0];
            point.y = next_state[1];
            point.theta = next_state[2];
            traj0.Traj.push_back(point);
            state_sim = next_state;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            line1.points.push_back(p);
        }
        line2.points = line1.points;
        traj1.Traj = traj0.Traj;
    }
    else{// should consider two policies
        state[4] = 0;
        state_sim = state;
        for (int i=0; i<policy0.size(); i++){
            geometry_msgs::Point p;
            driving_simulator_msgs::Waypoint point;
            next_state = pomdp_plan->Dynamics_A(state_sim, policy0[i]);
            point.x = next_state[0];
            point.y = next_state[1];
            point.theta = next_state[2];
            traj0.Traj.push_back(point);
            state_sim = next_state;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            line1.points.push_back(p);
        }

        next_state.clear(); state_sim.clear();
        state[4] = 1;
        state_sim = state;
        for (int i=0; i<policy1.size(); i++){
            geometry_msgs::Point p;
            driving_simulator_msgs::Waypoint point;
            next_state = pomdp_plan->Dynamics_A(state_sim, policy1[i]);
            point.x = next_state[0];
            point.y = next_state[1];
            point.theta = next_state[2];
            traj1.Traj.push_back(point);
            state_sim = next_state;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            line2.points.push_back(p);

        }
    }




    hist1.header.stamp = ros::Time::now();
    hist1.pose.position.x = state_A[0] -2;
    hist1.pose.position.y = state_A[1] +2;
    hist1.pose.position.z = 0;

    hist1.scale.x = goal_probs[0] *3;

    hist2.header.stamp = ros::Time::now();
    hist2.pose.position.x = state_A[0] -2;
    hist2.pose.position.y = state_A[1] +5;
    hist2.pose.position.z = 0;

    hist2.scale.x = goal_probs[1] *3;

    hist3.header.stamp = ros::Time::now();
    hist3.scale.x = goal_probs[2] * 1;
    hist3.scale.y = goal_probs[2] * 1;
    hist3.scale.z = 0.5;

    hist3.pose.position.x = state_A[0] -2;
    hist3.pose.position.y = state_A[1] +6;
    hist3.pose.position.z = 0;

    line_pub.publish(hist1);
    line_pub.publish(hist2);
    line_pub.publish(hist3);


    trajA0_pub.publish(traj0);
    trajA1_pub.publish(traj1);
    belief_pub.publish(beliefs);

    line1.header.stamp = ros::Time::now();
    line2.header.stamp = ros::Time::now();
    line_pub.publish(line1);
    line_pub.publish(line2);

    state_A = pomdp_plan->Dynamics_A(state_A, 2);


      /*** Plot uncertainty ellipses ***/
    vector<double> dt(50);
    for (int i=0; i<50; i++){
		if (i<40)
			dt[i] = 0.1;
		else
			dt[i] = 0.2;
	}

    vector<double> time(50);
	time[0] = 0;
	for (int i=1; i<50; i++){
		time[i] = time[i-1] + dt[i-1];
	}

    for (int i=0; i < traj0.Traj.size(); i=i+9){
        ellips1.id = 60+i;
        ellips1.pose.position.x = traj0.Traj[i].x;
        ellips1.pose.position.y = traj0.Traj[i].y;
        ellips1.pose.orientation.x = 0;
        ellips1.pose.orientation.y = 0;
        ellips1.pose.orientation.z = sin(traj0.Traj[i].theta /2);
        ellips1.pose.orientation.w = cos(traj0.Traj[i].theta /2);

        vector<double> unc = Uncertainty(time[i], goal_probs[0]);
        ellips1.scale.x = unc[0] + goal_probs[0]*(sqrt(2)*2.0+sqrt(5)/2.0);
        ellips1.scale.y = unc[1] + goal_probs[0]*(sqrt(2)+sqrt(5)/2.0);
        ellips1.scale.z = 0.5;
        ellips1.color.a = 0.8 - i / 70.0;
        line_pub.publish(ellips1);
    }

    for (int i=0; i < traj1.Traj.size(); i=i+9){
        ellips2.id = 100+i;
        ellips2.pose.position.x = traj1.Traj[i].x;
        ellips2.pose.position.y = traj1.Traj[i].y;
        ellips2.pose.orientation.x = 0;
        ellips2.pose.orientation.y = 0;
        ellips2.pose.orientation.z = sin(traj1.Traj[i].theta/2);
        ellips2.pose.orientation.w = cos(traj1.Traj[i].theta/2);


        vector<double> unc = Uncertainty(time[i], goal_probs[1]);
        ellips2.scale.x = unc[0] + goal_probs[1]*(sqrt(2)*2.0+sqrt(5)/2.0);
        ellips2.scale.y = unc[1] + goal_probs[1]*(sqrt(2)+sqrt(5)/2.0);
        ellips2.scale.z = 0.5;
        ellips2.color.a = 0.8 - i / 70.0;
        line_pub.publish(ellips2);
    }


    return action;

}

void POMDP::POMDP_Update(int action, vector<double> state_R, vector<double> state_A){
    POMDP_Plan_State next_state(state_R,state_A);
    POMDP_Plan* pomdp_plan = static_cast<POMDP_Plan*>(model);
  // cout << " observation: " << state_R[0] << " " << state_R[1] << " " << state_A[0] << " " << state_A[1] << endl;
    int observation = pomdp_plan->MakeObservation(next_state);

    solver->Update(action, observation);


}


}