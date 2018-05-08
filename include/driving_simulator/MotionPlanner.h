#ifndef MPMPC_H_
#define MPMPC_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "FORCESNLPsolver.h"
#include "/home/bdebrito/code/ProbabilisticMotionPlanning-MasterThesis/POMDP/POMDP/src/POMDPrviz.h"
#include "/home/bdebrito/code/ProbabilisticMotionPlanning-MasterThesis/POMDP/POMDP/src/spline.h"

#include <vector>
#include <cmath>

#include <typeinfo>
#include <despot/solver/despot.h>
#include <despot/solver/aems.h>
#include <despot/solver/pomcp.h>

#include <despot/util/optionparser.h>
#include <despot/util/seeds.h>

#include <despot/core/pomdp.h>
#include <despot/ippc/client.h>

#include <despot/evaluator.h>


class Road {

public:
    vector<tk::spline> Ref_path(vector<double> x, vector<double> y, vector<double> theta, double& dist_spline_pts );
};


class MPMPC : public Road{
private:


	FORCESNLPsolver_params mpcparams;
	FORCESNLPsolver_output mpcoutput;
	FORCESNLPsolver_info mpcinfo;
	FORCESNLPsolver_ExtFunc pt2Function = &FORCESNLPsolver_casadi2forces;

	bool Flag = false;

	double time[50], dt[50];


public:
	MPMPC();
    vector<tk::spline> ref_path_R;
    double dist_spline_pts;

	void MPCSolver(vector<double>& state, despot::Traj& traj_R);

	void MPCUpdateParams(vector<double> state_R_, vector<despot::Traj> est_traj_A_, vector<double> beliefs);

	vector<double> Uncertainty(double time, double weight);

	inline vector<double> Rotation(double angle){
		vector<double> rotation(4);
		rotation[0] = cos(angle);
		rotation[1] = sin(angle);
		rotation[2] = -sin(angle);
		rotation[3] = cos(angle);
		return rotation;
	};



};

using namespace std;
namespace despot{
void disableBufferedIO(void);

enum OptionIndex {
  E_UNKNOWN,
  E_HELP,
  E_PARAMS_FILE,
  E_DEPTH,
  E_DISCOUNT,
  E_SIZE,
  E_NUMBER,
  E_SEED,
  E_TIMEOUT,
  E_NUMPARTICLES,
  E_PRUNE,
  E_GAP,
  E_SIM_LEN,
  E_EVALUATOR,
  E_MAX_POLICY_SIM_LEN,
  E_DEFAULT_ACTION,
  E_RUNS,
  E_BLBTYPE,
  E_LBTYPE,
  E_BUBTYPE,
  E_UBTYPE,
  E_BELIEF,
  E_KNOWLEDGE,
  E_VERBOSITY,
  E_SILENCE,
  E_SOLVER,
  E_TIME_LIMIT,
  E_NOISE,
  E_SEARCH_SOLVER,
  E_PRIOR,
  E_SERVER,
  E_PORT,
  E_LOG,
};

const option::Descriptor usage[] = {
  { E_HELP, 0, "", "help", option::Arg::None,
    "  \t--help\tPrint usage and exit." },
  { E_PARAMS_FILE, 0, "m", "model-params", option::Arg::Required,
    "-m <arg>  \t--model-params <arg>  \tPath to model-parameters file, if "
    "any." },
  { E_SIZE, 0, "", "size", option::Arg::Required,
    "  \t--size <arg>  \tSize of a problem (problem specific)." },
  { E_NUMBER, 0, "", "number", option::Arg::Required,
    "  \t--number <arg>  \tNumber of elements of a problem (problem "
    "specific)." },
  { E_DEPTH, 0, "d", "depth", option::Arg::Required,
    "-d <arg>  \t--depth <arg>  \tMaximum depth of search tree (default 90)." },
  { E_DISCOUNT, 0, "g", "discount", option::Arg::Required,
    "-g <arg>  \t--discount <arg>  \tDiscount factor (default 0.95)." },
  { E_TIMEOUT, 0, "t", "timeout", option::Arg::Required,
    "-t <arg>  \t--timeout <arg>  \tSearch time per move, in seconds (default "
    "1)." },
  { E_NUMPARTICLES, 0, "n", "nparticles", option::Arg::Required,
    "-n <arg>  \t--nparticles <arg>  \tNumber of particles (default 500)." },
  { E_PRUNE, 0, "p", "prune", option::Arg::Required,
    "-p <arg>  \t--prune <arg>  \tPruning constant (default no pruning)." },
  { E_GAP, 0, "", "xi", option::Arg::Required,
    "  \t--xi <arg>  \tGap constant (default to 0.95)." },
  { E_MAX_POLICY_SIM_LEN, 0, "", "max-policy-simlen", option::Arg::Required,
    "  \t--max-policy-simlen <arg>  \tDepth to simulate the default policy "
    "until. (default 90)." },

  { E_SEED, 0, "r", "seed", option::Arg::Required,
    "-r <arg>  \t--seed <arg>  \tRandom number seed (default is random)." },
  { E_SIM_LEN, 0, "s", "simlen", option::Arg::Required,
    "-s <arg>  \t--simlen <arg>  \tNumber of steps to simulate. (default 90; 0 "
    "= infinite)." },
  { E_RUNS, 0, "", "runs", option::Arg::Required,
    "  \t--runs <arg>  \tNumber of runs. (default 1)." },
  // { E_EVALUATOR, 0, "", "evaluator", option::Arg::Required, "  \t--evaluator
  // <arg>  \tUse IPPC server or a POMDP model as the evaluator." },
  // { E_DEFAULT_ACTION, 0, "", "default-action", option::Arg::Required, "
  // \t--default-action <arg>  \tType of default action to use. (default none)."
  // },
  { E_LBTYPE, 0, "l", "lbtype", option::Arg::Required,
    "-l <arg>  \t--lbtype <arg>  \tLower bound strategy." },
  { E_BLBTYPE, 0, "", "blbtype", option::Arg::Required,
    "  \t--blbtype <arg>  \tBase lower bound." },
  { E_UBTYPE, 0, "u", "ubtype", option::Arg::Required,
    "-u <arg>  \t--ubtype <arg>  \tUpper bound strategy." },
  { E_BUBTYPE, 0, "", "bubtype", option::Arg::Required,
    "  \t--bubtype <arg>  \tBase upper bound." },

  { E_BELIEF, 0, "b", "belief", option::Arg::Required,
    "-b <arg>  \t--belief <arg>  \tBelief update strategy, if applicable." },
  { E_NOISE, 0, "", "noise", option::Arg::Required,
    "  \t--noise <arg>  \tNoise level for transition in POMDPX belief "
    "update." },

  { E_VERBOSITY, 0, "v", "verbosity", option::Arg::Required,
    "-v <arg>  \t--verbosity <arg>  \tVerbosity level." },
  { E_SILENCE, 0, "", "silence", option::Arg::None,
    "  \t--silence  \tReduce default output to minimal." },
  { E_SOLVER, 0, "", "solver", option::Arg::Required,
    "  \t--solver <arg>  \t" },
  // { E_TIME_LIMIT, 0, "", "time-limit", option::Arg::Required, "
  // \t--time-limit <arg>  \tTotal amount of time allowed for the program." },
  // { E_SEARCH_SOLVER, 0, "", "search-solver", option::Arg::None, "
  // \t--search-solver\tUse first few runs to select DESPOT or POMCP as the
  // solver for remaining runs." },
  { E_PRIOR, 0, "", "prior", option::Arg::Required,
    "  \t--prior <arg>  \tPOMCP prior." },
  // { E_SERVER, 0, "", "server", option::Arg::Required, "  \t--server <arg>
  // \tServer address." },
  // { E_PORT, 0, "", "port", option::Arg::Required, "  \t--port <arg>  \tPort
  // number." },
  // { E_LOG, 0, "", "log", option::Arg::Required, "  \t--log <arg>  \tIPPC log
  // file." },
  { 0, 0, 0, 0, 0, 0 }
};


class POMDP {
private:

	double v_A = 5.0;

protected:
    Solver* solver = NULL;
    DSPOMDP* model = NULL;
public:
    POMDP();
    vector<int> policyStar;
    vector<int> depthOrder;
    vector<double> goal_probs;

    DSPOMDP* InitializeModel(option::Option* options);
    void InitializeDefaultParameters();

    Solver* InitializeSolver(DSPOMDP* model, std::string solver_type,
                           option::Option* options);

    void Initialization(int argc, char* argv[]);

    void OptionParse(option::Option* options, int& num_runs,
                   std::string& simulator_type, std::string& belief_type, int& time_limit,
                   std::string& solver_type, bool& search_solver);



    void DisplayParameters(option::Option* options, DSPOMDP* model);

    int POMDP_Solver(vector<double>& state_A, Traj& traj0, Traj& traj1, Traj traj_R);

    void POMDP_Update(int action, vector<double> pos0, vector<double> pos1);
    void ReconstructPolicy(vector<int> policyStar, vector<int> depthOrder, vector<int>& policy0, vector<int>& policy1);

};

}



#endif
