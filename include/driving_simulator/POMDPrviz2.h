/*
 * POMDP_Plan.h
 *
 *  Created on: Apr 5, 2017
 *      Author: bingyu
 */

#ifndef POMDP_H_
#define POMDP_H_

#include <despot/core/pomdp.h>
#include <despot/core/mdp.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <driving_simulator/spline.h>
#include <driving_simulator/Clothoid.h>

using namespace std;
namespace despot{
/*===========================================
 * POMDP_Plan State class
 *===========================================*/

class POMDP_Plan_State : public State {
public:
	std::vector<double> state_R; //x,y,theta
	std::vector<double> state_A; //x,y,theta,v,g,s

	POMDP_Plan_State();

	POMDP_Plan_State(std::vector<double> _state_R, std::vector<double> _state_A);

	~POMDP_Plan_State();

	std::string text() const;

};

/*===============================================
 * POMDP_Plan class
 ================================================*/



class POMDP_Plan : public DSPOMDP {
protected:
	mutable MemoryPool<POMDP_Plan_State> memory_pool_;
	std::vector<POMDP_Plan_State*> states_;
	void Init();
	std::vector<tk::spline> REF_PATH_A_0, REF_PATH_A_1;


public:
	enum{//action
		Acc = 0, Dec = 1, Cur = 2
	};
	map<vector<int>, int> obs_;
public:
	POMDP_Plan();

   // vector<double> goal_prob_;
	//Reference path
	vector<tk::spline> Ref_path(vector<double> x, vector<double> y, vector<double> theta) const;

	//Find the steer angle
	double FindSteer_A(vector<double>& state_A)const;
	double FindSteer_R(vector<double>& state_R)const;

	//Find the segmentation of reference path
	double FindSegmentation_A(std::vector<double>& state_A)const;
	double FindSegmentation_R(std::vector<double>& state_R)const;

	inline Eigen::Matrix2d Rotation(double theta)const;
	inline double Gausspdf(double d)const;

	// dynamics
	std::vector<double> Dynamics_A(std::vector<double> state_A_, int action_) const;
	std::vector<double> Dynamics_R(std::vector<double> state_R_)const;

	//Observation
	int MakeObservation(const POMDP_Plan_State _pomdp_state)const;

	//Total number of actions
	int NumActions() const;

	//Deterministic simulative model
	bool Step(State& state, double rand_num, int action, double& reward,
			OBS_TYPE& obs) const;

	//Beliefs (motion intention of obstacle vehs) and starting states
	double ObsProb(OBS_TYPE obs, const State& state, int action) const;
	Belief* InitialBelief(const State* start, string type = "PARTICLE") const;
	State* CreateStartState(string type = "DEFAULT") const;
	//Bound-related functions
	double GetMaxReward() const;
	//ParticleUpperBound* CreateParticleUpperBound(std::string name = "DEFAULT") const;
	ScenarioUpperBound* CreateScenarioUpperBound(string name = "DEFAULT",string particle_bound_name = "DEFAULT") const;
	ValuedAction GetMinRewardAction() const;
	ParticleLowerBound* CreateParticleLowerBound(std::string name = "DEFAULT") const;
	ScenarioLowerBound* CreateScenarioLowerBound(string name = "DEFAULT",string particle_bound_name = "DEFAULT") const;


	//Memory management
	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	//Display
	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::vector<double>& goal_probs, std::ostream& out = std::cout) const;
	void PrintObs(const State& state, OBS_TYPE observation,std::ostream& out = std::cout) const;
	void PrintAction(int action, std::ostream& out = std::cout) const;







};
}


#endif /* POMDP_H_ */
