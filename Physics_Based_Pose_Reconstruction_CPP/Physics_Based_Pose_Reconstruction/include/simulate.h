#pragma once
#include <callbacks.h>
#include <utilities.h>


class simulation {
private:
	//mujoco data structures
	mjModel* m;	//pointer to mjModel
	mjData* d;	//pointer to mjData
	mjvCamera cam;		//abstract camera
	mjvOption opt;		//visualization options
	mjvScene scn;		//abstract scene
	mjrContext con;		//custom GPU context

public:
	utilities ::jnt_info info;
	const char* model_path;	//path to the mujoco xml model
	simulation(const char* model_path);
	void simulate();
	bool add_actuator_value(const char* name, const double value = 0);
private:
	utilities::jnt_info get_joint_pos();

};
