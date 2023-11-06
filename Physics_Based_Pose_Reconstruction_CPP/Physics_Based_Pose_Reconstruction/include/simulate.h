#pragma once
#include <callbacks.h>
#include <utilities.h>
#include <fstream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

class simulation {
private:
	//mujoco data structures
	mjModel* m;		//pointer to mjModel
	mjData* d;		//pointer to mjData
	mjvCamera cam;		//abstract camera
	mjvOption opt;		//visualization options
	mjvScene scn;		//abstract scene
	mjrContext con;		//custom GPU context
	utilities::jnt_info get_joint_info();		//stores joints data
	void initialize();
	void deinitialize();
	
public:
	const char* model_path;	//path to the mujoco xml model
	utilities ::jnt_info info;
	simulation(const char* model_path);
	~simulation();
	void simulate();
	bool add_actuator_value(const char* name, const double value = 0);
	void controls(py::array_t<double> arr);
	void simulate_from_state(bool visualize, double sim_time, double step_time);
	bool save_data(const char* data_path);
	bool load_data(const char* data_path);


};
