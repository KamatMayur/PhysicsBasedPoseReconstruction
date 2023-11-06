#pragma once
#include <simulate.h>

using namespace std;

simulation::simulation(const char* model_path) {
	// Load the XML model
	this->model_path = model_path;
	//initialize model and data
	char err[500] = "ERROR: Could not load the XML model!";
	m = mj_loadXML(model_path, 0, err, 500);
	d = mj_makeData(m);
}

simulation::~simulation()
{
	this->deinitialize();
}

void simulation::simulate() {
	/*
	* simulate continuously till the window is closed at 60 steps / sec
	*/
	if (!glfwInit())
		mju_error("GLFW not initialized");

	GLFWwindow* window;
	char title[100] = "My Window";
	window = glfwCreateWindow(1280, 720, title, NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	this->initialize();

	//set input event callbacks
	glfwSetKeyCallback(window, keyboard_callback);
	glfwSetCursorPosCallback(window, mouse_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	while (!glfwWindowShouldClose(window)) {

		mjtNum sim_start = d->time;
		while (d->time - sim_start < 1.0 / 25) {
			mj_step(m, d);
			this->info = this->get_joint_info();
		}
		
		//Create a rectangular viewport and render the model
		mjrRect viewport = { 0, 0, 0, 0 };
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		glfwSwapBuffers(window);

		// process pending GUI events, call GLFW callbacks
		glfwPollEvents();

		// Handle simultaneous key presses
		handle_multikey_press();
	}
	glfwTerminate();
}

utilities::jnt_info simulation::get_joint_info()
{
	info = utilities::get_joint_info(m, d);
	return info;
}

bool simulation::add_actuator_value(const char* name, const double value)
{
	return utilities::add_actuator_value(d, m, name, value);
}

void simulation::controls(py::array_t<double> arr)
{
	auto r = arr.unchecked<1>(); // 1D read-only array
	int n = arr.size();
	if (n == (int)this->info.act_num)
	{
		// Set the ctrl field in mjData
		for (int i = 0; i < n; i++)
		{
			this->d->ctrl[i] = r(i);
		}
		return;
	}

	else cerr << "ERROR: the size of the list doesn't match the number of actuators, model has " << this->info.act_num << " actuators";
	
}


void simulation::initialize()
{
	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

	//Make the Scene and the Context for rendering
	mjv_makeScene(m, &scn, 500);
	mjr_makeContext(m, &con, 150);
	return;
}

void simulation::deinitialize()
{
	// free visualization storage
	mjv_freeScene(&scn);
	mjr_freeContext(&con);

	// free MuJoCo model and data, deactivate
	mj_deleteData(d);
	mj_deleteModel(m);
	return;
}


void simulation::simulate_from_state(bool visualize, double sim_time, double step_time)
{
	/*
	* simulate for the specified time with specified step time 
	 */
	if (visualize)
	{
		if (!glfwInit())
			mju_error("GLFW not initialized");
		char title[100] = "My Window";
		GLFWwindow* window = glfwCreateWindow(1280, 720, title, NULL, NULL);
		glfwMakeContextCurrent(window);
		//set input event callbacks

		glfwSetKeyCallback(window, keyboard_callback);
		glfwSetCursorPosCallback(window, mouse_position_callback);
		glfwSetMouseButtonCallback(window, mouse_button_callback);
		glfwSwapInterval(1);

		this->initialize();

		long int prevTime = (long int)this->d->time;
		long int currTime;
		while ((currTime = (long int)this->d->time) - prevTime < sim_time) {

			mjtNum sim_start = this->d->time;
			while (this->d->time - sim_start < step_time) {
				mj_step(this->m, this->d);
			}
			this->get_joint_info();
			
			//Create a rectangular viewport and render the model
			mjrRect viewport = { 0, 0, 0, 0 };
			glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
			mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
			mjr_render(viewport, &scn, &con);
			glfwSwapBuffers(window);

			// process pending GUI events, call GLFW callbacks
			glfwPollEvents();

			// Handle simultaneous key presses
			handle_multikey_press();
	
		}
		glfwTerminate();
	}

	// no visualization
	else
	{
		long int prevTime = (long int)this->d->time;
		long int currTime;
		while ((currTime = (long int)this->d->time) - prevTime < sim_time) {

			mjtNum sim_start = this->d->time;
			while (this->d->time - sim_start < step_time) {
				mj_step(this->m, this->d);
			}
			this->get_joint_info();
		}
	}
	return;
	
}

// Save the mjData to a file
bool simulation::save_data(const char* data_path)
{
	std::ofstream outFile(data_path, std::ios::binary);
	if (outFile.is_open()) {
		outFile.write(reinterpret_cast<const char*>(d->qpos), m->nq * sizeof(double));
		outFile.write(reinterpret_cast<const char*>(d->qvel), m->nv * sizeof(double));
		outFile.write(reinterpret_cast<const char*>(&d->time), sizeof(double));
		outFile.close();
		std::cout << "Data saved successfully saved at : "<< data_path << std::endl;
		return true;
	}
	else {
		std::cerr << "Unable to save the data to the file." << std::endl;
		return false;
	}
}

// load the saved data
bool simulation::load_data(const char* data_path)
{
	std::ifstream inFile(data_path, std::ios::binary);
	if (inFile.is_open()) {
		inFile.read(reinterpret_cast<char*>(d->qpos), m->nq * sizeof(double));
		inFile.read(reinterpret_cast<char*>(d->qvel), m->nv * sizeof(double));
		inFile.read(reinterpret_cast<char*>(&d->time), sizeof(double));
		std::cout << "Data loaded successfully." << std::endl;
		return true;
	}
	else {
		std::cerr << "Unable to open the saved data file." << std::endl;
		return false;
	}
}

