#pragma once
#include <simulate.h>
#include <thread>

using namespace std;


simulation::simulation(const char* model_path) {
	char err[500] = "ERROR: Could not load the XML model!";
	//Load the XML model and make the mjData
	m = mj_loadXML(model_path, 0, err, 500);
	d = mj_makeData(m);
	this->model_path = model_path;
}

void simulation::simulate() {

	if (!glfwInit())
		mju_error("GLFW not initialized");

	GLFWwindow* window;
	char title[100] = "My Window";
	window = glfwCreateWindow(1280, 720, title, NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

	//Make the Scene and the Context for rendering
	mjv_makeScene(m, &scn, 500);
	mjr_makeContext(m, &con, 150);

	//set input event callbacks
	glfwSetKeyCallback(window, keyboard_callback);
	glfwSetCursorPosCallback(window, mouse_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	while (!glfwWindowShouldClose(window)) {

		mjtNum sim_start = d->time;
		while (d->time - sim_start < 1.0 / 60.0) {
			mj_step(m, d);
			this->info = this->get_joint_pos();
		}
		
		//Create a rectangula viewport and render the model
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

	// free visualization storage
	mjv_freeScene(&scn);
	mjr_freeContext(&con);

	// free MuJoCo model and data, deactivate
	mj_deleteData(d);
	mj_deleteModel(m);

	glfwTerminate();
}

utilities::jnt_info simulation::get_joint_pos()
{
	info = utilities::get_joint_pos(m, d);
	return info;
}

bool simulation::add_actuator_value(const char* name, const double value)
{
	return utilities::add_actuator_value(d, m, name, value);
}

