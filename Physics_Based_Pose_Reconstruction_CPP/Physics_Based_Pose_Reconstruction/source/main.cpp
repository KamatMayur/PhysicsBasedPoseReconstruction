#pragma once
#include <callbacks.h>
#include <shared_data.h>


using namespace std;

char model_path[100] = "Mujoco_Models/Humanoid/Ybot.xml";

//mujoco data structures
mjModel* m = NULL;	//pointer to mjModel
mjData* d = NULL;	//pointer to mjData
mjvCamera cam;		//abstract camera
mjvOption opt;		//visualization options
mjvScene scn;		//abstract scene
mjrContext con;		//custom GPU context

SOCKET client_socket;
SOCKET server_socket;

int main() {
	//start the server and client
	if (init_sockets()) cout << "Server and Client initalized /n";
	else return 0;

	char err[500] = "ERROR: Could not load the XML model!";
	//Load the XML model and make the mjData
	m = mj_loadXML(model_path, 0, err, 500);
	d = mj_makeData(m);

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
			updateSharedData(d, m, cam, opt, scn, con);
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

	closesocket(client_socket);
	closesocket(server_socket);
	WSACleanup();

	glfwTerminate();
	return 1;

}

