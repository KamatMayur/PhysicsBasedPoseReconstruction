#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <inputactions.h>
#include <shared_data.h>
#include <iostream>
#include <map>

using namespace std;

static map<int, int> key_action_map;

// Handle multiple simultaneous key presses as the keyboard callback doesn't support this functionality
static void handle_multikey_press() {
	return;
}

// callback for keyboard events
static void keyboard_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {

	//Set window to be closed
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		close_window(window);
		return;
	}
	
	//Adds or removes key-action pairs from the map
	

	return;
}



// callback for mouse events
static void mouse_position_callback(GLFWwindow* window, double xpos, double ypos) {
	
	return;
}

static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
		//
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
		//
	if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS)
		//
	
	return;
}