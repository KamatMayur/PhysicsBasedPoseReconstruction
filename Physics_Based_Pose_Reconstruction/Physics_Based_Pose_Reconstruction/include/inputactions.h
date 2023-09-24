#pragma once
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <shared_data.h>
#include <utilities.h>
#include <iostream>

using namespace std;

static bool camera_mode_hover = false;
static double last_xpos, last_ypos;

// close window
static void close_window(GLFWwindow* window) {
	glfwSetWindowShouldClose(window, true);
	return;
}
