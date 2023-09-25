#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

extern mjData* d;	//defined and updated in main.cpp
extern mjModel* m;	//defined in main.cpp
extern mjvCamera cam;		//abstract camera
extern mjvOption opt;		//visualization options
extern mjvScene scn;		//abstract scene
extern mjrContext con;		//custom GPU context

//data needed to be shared across different programs.....Make sure to define the variables in some file.
static void updateSharedData(mjData* d, mjModel* m, mjvCamera cam, mjvOption opt, mjvScene scn, mjrContext con) {
	d = d;
	m = m;
	cam = cam;
	opt = opt;
	scn = scn;
	con = con;
}