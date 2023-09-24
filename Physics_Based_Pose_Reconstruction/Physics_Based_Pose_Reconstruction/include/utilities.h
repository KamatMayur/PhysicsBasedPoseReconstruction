#pragma once
#include <mujoco/mujoco.h>
#include <iostream>

using namespace std;

// adds a given value to the actuator specified by the parametere "name", 
// it will return true if the actuator exists else it will return false;
static bool add_actuator_value(const char* name, const double value = 0)
{
	int found_actuator = mj_name2id(m, mjOBJ_ACTUATOR, name);
	if (found_actuator != -1)
	{
		d->ctrl[found_actuator] = value;
		return true;
	}
	else return false;
}