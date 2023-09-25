#pragma once
#include <mujoco/mujoco.h>
#include <iostream>
#include <string>

using namespace std;

// Adds a given value to the actuator specified by the parametere "name", 
// it will return true if the actuator exists else it will return false
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

// Gives the global cartesian positon of the joints along with the corresponding joint names
static void get_joint_pos(string (&names)[], double(&x)[], double(&y)[], double(&z)[])
{
	/*for (int i = 0; i < m->njnt; ++i)
	{
		cout << mj_id2name(m, mjOBJ_JOINT, i) << ": " << "X =" << d->xanchor[i * 3] << "Y =" << d->xanchor[i * 3 + 1] << "Zs =" << d->xanchor[i * 3 + 2] << endl;
	}*/
	for (int i = 0; i < m->njnt; i++)
	{
		x[i] = d->xanchor[i * 3];
		y[i] = d->xanchor[i * 3 + 1];
		z[i] = d->xanchor[i * 3 + 2];
		names[i] = mj_id2name(m, mjOBJ_JOINT, i);
	}

	return;
}