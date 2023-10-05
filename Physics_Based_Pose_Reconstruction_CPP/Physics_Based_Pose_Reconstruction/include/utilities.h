#pragma once
#include <mujoco/mujoco.h>
#include <iostream>
#include <shared_data.h>
#include <vector>
#include <cstring>

using namespace std;


// structure to store joint information; currently stores names and locations;
struct jnt_info {
	int jnt_num;
	vector<string> names;
	vector<double> x, y, z;
};


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
// and writes it to the server in order names->x->y->z
// will return false if the upload to server fails
static jnt_info get_joint_pos()
{
	/*for (int i = 0; i < m->njnt; ++i)
	{
		cout << mj_id2name(m, mjOBJ_JOINT, i) << ": " << "X =" << d->xanchor[i * 3] << "Y =" << d->xanchor[i * 3 + 1] << "Zs =" << d->xanchor[i * 3 + 2] << endl;
	}*/

	jnt_info data;

	data.jnt_num = m->njnt;
	data.names.resize(m->njnt);
	data.x.resize(m->njnt);
	data.y.resize(m->njnt);
	data.z.resize(m->njnt);

	for (int i = 0; i < m->njnt; i++)
	{
		data.x[i] = d->xanchor[i * 3];
		data.y[i] = d->xanchor[i * 3 + 1];
		data.z[i] = d->xanchor[i * 3 + 2];
		data.names[i] = mj_id2name(m, mjOBJ_JOINT, i);
	}

	return data;


}