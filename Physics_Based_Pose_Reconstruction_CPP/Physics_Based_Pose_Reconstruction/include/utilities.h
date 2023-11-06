#pragma once
#include <mujoco/mujoco.h>
#include <iostream>
#include <vector>
#include <cstring>

using namespace std;


namespace utilities {
	// structure to store joint information; currently stores names and locations;
	struct jnt_info {
		int jnt_num;
		int act_num;
		vector<string> jnt_names;
		vector<string> act_names;
		vector<double> x, y, z;
	};



	// Adds a given value to the actuator specified by the parametere "name", 
	// it will return true if the actuator exists else it will return false
	static bool add_actuator_value(mjData* d, mjModel* m, const char* name, const double value = 0)
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
	static jnt_info get_joint_info(mjModel* m, mjData* d)
	{
		/*for (int i = 0; i < m->njnt; ++i)
		{
			cout << mj_id2name(m, mjOBJ_JOINT, i) << ": " << "X =" << d->xanchor[i * 3] << "Y =" << d->xanchor[i * 3 + 1] << "Zs =" << d->xanchor[i * 3 + 2] << endl;
		}*/

		jnt_info data;

		data.jnt_num = m->njnt;
		data.act_num = m->nu;
		data.jnt_names.resize(m->njnt);
		data.act_names.resize(m->nu);
		data.x.resize(m->njnt);
		data.y.resize(m->njnt);
		data.z.resize(m->njnt);

		for (int i = 0; i < m->njnt; i++)
		{
			data.x[i] = d->xanchor[i * 3];
			data.y[i] = d->xanchor[i * 3 + 1];
			data.z[i] = d->xanchor[i * 3 + 2];
			data.jnt_names[i] = mj_id2name(m, mjOBJ_JOINT, i);
		}

		for (int i = 0; i < m->nu; i++)
		{
			data.act_names[i] = mj_id2name(m, mjOBJ_ACTUATOR, i);
		}

		return data;


	}

}