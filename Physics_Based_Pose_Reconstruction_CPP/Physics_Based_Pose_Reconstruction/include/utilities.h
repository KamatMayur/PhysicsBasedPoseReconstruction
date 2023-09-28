#pragma once
#include <mujoco/mujoco.h>
#include <iostream>]
#include <vector>
#include <cstring>

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
// and writes it to the server in order names->x->y->z
static void get_joint_pos(vector<string> &names , vector<double> &x, vector<double> &y, vector<double> &z)
{
	/*for (int i = 0; i < m->njnt; ++i)
	{
		cout << mj_id2name(m, mjOBJ_JOINT, i) << ": " << "X =" << d->xanchor[i * 3] << "Y =" << d->xanchor[i * 3 + 1] << "Zs =" << d->xanchor[i * 3 + 2] << endl;
	}*/
	
	names.resize(m->njnt);
	x.resize(m->njnt);
	y.resize(m->njnt);
	z.resize(m->njnt);
	for (int i = 0; i < m->njnt; i++)
	{
		x[i] = d->xanchor[i * 3];
		y[i] = d->xanchor[i * 3 + 1];
		z[i] = d->xanchor[i * 3 + 2];
		names[i] = mj_id2name(m, mjOBJ_JOINT, i);
	}

	//converts the arrays to byte strean and uploads it to a server
	//buffer for string vector
	int str_buffer_len = sizeof(string) * names.size();
	char* str_buffer = new char[str_buffer_len];
	memcpy(str_buffer, names.data(), str_buffer_len);
	send(client_socket, str_buffer, sizeof(str_buffer), 0);		//upload names to the server
	delete[] str_buffer;										//free the buffer

	//buffer for double vector
	int doub_buffer_len = sizeof(double) * x.size();
	char *doub_buffer = new char[doub_buffer_len];
	memcpy(doub_buffer, x.data(), doub_buffer_len);
	send(client_socket, doub_buffer, sizeof(doub_buffer), 0);	//upload x to the server 

	memcpy(doub_buffer, y.data(), doub_buffer_len);
	send(client_socket, doub_buffer, sizeof(doub_buffer), 0);	//upload y to the server

	memcpy(doub_buffer, z.data(), doub_buffer_len);
	send(client_socket, doub_buffer, sizeof(doub_buffer), 0);	//upload z to the server 
	delete[] doub_buffer;										//free the buffer

	return;
}