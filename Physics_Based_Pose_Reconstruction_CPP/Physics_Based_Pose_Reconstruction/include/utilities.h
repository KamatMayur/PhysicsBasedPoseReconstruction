#pragma once
#include <mujoco/mujoco.h>
#include <iostream>]
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
static bool get_joint_pos()
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


	int bytes_sent;															//stores the output of send() to check for errors

	//buffer for jnt_info 
	int buffer_len = sizeof(string) * data.names.size() + sizeof(double) * data.x.size() * 3;		//actual size of data in bytes
	char* buffer = new char[buffer_len];
	memcpy(buffer, &data, buffer_len);
	bytes_sent = send(client_socket, buffer, sizeof(buffer), 0);		//upload names to the server
	delete[] buffer;		//free the buffer

	//check if upload was successfull
	if (bytes_sent == SOCKET_ERROR) {
		cerr << "Sending X vector failed \n Exiting...";
		closesocket(client_socket);
		WSACleanup();
		return false;

	}
	




	//converts the arrays to byte strean and uploads it to a server
	
	//int bytes_sent;															//stores the output of send() to check for errors

	////buffer for string vector
	//int str_buffer_len = sizeof(string) * names.size();
	//char* str_buffer = new char[str_buffer_len];
	//memcpy(str_buffer, names.data(), str_buffer_len);
	//bytes_sent = send(client_socket, str_buffer, sizeof(str_buffer), 0);	//upload names to the server

	////check if upload was successfull
	//if (bytes_sent == SOCKET_ERROR) {
	//	cerr << "Sending X vector failed /n Exiting...";
	//	closesocket(client_socket);
	//	WSACleanup();
	//	return false;

	//}
	//delete[] str_buffer;													//free the buffer

	////buffer for double vector				
	//int doub_buffer_len = sizeof(double) * x.size();
	//char *doub_buffer = new char[doub_buffer_len];
	//memcpy(doub_buffer, x.data(), doub_buffer_len);
	//bytes_sent = send(client_socket, doub_buffer, sizeof(doub_buffer), 0);	//upload x to the server 

	////check if upload was successfull
	//if (bytes_sent == SOCKET_ERROR) {
	//	cerr << "Sending X vector failed /n Exiting...";
	//	closesocket(client_socket);
	//	WSACleanup();
	//	return false;

	//}

	//memcpy(doub_buffer, y.data(), doub_buffer_len);
	//bytes_sent = send(client_socket, doub_buffer, sizeof(doub_buffer), 0);	//upload y to the server

	////check if upload was successfull
	//if (bytes_sent == SOCKET_ERROR) {
	//	cerr << "Sending X vector failed /n Exiting...";
	//	closesocket(client_socket);
	//	WSACleanup();
	//	return false;

	//}

	//memcpy(doub_buffer, z.data(), doub_buffer_len);
	//bytes_sent = send(client_socket, doub_buffer, sizeof(doub_buffer), 0);	//upload z to the server 

	////check if upload was successfull
	//if (bytes_sent == SOCKET_ERROR) {
	//	cerr << "Sending X vector failed /n Exiting...";
	//	closesocket(client_socket);
	//	WSACleanup();
	//	return false;

	//}
	//delete[] doub_buffer;													//free the buffer

	//return true;
}