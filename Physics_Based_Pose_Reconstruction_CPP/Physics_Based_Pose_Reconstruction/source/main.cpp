#include <iostream>
#include <simulate.h>
#include <utilities.h>
#include <thread>
#include <Windows.h>

int main() {
	//utilities::jnt_info info;
	const char m_path[] = "C:/Users/mayur/OneDrive/Documents/GitHub/PhysicsBasedPoseReconstruction/Physics_Based_Pose_Reconstruction_CPP/Physics_Based_Pose_Reconstruction/Mujoco_Models/Humanoid/Ybot.xml";
	const char d_path[] = "C:/Users/mayur/Desktop/hum.mjdata";		//path to save the data

	simulation sim(m_path);
	//sim.simulate();		//normal simulation with event pooling (closes when the window is close or the specified callback is recieved)
	//sim.simulate_from_state(true, true, 5, 1.0/30.0);		//simulate for the specified time and save the data
	sim.simulate_from_state( true, 5, 1.0/30.0);	//simulate from the saved data for the specified time and save the data
	return 0;

}