#include <iostream>
#include <simulate.h>
#include <utilities.h>
#include <thread>

int main() {
	utilities::jnt_info info;
	const char path[] = "C:/Users/mayur/Desktop/PhysicsBasedPoseReconstruction/Physics_Based_Pose_Reconstruction_CPP/Physics_Based_Pose_Reconstruction/Mujoco_Models/Humanoid/Ybot.xml";
	simulation sim(path);
	
	std::thread myThread(&simulation::simulate, &sim);

	cout << "this is a seperate thread";
	int i = 0;
	while (i < 10000) {
		cout << endl << sim.info.x[0];
		i++;
	}
	myThread.join();
	return 0;

}