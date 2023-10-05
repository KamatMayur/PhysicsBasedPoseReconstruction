#include <pybind11/pybind11.h>
#include <Python.h>
#include <utilities.h>
#include <simulate.h>


namespace py = pybind11;

PYBIND11_MODULE(Physics_Based_Pose_Reconstruction, m) {
	m.def("simulate", &simulate, "runs the physics simulation loop");
	m.def("get_jnt_pos", &get_joint_pos, "returns the positions of the joints");

}