#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Python.h>
#include <utilities.h>
#include <simulate.h>


namespace py = pybind11;

namespace pybind11 {
    namespace detail {
        template <>
        struct type_caster<utilities::jnt_info> {
        public:
            PYBIND11_TYPE_CASTER(utilities::jnt_info, _("utilities::jnt_info"));
            // Conversion from C++ to Python
            static py::handle cast(const utilities::jnt_info& src, py::return_value_policy /* policy */, py::handle /* parent */) {
                py::dict info_dict;
                info_dict["jnt_num"] = src.jnt_num;
                info_dict["names"] = src.names;
                info_dict["x"] = src.x;
                info_dict["y"] = src.y;
                info_dict["z"] = src.z;
                return info_dict.release();
            }
        };
    } // namespace detail
} //


PYBIND11_MODULE(Physics_Based_Pose_Reconstruction , m) {
    // Define the Python bindings for the simulation class
    py::class_<simulation>(m, "simulation")
        .def(py::init<const char*>())
        .def("simulate", &simulation::simulate, py::call_guard<py::gil_scoped_release>())
        .def("add_actuator_value", &simulation::add_actuator_value, py::call_guard<py::gil_scoped_release>())
        .def_readonly ("info", &simulation::info)
        .def_readonly("model_path", &simulation::model_path);
}