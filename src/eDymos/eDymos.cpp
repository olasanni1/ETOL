/******************************************************************************!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @author Mark Kotwicz
 * @author Emre Yilmaz
 * @author Daniel Sagan
 * @date 5 June 2020
 * @version 1.0.0
 * @brief A Dymos implementation for ETOL
 * @section DESCRIPTION
 * A <a href="https://github.com/OpenMDAO/dymos">Dymos</a> implementation for
 * the pure virtual methods in the TrajectoryOptimization class. It casts the
 * trajectory optimization problem as a Nonlinear Linear Programming (NLP) type.
 ******************************************************************************/


#include <ETOL/eDymos.hpp>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

// https://stackoverflow.com/questions/6626167/build-a-pyobject-from-a-c-function

namespace ETOL {
// Constructors

/*!
 * @todo
 */
eDymos::eDymos() : eDymos::scoped_interpreter(true),
                            eDymos::TrajectoryOptimizer() {
}

// API

/*!
 * @todo
 */
void eDymos::setup() {
    py::print("eDymos Setup");
}

/*!
 * @todo
 */
void eDymos::solve() {
    py::print("eDymos Solve");
    py::object Decimal = py::module::import("decimal").attr("Decimal");
    py::object pi = Decimal("3.14159");
    py::object exp_pi = pi.attr("exp")();
    py::print(py::str(exp_pi));
    py::object decimal_exp = Decimal.attr("exp");
    for (int n = 0; n < 5; n++) {
        py::print(decimal_exp(Decimal(n)));
    }
}

/*!
 * @todo
 */
void eDymos::debug() {
    py::print("eDymos Debug");
}

void eDymos::close() {}


// Private functions
/*!
 * @todo
 */
void eDymos::getTraj() {
    traj_t* xtraj = this->getXtraj();
    xtraj->clear();

    traj_t* utraj = this->getUtraj();
    utraj->clear();
    double t;
    state_t x, u;
    xtraj->push_back(traj_elem_t(t, x));
    utraj->push_back(traj_elem_t(t, u));
}

} /* namespace ETOL */
