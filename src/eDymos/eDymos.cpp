/******************************************************************************!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 5 June 2020
 * @version 1.0.0
 * @brief A Dymos implementation for ETOL
 * @section DESCRIPTION
 * A <a href="https://github.com/OpenMDAO/dymos">Dymos</a> implementation for
 * the pure virtual methods in the TrajectoryOptimization class. It casts the
 * trajectory optimization problem as a Nonlinear Linear Programming (NLP) type.
 * Constructed to solve optimal control problems like...
 * https://openmdao.github.io/dymos/examples/vanderpol.html
 ******************************************************************************/

#include <ETOL/eDymos.hpp>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <any>
#include <iostream>

namespace py = pybind11;
using namespace py::literals;   // to bring in the `_a` literal   // NOLINT


namespace ETOL {

PYBIND11_EMBEDDED_MODULE(edymos, m) {
// `m` is a `py::module` which is used to bind functions and classes
    m.doc() = "The eDymos implementations for the OpenMDAO ExplicitComponents";
    m.def("compute", &eDymos::dymosCompute,
            "Compute OpenMDAO outputs given inputs.");
    m.def("compute_partials", &eDymos::dymosComputePartials,
            "Compute OpenMDAO sub-jacobian parts.");
}

eDymos::eDymos() : eDymos::scoped_interpreter(true),
                            eDymos::TrajectoryOptimizer(),
                            _np(py::module::import("numpy")),
                            _om(py::module::import("openmdao.api")),
                            _dm(py::module::import("dymos")) {
    this->addODE();
}

// API
void eDymos::setup() {
    // Define a dymos component
    setAlg();

    setSol();

    setProb();
    std::cout << "setProb() complete" << std::endl;

    _alg.attr("model").attr("linear_solver") = _om.attr("DirectSolver")();
    _alg.attr("setup")("check"_a = true);

    setGuess();
    std::cout << "setGuess() complete" << std::endl;

    _alg.attr("final_setup")();
    std::cout << "setup complete" << std::endl;
}

void eDymos::solve() {
    if (!_alg.attr("run_driver")())
        getTraj();
}

void eDymos::debug() {
    if (_alg.attr("driver").attr("options")["optimizer"].cast<std::string>()
            == "SNOPT")
        _alg.attr("driver").attr("opt_settings")["iSumm"] = 6;
}

void eDymos::close() {}

// Static Members

py::dict eDymos::dymosCompute(void* handle, py::dict inputs) {
    py::dict kv;
    scalar_t obj_val;

    std::vector<py::buffer_info> xbufs, ubufs;

    eDymos* ptr = reinterpret_cast<eDymos*>(handle);
    double dt = ptr->getDt();

    // Extract variable values
    py::array_t<double> tvec = inputs["t"].cast<py::array_t<double>>();
    py::buffer_info tbuf = tvec.request();

    for (size_t j(0); j < ptr->getNStates(); j++) {
        py::array_t<double> xvec = inputs[getStateName(j).c_str()]
                                      .cast<py::array_t<double>>();
        xbufs.push_back(xvec.request());
    }

    for (size_t j(0); j < ptr->getNControls(); j++) {
        py::array_t<double> uvec = inputs[getControlName(j).c_str()]
                                      .cast<py::array_t<double>>();
        ubufs.push_back(uvec.request());
    }

    // Declare output arrays
    auto Jdotvec = py::array_t<double>(  // @suppress("Symbol is not resolved")
            tbuf.size);
    std::vector<py::array_t<double>> xdotvec(ptr->getNStates(),
                py::array_t<double>(     // @suppress("Symbol is not resolved")
                    tbuf.size));
    std::vector<py::array_t<double>> pvec(ptr->getConstraints()->size(),
                py::array_t<double>(     // @suppress("Symbol is not resolved")
                        tbuf.size));

    for (size_t idx(0); idx < tbuf.shape[0]; idx++) {
        vector_t x, u, dx_val, p_val;
        // Set variables for callbacks
        scalar_t t = static_cast<double *>(tbuf.ptr)[idx];

        for_each(xbufs.begin(), xbufs.end(), [&x, &idx](py::buffer_info& xbuf) {
           x.push_back(static_cast<double *>(xbuf.ptr)[idx]);
        });
        for_each(xbufs.begin(), xbufs.end(), [&u, &idx](py::buffer_info& ubuf) {
           u.push_back(static_cast<double *>(ubuf.ptr)[idx]);
        });

        // Call callbacks for calculating computes
        // Objective function
        obj_val = (*ptr->_objective)(x, u, {std::string()}, {std::string()},
                t, dt);
        // State time derivatives
        for_each(ptr->getGradient()->begin(), ptr->getGradient()->end(),
            [&x, &u, &t, &dt, &dx_val](f_t* grad) {
                scalar_t val;
                val = (*grad)(x, u, {std::string()}, {std::string()}, t, dt);
                dx_val.push_back(val);
        });
        // Path constraints
        for_each(ptr->getConstraints()->begin(), ptr->getConstraints()->end(),
            [&x, &u, &t, &dt, &p_val](f_t* con) {
                scalar_t val;
                val = (*con)(x, u, {std::string()}, {std::string()}, time, dt);
                p_val.push_back(val);
        });

        // Pass the results to the numpy array outputs
        try {
            std::vector<double> obj = std::any_cast<std::vector<double>>(
                    obj_val);
            static_cast<double*>(Jdotvec.request().ptr)[idx] = obj.at(0);

            size_t sIdx = 0;
            for_each(dx_val.begin(), dx_val.end(),
                [&kv, &sIdx, &xdotvec, &idx](scalar_t &val){
                    static_cast<double*>(xdotvec.at(sIdx++).request().ptr)[idx]
                       = (std::any_cast<std::vector<double>>(val)).at(0);
            });

            size_t pIdx = 0;
            for_each(p_val.begin(), p_val.end(),
                [&kv, &pIdx, &pvec, &idx](scalar_t &val){
                    static_cast<double*>(pvec.at(pIdx++).request().ptr)[idx] =
                            (std::any_cast<std::vector<double>>(val)).at(0);
            });
        } catch (...) {
            std::cout << "Error casting std::any for compute()" << std::endl;
        }
    }

    // Store results in the dictionary output
    kv["Jdot"] = Jdotvec;
    for (size_t j(0); j < ptr->getNStates(); j++)
        kv[getDerivName(j).c_str()] = xdotvec.at(j);
    for (size_t j(0); j < ptr->getConstraints()->size(); j++)
        kv[getPathConstraintName(j).c_str()] = pvec.at(j);
    return kv;
}

py::dict eDymos::dymosComputePartials(void* handle, py::dict inputs) {
    py::dict kv;
    scalar_t obj_val;

    std::vector<py::buffer_info> xbufs, ubufs;

    std::cout << "Cast begin complete" << std::endl;
    eDymos* ptr = reinterpret_cast<eDymos*>(handle);
    double dt = ptr->getDt();
    std::cout << "Cast handle complete" << std::endl;

    // Extract variable values
    py::array_t<double> tvec = inputs["t"].cast<py::array_t<double>>();
    py::buffer_info tbuf = tvec.request();
    std::cout << "Cast time complete" << std::endl;

    for (size_t j(0); j < ptr->getNStates(); j++) {
        py::array_t<double> xvec = inputs[getStateName(j).c_str()]
                                      .cast<py::array_t<double>>();
        xbufs.push_back(xvec.request());
    }
    std::cout << "Cast state complete" << std::endl;

    for (size_t j(0); j < ptr->getNControls(); j++) {
        py::array_t<double> uvec = inputs[getControlName(j).c_str()]
                                      .cast<py::array_t<double>>();
        ubufs.push_back(uvec.request());
    }
    std::cout << "Cast control complete" << std::endl;

    // Declare output arrays
    std::vector<py::array_t<double>> Jdotvecs(
            ptr->getNStates() + ptr->getNControls(),
                py::array_t<double>(    // @suppress("Symbol is not resolved")
                        tbuf.size));
    std::vector<py::array_t<double>> xdotvec(
            ptr->getNStates() * (ptr->getNStates() + ptr->getNControls()),
                py::array_t<double>(    // @suppress("Symbol is not resolved")
                    tbuf.size));
    std::vector<py::array_t<double>> pvec(
            ptr->getConstraints()->size() * (ptr->getNStates()
                    + ptr->getNControls()),
            py::array_t<double>(        // @suppress("Symbol is not resolved")
                        tbuf.size));
    std::cout << "Created arrays" << std::endl;

    for (size_t idx(0); idx < tbuf.shape[0]; idx++) {
        vector_t x, u;
        std::vector<double> obj;
        std::vector<std::vector<double>> dx, p;
        // Set variables for callbacks
        scalar_t t = static_cast<double *>(tbuf.ptr)[idx];
        for_each(xbufs.begin(), xbufs.end(), [&x, &idx](py::buffer_info& xbuf) {
           x.push_back(static_cast<double *>(xbuf.ptr)[idx]);
        });
        for_each(xbufs.begin(), xbufs.end(), [&u, &idx](py::buffer_info& ubuf) {
           u.push_back(static_cast<double *>(ubuf.ptr)[idx]);
        });
        std::cout << "Set variables" << std::endl;


        // Call callbacks for calculating computes
        // Objective function
        obj_val = (*ptr->_objective)(x, u, {std::string()},
                    {std::string("partials")}, t, dt);
        try {
             obj = std::any_cast<std::vector<double>>(obj_val);
        } catch (...) {
            std::cout << "Error casting the objective value" << std::endl;
        }
        std::cout << "Get obj" << std::endl;
        // Time derivatives
        for_each(ptr->getGradient()->begin(), ptr->getGradient()->end(),
            [&x, &u, &t, &dt, &dx](f_t* grad) {
                scalar_t val;
                val = (*grad)(x, u, {std::string()}, {"partials"}, t, dt);
                try {
                    dx.push_back(std::any_cast<std::vector<double>>(val));
                } catch (...) {
                    std::cout << "Error casting the state derivatives"
                                << std::endl;
                }
        });
        std::cout << "Get grad" << std::endl;
        // Path Constraints
        for_each(ptr->getConstraints()->begin(), ptr->getConstraints()->end(),
            [&x, &u, &t, &dt, &p](f_t* con) {
                scalar_t val;
                val = (*con)(x, u, {std::string()}, {"partials"}, t, dt);
                try {
                    p.push_back(std::any_cast<std::vector<double>>(val));
                } catch (...) {
                    std::cout << "Error casting the path constraints"
                            << std::endl;
                }
        });
        std::cout << "Called callbacks" << std::endl;
        std::cout << "Obj: ";
        for (auto val : obj)
            std::cout << val << ", ";
        std::cout << std::endl;
        std::cout << "dx0: ";
        for (auto val : dx.at(0))
            std::cout << val << ", ";
        std::cout << std::endl;
        std::cout << "dx1: ";
        for (auto val : dx.at(1))
            std::cout << val << ", ";
        std::cout << std::endl;


        size_t k_jdot(0), k_dx(0), k_p(0);
        // Pass the partials wrt to states to numpy array outputs
        for (size_t j(0); j < ptr->getNStates(); j++) {
            // Set objective partials
            static_cast<double*>(Jdotvecs.at(k_jdot++).request().ptr)[idx] =
                    obj.at(j);
            std::cout << "Add obj to numpy array" << std::endl;
            // Set state derivative partials
            for_each(dx.begin(), dx.end(),
                [&idx, &j, &k_dx, &xdotvec](std::vector<double> &val){
                    static_cast<double*>(xdotvec.at(k_dx++).request().ptr)[idx]
                        = val.at(j);
            });
            std::cout << "Add dx to numpy array" << std::endl;
            // Set path constraint partials
            for_each(p.begin(), p.end(),
                [&idx, &j, &k_p, &pvec](std::vector<double> &val){
                    static_cast<double*>(pvec.at(k_p++).request().ptr)[idx] =
                            val.at(j);
            });
            std::cout << "Add p to numpy array" << std::endl;
        }
        std::cout << "Set state numpy arrays" << std::endl;

        // Pass the partials wrt to the controls to numpy array outputs
        for (size_t j(ptr->getNStates());
                j < (ptr->getNControls() + ptr->getNStates()); j++) {
            // Set objective partials
            static_cast<double*>(Jdotvecs.at(k_jdot++).request().ptr)[idx]=
                    obj.at(j);
            std::cout << "Add obj to numpy array" << std::endl;
            // Set state derivative partials
            for_each(dx.begin(), dx.end(),
                [&idx, &j, &k_dx, &xdotvec](std::vector<double> &val){
                    static_cast<double*>(xdotvec.at(k_dx++).request().ptr)[idx]
                        = val.at(j);
            });
            std::cout << "Add dx to numpy array" << std::endl;
            // Set path constraint partials
            for_each(p.begin(), p.end(),
                [&idx, &j, &k_p, &pvec](std::vector<double> &val){
                    static_cast<double*>(pvec.at(k_p++).request().ptr)[idx] =
                        val.at(j);
            });
            std::cout << "Add p to numpy array" << std::endl;
        }
    }


    // Store results in the dictionary output
    size_t jdot_iter(0), dx_iter(0), p_iter(0);
    for (size_t j(0); j < ptr->getNStates(); j++) {
        std::string name = getStateName(j);
        kv[std::string("Jdot_" + name).c_str()] = Jdotvecs.at(jdot_iter++);
        for (size_t k(0); k < ptr->getNStates(); k++)
            kv[(getDerivName(k) + "_" + name).c_str()] = xdotvec.at(dx_iter++);
        for (size_t k(0); k < ptr->getConstraints()->size(); k++)
            kv[(getPathConstraintName(k)  + "_" + name).c_str()] =
                    pvec.at(p_iter++);
    }
    for (size_t j(0); j < ptr->getNControls(); j++) {
        std::string name = getControlName(j);
        kv[std::string("Jdot_" + name).c_str()] = Jdotvecs.at(jdot_iter++);
        for (size_t k(0); k < ptr->getNStates(); k++)
            kv[(getDerivName(k) + "_" + name).c_str()] = xdotvec.at(dx_iter++);
        for (size_t k(0); k < ptr->getConstraints()->size(); k++)
            kv[(getPathConstraintName(k)  + "_" + name).c_str()] =
                    pvec.at(p_iter++);
    }
    return kv;
}

std::string eDymos::getStateName(const size_t& sIdx) {
    return("x" + std::to_string(sIdx));
}

std::string eDymos::getDerivName(const size_t& sIdx) {
    return (eDymos::getStateName(sIdx) + "dot");
}

std::string eDymos::getControlName(const size_t& sIdx) {
    return("u" + std::to_string(sIdx));
}

std::string eDymos::getPathConstraintName(const size_t& pIdx) {
    return("p" + std::to_string(pIdx));
}

// Private functions
void eDymos::setAlg() {
    _alg = _om.attr("Problem")("model"_a = _om.attr("Group")());
    _alg.attr("driver") = _om.attr("ScipyOptimizeDriver")();
    _alg.attr("driver").attr("declare_coloring")();
}

void eDymos::setSol() {
    _sol =  _alg.attr("model").attr("add_subsystem")("traj",
                _dm.attr("Trajectory")());
}
/**
 * @todo Update num_segements and order
 */
void eDymos::setProb() {
    // Define the OpenMDAO problem
    py::object scope = py::module::import("__main__").attr("__dict__");
    _prob = _sol.attr("add_phase")("phase0", _dm.attr("Phase")(
                "ode_class"_a = py::eval("eDymosODE", scope),
                "ode_init_kwargs"_a =
                py::dict(  // @suppress("Symbol is not resolved")
                    "edymos"_a = reinterpret_cast<void*>(this),
                    "num_states"_a = static_cast<int>(this->getNStates()),
                    "num_controls"_a = static_cast<int>(this->getNControls()),
                    "num_path_constraints"_a =
                            static_cast<int>(this->getConstraints()->size())),
            "transcription"_a = _dm.attr("GaussLobatto")(
                    "num_segments"_a = this->getNSteps(),
                    "order"_a = 3)));

    // Set the time variable
    _prob.attr("set_time_options")("fix_initial"_a = true, "units"_a = nullptr,
            "duration_val"_a = getDt() * getNSteps(), "targets"_a = "t");

    // Set the objective variable as a state
    _prob.attr("add_state")("J", "fix_initial"_a = false, "fix_final"_a = false,
            "rate_source"_a = "Jdot", "units"_a = nullptr);

    // Set the states
    state_t::iterator it_xlo = this->getXlower().begin();
    state_t::iterator it_xup = this->getXupper().begin();
    state_t::iterator it_x0 = this->getX0().begin();
    state_t::iterator it_xf = this->getXf().begin();
    state_t::iterator it_xtol = this->getXtol().begin();
    for (size_t j(0); j < this->getNStates(); j++)  {
        double lo = *(it_xf) - *(it_xtol);
        double up = *(it_xf++) + *(it_xtol++);
        _prob.attr("add_state")(getStateName(j).c_str(),
                "rate_source"_a = getDerivName(j),
                "units"_a = nullptr,
                "lower"_a = *(it_xlo++),
                "upper"_a = *(it_xup++),
                "fix_initial"_a = true,
                "fix_final"_a = true,
                "solve_segments"_a = false,
                "targets"_a = getStateName(j).c_str());
        _prob.attr("add_boundary_constraint")(getStateName(j).c_str(),
                "loc"_a = "initial", "equals"_a = *(it_x0++));
        _prob.attr("add_boundary_constraint")(getStateName(j).c_str(),
                "loc"_a = "final", "lower"_a = lo, "upper"_a = up);
    }


    // Set the controls
    state_t::iterator it_ulo = this->getUlower().begin();
    state_t::iterator it_uup = this->getUupper().begin();
    for (size_t j(0); j < this->getNControls(); j++)
        _prob.attr("add_control")(getControlName(j).c_str(),
                "continuity"_a = false,
                "rate_continuity"_a = false,
                "units"_a = nullptr,
                "lower"_a = *(it_ulo++),
                "upper"_a = *(it_uup++),
                "targets"_a = getControlName(j).c_str());

    // Add objective initial value
    _prob.attr("add_boundary_constraint")("J", "loc"_a = "initial",
                "equals"_a = 0.);
    // Set objective variable that will be minimized
    _prob.attr("add_objective")("J", "loc"_a = "final");
}

void eDymos::setGuess() {
    // Set initial guesses
    std::string xprefix = "traj.phase0.states:";
    std::string uprefix = "traj.phase0.controls:";
    _alg["traj.phase0.t_initial"] = 0.;
    _alg["traj.phase0.t_duration"] = getDt() * getNSteps();

    _alg["traj.phase0.states:J"] = _prob.attr("interpolate")(
            "ys"_a = py::make_tuple(0.0, 0.0), "nodes"_a = "state_input");

    for (size_t j(0); j < this->getNStates(); j++)
        _alg[(xprefix + getStateName(j)).c_str()] = _prob.attr("interpolate")(
                "ys"_a = py::make_tuple(0.0, 0.0), "nodes"_a = "state_input");

    for (size_t j(0); j < this->getNControls(); j++)
        _alg[(uprefix + getControlName(j)).c_str()] = _prob.attr("interpolate")(
                "ys"_a = py::make_tuple(0.0, 0.0), "nodes"_a = "control_input");
}

void eDymos::getTraj() {
    py::list tval = _prob.attr("get_val")("traj.phase0.timeseries.time");

    traj_t* xtraj = this->getXtraj();
    xtraj->clear();

    traj_t* utraj = this->getUtraj();
    utraj->clear();

    std::string xprefix = "traj.phase0.timeseires.states:";
    std::string uprefix = "traj.phase0.timeseires.states:";
    for (size_t i(0); i < this->getNSteps(); i++) {
        double t = 0.0;  // *(tval.begin() + i)).cast<double>();
        state_t x, u;
        for (size_t j(0); j < this->getNStates(); j++) {
            py::list xvals = _prob.attr("get_val")(
                    (xprefix + getStateName(j)).c_str());
            x.push_back(xvals[i].cast<double>());
        }
        xtraj->push_back(traj_elem_t(t, x));

        for (size_t j(0); j < this->getNControls(); j++) {
            py::list uvals = _prob.attr("get_val")(uprefix + getControlName(j));
            u.push_back(uvals[i].cast<double>());
        }
        utraj->push_back(traj_elem_t(t, u));
    }
}

void eDymos::addODE() {
    py::object scope = py::module::import("__main__").attr("__dict__");

    py::exec(R"(
import numpy as np
import openmdao.api as om
import edymos as ed

class eDymosODE(om.ExplicitComponent):
    """
    Notes:
    """
    def initialize(self):
        self.options.declare('num_nodes', types=int)
        
        # Set by eDymos
        self.options.declare('edymos', types=None)
        self.options.declare('num_states', types=int)
        self.options.declare('num_controls', types=int)
        self.options.declare('num_path_constraints', types=int)
        
    def setup(self):
        nn = self.options['num_nodes']
        ns = self.options['num_states']
        nc = self.options['num_controls']
        npc = self.options['num_path_constraints']
        r = c = np.arange(nn)
        
        self.add_input('t', val=np.zeros(nn), desc='time', units=None)
        self.add_output('Jdot', val=np.zeros(nn), desc='derivative of objective',
                        units=None)
               
        for i in range(ns):
            name = 'x' + str(i)
            descr = 'state: ' + name
            self.add_input(name, val=np.zeros(nn), desc=descr, units=None)
            self.declare_partials(of='Jdot', wrt=name,
                                  rows=r, cols=c)
            
        for i in range(nc):
            name = 'u' + str(i)
            descr = 'control: ' + name
            self.add_input(name, val=np.zeros(nn), desc=descr, units=None)
            self.declare_partials(of='Jdot', wrt=name,
                                  rows=r, cols=c)
            
        for i in range(ns):           
            dname = 'x' + str(i) + 'dot'
            self.add_output(dname, val=np.zeros(nn),
                        desc='output', units=None)
            for j in range(ns):
                name = 'x' + str(j)
                self.declare_partials(of=dname, wrt=name,
                                  rows=r, cols=c)
                
            for j in range(nc):
                name = 'u' + str(j)
                self.declare_partials(of=dname, wrt=name,
                                  rows=r, cols=c)
                        
        for i in range(npc):
            pname = 'p' + str(i)
            self.add_output(pname, val=np.zeros(nn),
                        desc='output', units=None)
            for j in range(ns):
                name = 'x' + str(j)
                self.declare_partials(of=pname, wrt=name,
                                  rows=r, cols=c)
                
            for j in range(nc):
                name = 'u' + str(j)
                self.declare_partials(of=pname, wrt=name,
                                  rows=r, cols=c)
            
    def compute(self, inputs, outputs):
        edymos = self.options['edymos']
        ns = self.options['num_states']
        nc = self.options['num_controls']
        npc = self.options['num_path_constraints']
        data = {'t':inputs['t']}
        for i in range(ns):
            name = 'x' + str(i)
            data[name] = inputs[name]
        for i in range(nc):
            name = 'u' + str(i)
            data[name] = inputs[name]
        
        result = ed.compute(edymos, data)
        
        outputs['Jdot'] = result['Jdot']
        for i in range(ns):
            name = 'x' + str(i) + 'dot'
            outputs[name] = result[name]
        for i in range(npc):
            name = 'p' + str(i)
            outputs[name] = result[name]
            
                     
    def compute_partials(self, inputs, partials):
        edymos = self.options['edymos']
        ns = self.options['num_states']
        nc = self.options['num_controls']
        npc = self.options['num_path_constraints']
        data = {'t':inputs['t']}
        for i in range(ns):
            name = 'x' + str(i)
            data[name] = inputs[name]
        for i in range(nc):
            name = 'u' + str(i)
            data[name] = inputs[name]
        
        result = ed.compute_partials(edymos, data)
                
        for i in range(ns):
            name = 'x' + str(i)
            partials['Jdot', name] = result['Jdot_' + name]
            for j in range(ns):
                dname = 'x' + str(j) + 'dot'
                partials[dname, name] = result[dname + '_'+ name]
            for j in range(npc):
                dname = 'p' + str(j)
                partials[dname, name] = result[dname + '_'+ name]
        for i in range(nc):
            name = 'u' + str(i)
            partials['Jdot', name] = result['Jdot' + '_'+ name]
            for j in range(ns):
                dname = 'x' + str(j) + 'dot'
                partials[dname, name] = result[dname + '_'+ name]
            for j in range(npc):
                dname = 'p' + str(j)
                partials[dname, name] = result[dname + '_'+ name]
            )", scope);
}

} /* namespace ETOL */
