/******************************************************************************!
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @author Emre Yilmaz <ae.emre.yilmaz@gmail.com>
 * @author Mark Kotwicz <mark.kotwicz@gmail.com>
 * @date 5 June 2020
 * @version 1.0.0
 * @brief A Dymos implementation for ETOL
 * @section DESCRIPTION
 * A <a href="https://github.com/OpenMDAO/dymos">Dymos</a> implementation for
 * the pure virtual methods in the TrajectoryOptimization class. It casts the
 * trajectory optimization problem as a Nonlinear Linear Programming (NLP) type
 * and solves this NLP with an optimizer.
 ******************************************************************************/

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <execution>
#include <algorithm>
#include <any>
#include <iostream>
#include <ETOL/eDymos.hpp>

namespace py = pybind11;
using namespace py::literals;   // to bring in the `_a` literal   // NOLINT

PYBIND11_EMBEDDED_MODULE(edymos, m) {
// `m` is a `py::module` which is used to bind functions and classes
    m.doc() = "The eDymos implementations for the OpenMDAO ExplicitComponents";
    m.def("compute", &ETOL::eDymos::dymosCompute,
            "Compute OpenMDAO outputs given inputs.");
    m.def("compute_partials", &ETOL::eDymos::dymosComputePartials,
            "Compute OpenMDAO sub-jacobian parts.");
}

namespace ETOL {

#ifdef PSTL_USE_PARALLEL_POLICIES
constexpr std::execution::parallel_policy EXEC_POLICY_SEQ{};
constexpr std::execution::parallel_unsequenced_policy EXEC_POLICY_UNSEQ{};
#else
constexpr std::execution::sequenced_policy EXEC_POLICY_SEQ{};
constexpr std::execution::unsequenced_policy EXEC_POLICY_UNSEQ{};
#endif

// Constructor

eDymos::eDymos() :  eDymos::scoped_interpreter(true),
                    eDymos::TrajectoryOptimizer(),
                    _np(py::module::import("numpy")),
                    _om(py::module::import("openmdao.api")),
                    _dm(py::module::import("dymos")),
                    optimizer_("SNOPT"), max_iter_(1000), with_coloring_(true),
                    _alg(_om.attr("Problem")("model"_a = _om.attr("Group")())) ,
                    _sol(_alg.attr("model").attr("add_subsystem")("traj",
                            _dm.attr("Trajectory")())),
                    mesh_refine_(true), mesh_tol_(1.e-3),  max_mesh_iter_(5),
                    num_segments_(10), order_(3), compressed_(false) {
    this->addODE();
}

// Static Members

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

py::dict eDymos::dymosCompute(void* handle, py::dict inputs) {
    py::dict kv;

    eDymos* ptr = reinterpret_cast<eDymos*>(handle);
    double dt = ptr->getDt();

    // Extract variable values

    // Time
    py::buffer_info tbuf = (inputs["t"].cast<py::array_t<double>>()).request();

    // States
    std::vector<py::buffer_info> xbufs;
    std::vector<size_t> x_idx(ptr->getNStates());
    std::iota(x_idx.begin(), x_idx.end(), 0);
    for (size_t j(0); j < ptr->getNStates(); j++) {
       xbufs.push_back((inputs[getStateName(j).c_str()]
                               .cast<py::array_t<double>>()).request());
    }

    // Controls
    std::vector<py::buffer_info> ubufs;
    std::vector<size_t> u_idx(ptr->getNControls());
    std::iota(u_idx.begin(), u_idx.end(), 0);
    for (size_t j(0); j < ptr->getNControls(); j++) {
       ubufs.push_back((inputs[getControlName(j).c_str()]
                               .cast<py::array_t<double>>()).request());
    }

    // Declare output arrays

    // Objective array
    py::array_t<double> jdotvec =
            py::array_t<double>(  // @suppress("Symbol is not resolved")
            tbuf.size);
    py::buffer_info jdot = jdotvec.request();

    // State time-derivative arrays
    std::vector<py::array_t<double>> xdotvec;
    std::vector<py::buffer_info> xdot;
    for (size_t i(0); i < ptr->getNStates(); i++) {
        xdotvec.push_back(
                py::array_t<double>(     // @suppress("Symbol is not resolved")
                    tbuf.size));
        xdot.push_back(xdotvec.back().request());
    }

    // Path arrays
    std::vector<size_t> p_idx(ptr->getConstraints()->size());
    std::iota(p_idx.begin(), p_idx.end(), 0);
    std::vector<py::array_t<double>> pathvec;
    std::vector<py::buffer_info> path;
    for (size_t i(0); i < ptr->getConstraints()->size(); i++) {
        pathvec.push_back(
                py::array_t<double>(     // @suppress("Symbol is not resolved")
                        tbuf.size));
        path.push_back(pathvec.back().request());
    }

    // Compute values for each time index
    std::vector<size_t> t_idx(tbuf.shape[0]);
    std::iota(t_idx.begin(), t_idx.end(), 0);
    for_each(EXEC_POLICY_UNSEQ,
            t_idx.begin(), t_idx.end(), [&](const size_t& idx){
        // Set variables for callbacks
        // Time
        scalar_t t = reinterpret_cast<double *>(tbuf.ptr)[idx];

        // States
        vector_t x(ptr->getNStates());
        std::transform(EXEC_POLICY_UNSEQ,
                xbufs.begin(), xbufs.end(), x.begin(),
            [&idx](py::buffer_info& xbuf) -> scalar_t {
            return (reinterpret_cast<double *>(xbuf.ptr)[idx]);
        });

        // Controls
        vector_t u(ptr->getNControls());
        std::transform(EXEC_POLICY_UNSEQ,
                ubufs.begin(), ubufs.end(), u.begin(),
            [&idx](py::buffer_info& ubuf) -> scalar_t {
            return (reinterpret_cast<double *>(ubuf.ptr)[idx]);
        });

        // Use callbacks to get compute() results

        // Objective function
        vector_t params = {std::string()};
        std::vector<std::string> pnames = {std::string("")};
        scalar_t obj_val = (*ptr->_objective)(x, u, params, pnames, t, dt);

        // State time derivatives
        std::vector<double> dx_val(ptr->getGradient()->size());
        auto it_dx = ptr->getGradient()->begin();
        std::transform(EXEC_POLICY_UNSEQ,
                ptr->getGradient()->cbegin(), ptr->getGradient()->cend(),
                dx_val.begin(),
            [&x, &u, &t, &dt, &dx_val](const f_t* grad) -> double {
            scalar_t val;
            vector_t params = {std::string()};
            std::vector<std::string> pnames = {std::string()};
            val = (*grad)(x, u, params, pnames, t, dt);
            try {
                return((std::any_cast<std::vector<double>>(val)).at(0));
            } catch (...) {
                std::cout << "compute() gradient any_cast failed!" << std::endl;
            }
            return 0.;
        });

        // Path constraints
        std::vector<double> p_val(ptr->getConstraints()->size());
        std::transform(EXEC_POLICY_UNSEQ,
                ptr->getConstraints()->cbegin(), ptr->getConstraints()->cend(),
                p_val.begin(),
            [&x, &u, &t, &dt, &p_val](const f_t* con) -> double {
            scalar_t val;
            vector_t params = {std::string()};
            std::vector<std::string> pnames = {std::string()};
            val = (*con)(x, u, params, pnames, t, dt);
            try {
                return((std::any_cast<std::vector<double>>(val)).at(0));
            } catch (...) {
                std::cout << "compute() path any_cast failed!" << std::endl;
            }
            return 0.;
        });

        // Pass the results to the numpy array outputs

        // Objective
        try {
        reinterpret_cast<double*>(jdot.ptr)[idx] =
                (std::any_cast<std::vector<double>>(obj_val)).at(0);
        } catch (...) {
            std::cout << "compute() objective any_cast failed!" << std::endl;
        }

        // State time derivatives
        for_each(EXEC_POLICY_UNSEQ, x_idx.cbegin(), x_idx.cend(),
            [&dx_val, &xdot, &idx](const size_t& j) {
            reinterpret_cast<double*>(xdot.at(j).ptr)[idx] = dx_val.at(j);
        });

        // Path constraints
        for_each(EXEC_POLICY_UNSEQ, p_idx.cbegin(), p_idx.cend(),
            [&p_val, &path, &idx](const size_t& j) {
            reinterpret_cast<double*>(path.at(j).ptr)[idx] = p_val.at(j);
        });
    });
    // Store results in the dictionary output
    kv["jdot"] = jdotvec;
    for (size_t j(0); j < ptr->getNStates(); j++)
        kv[getDerivName(j).c_str()] = xdotvec.at(j);
    for (size_t j(0); j < ptr->getConstraints()->size(); j++)
        kv[getPathConstraintName(j).c_str()] = pathvec.at(j);
    return kv;
}

py::dict eDymos::dymosComputePartials(void* handle, py::dict inputs) {
    py::dict kv;

    eDymos* ptr = reinterpret_cast<eDymos*>(handle);
    const double dt = ptr->getDt();

    // Extract variable values

    // Time
    py::buffer_info tbuf = (inputs["t"].cast<py::array_t<double>>()).request();

    // States
    std::vector<py::buffer_info> xbufs;
    std::vector<size_t> x_idx(ptr->getNStates());
    std::iota(x_idx.begin(), x_idx.end(), 0);
    for (size_t j(0); j < ptr->getNStates(); j++)
        xbufs.push_back((inputs[getStateName(j).c_str()]
                                .cast<py::array_t<double>>()).request());

    // Controls
    std::vector<py::buffer_info> ubufs;
    std::vector<size_t> u_idx(ptr->getNControls());
    std::iota(u_idx.begin(), u_idx.end(), x_idx.back() + 1u);
    for (size_t j(0); j < ptr->getNControls(); j++)
        ubufs.push_back((inputs[getControlName(j).c_str()]
                                .cast<py::array_t<double>>()).request());

    // Declare output arrays

    // Objective partial derivative array
    std::vector<py::array_t<double>> jdotvecs;
    std::vector<py::buffer_info> jdot;
    for (size_t j(0); j < (ptr->getNStates() + ptr->getNControls()); j++) {
        jdotvecs.push_back(
                py::array_t<double>(    // @suppress("Symbol is not resolved")
                        tbuf.size));
        jdot.push_back(jdotvecs.back().request());
    }
    auto jdot_it = jdot.begin();

    // State time-derivative partial derivative arrays
    std::vector<py::array_t<double>> xdotvec;
    std::vector<py::buffer_info> xdot;
    for (size_t j(0);
            j < ptr->getNStates() * (ptr->getNStates() + ptr->getNControls());
                j++) {
        xdotvec.push_back(
                py::array_t<double>(    // @suppress("Symbol is not resolved")
                        tbuf.size));
        xdot.push_back(xdotvec.back().request());
    }
    auto xdot_it = xdot.begin();

    // Path constraint partial derivative arrays
    std::vector<size_t> p_idx(ptr->getConstraints()->size());
    std::iota(p_idx.begin(), p_idx.end(), 0);
    std::vector<py::array_t<double>> pathvec;
    std::vector<py::buffer_info> path;
    for (size_t j(0); j < ptr->getConstraints()->size() * (ptr->getNStates()
                    + ptr->getNControls()); j++) {
        pathvec.push_back(
                py::array_t<double>(    // @suppress("Symbol is not resolved")
                        tbuf.size));
        path.push_back(pathvec.back().request());
    }
    auto path_it = path.begin();

    // Compute values for each time index
    std::vector<size_t> t_idx(tbuf.shape[0]);
    std::iota(t_idx.begin(), t_idx.end(), 0);
    for_each(EXEC_POLICY_UNSEQ, t_idx.begin(), t_idx.end(),
        [&](const size_t& idx) {
        // Set variables for callbacks
        // Time
        scalar_t t = reinterpret_cast<double *>(tbuf.ptr)[idx];

        // States
        vector_t x(ptr->getNStates());
        std::transform(EXEC_POLICY_UNSEQ,
                xbufs.begin(), xbufs.end(), x.begin(),
            [&idx](py::buffer_info& xbuf) -> scalar_t {
            return (reinterpret_cast<double *>(xbuf.ptr)[idx]);
        });

        // Controls
        vector_t u(ptr->getNControls());
        std::transform(EXEC_POLICY_UNSEQ,
                ubufs.begin(), ubufs.end(), u.begin(),
            [&idx](py::buffer_info& ubuf) -> scalar_t {
            return (reinterpret_cast<double *>(ubuf.ptr)[idx]);
        });

        // Use user lambda expressions to calculate Dymos compute() values

        // Objective function
        std::vector<double> obj;
        vector_t params = {std::string()};
        std::vector<std::string> pnames = {std::string("partials")};
        scalar_t obj_val = (*ptr->_objective)(x, u, params, pnames, t, dt);
        try {
            obj = std::any_cast<std::vector<double>>(obj_val);
        } catch (...) {
            std::cout << "Error casting objective for compute_partials()"
                    << std::endl;
        }

        // State time derivatives
        std::vector<std::vector<double>> dx(ptr->getGradient()->size());
        std::transform(EXEC_POLICY_UNSEQ,
                ptr->getGradient()->cbegin(), ptr->getGradient()->cend(),
                dx.begin(),
            [&x, &u, &t, &dt, &dx](const f_t* grad) -> std::vector<double> {
            scalar_t val;
            vector_t params = {std::string()};
            std::vector<std::string> pnames = {std::string("partials")};
            val = (*grad)(x, u, params, pnames, t, dt);
            try {
                return(std::any_cast<std::vector<double>>(val));
            } catch (...) {
                std::cout << "Error casting state for compute_partials()"
                        << std::endl;
            }
            std::vector<double> empty;
            return (empty);
        });

        // Path Constraints
        std::vector<std::vector<double>> p(ptr->getConstraints()->size());
        std::transform(EXEC_POLICY_UNSEQ,
                ptr->getConstraints()->cbegin(), ptr->getConstraints()->cend(),
                p.begin(),
            [&x, &u, &t, &dt, &p](const f_t* con)-> std::vector<double> {
            scalar_t val;
            vector_t params = {std::string()};
            std::vector<std::string> pnames = {std::string("partials")};
            val = (*con)(x, u, params, pnames, t, dt);
            try {
                return (std::any_cast<std::vector<double>>(val));
            } catch (...) {
                std::cout << "Error casting path for compute_partials()"
                        << std::endl;
            }
            std::vector<double> empty;
            return (empty);
        });

        auto obj_it = obj.begin();
        auto dx_it = dx.begin();
        auto p_it = p.begin();
        // Partials with respect to the state
        for_each(EXEC_POLICY_UNSEQ, x_idx.cbegin(), x_idx.cend(),
            [&idx, &x_idx, &p_idx, &jdot_it, &obj_it, &dx_it, &xdot_it, &p_it,
             &path_it, &ptr](const size_t& j) {
            // Partial of objective wrt to state j
            reinterpret_cast<double*>((jdot_it + j)->ptr)[idx] = *(obj_it + j);

            // Partial of state time derivatives wrt state j
            const size_t k_x = j * ptr->getNStates();
            for_each(EXEC_POLICY_UNSEQ, x_idx.cbegin(), x_idx.cend(),
                    [&idx, &j, &k_x, &xdot_it, &dx_it, &ptr](const size_t& k) {
                reinterpret_cast<double*>((xdot_it + k_x + k)->ptr)[idx] =
                        (dx_it + k)->at(j);
            });

            // Partial of path constraint k wrt state j
            const size_t k_p = j * ptr->getConstraints()->size();
            for_each(EXEC_POLICY_UNSEQ, p_idx.cbegin(), p_idx.cend(),
                    [&idx, &j, &k_p, &path_it, &p_it, &ptr](const size_t& k) {
                reinterpret_cast<double*>((path_it + k_p + k)->ptr)[idx] =
                        (p_it + k)->at(j);
            });
        });

        const size_t j_dx = ptr->getNStates() * ptr->getNStates();
        const size_t j_dp = ptr->getNStates() * ptr->getConstraints()->size();
        // Partials with respect to the control
        for_each(EXEC_POLICY_UNSEQ, u_idx.cbegin(), u_idx.cend(),
            [&idx, &x_idx, &p_idx, &jdot_it, &obj_it, &dx_it, &p_it,
             &j_dx, &j_dp, &xdot_it, &path_it, &ptr] (const size_t &j) {
            // Partial of objective wrt to control j
            reinterpret_cast<double*>((jdot_it + j)->ptr)[idx] = *(obj_it + j);

            // Partial of state time derivative k wrt control j
            const size_t k_u = (j - ptr->getNStates()) * ptr->getNStates();
            for_each(EXEC_POLICY_UNSEQ, x_idx.cbegin(), x_idx.cend(),
                    [&idx, &j, &j_dx, &k_u, &xdot_it, &dx_it](const size_t& k) {
                reinterpret_cast<double*>((xdot_it + j_dx + k_u + k)->ptr)[idx]
                        = (dx_it + k)->at(j);
            });

            // Partial of path constraint k wrt control j
            const size_t k_p = (j - ptr->getNStates()) *
                    ptr->getConstraints()->size();
            for_each(EXEC_POLICY_UNSEQ, p_idx.cbegin(), p_idx.cend(),
                    [&idx, &j, &j_dp, &k_p, &path_it, &p_it](const size_t& k) {
                reinterpret_cast<double*>((path_it + j_dp + k_p + k)->ptr)[idx]
                        = (p_it + k)->at(j);
            });
        });
    });


    // Store results in the dictionary output
    size_t jdot_iter(0), dx_iter(0), p_iter(0);
    for (size_t j(0); j < ptr->getNStates(); j++) {
        std::string name = getStateName(j);
        kv[std::string("jdot_" + name).c_str()] = jdotvecs.at(jdot_iter++);
        for (size_t k(0); k < ptr->getNStates(); k++)
            kv[(getDerivName(k) + "_" + name).c_str()] = xdotvec.at(dx_iter++);
        for (size_t k(0); k < ptr->getConstraints()->size(); k++)
            kv[(getPathConstraintName(k)  + "_" + name).c_str()] =
                    pathvec.at(p_iter++);
    }
    for (size_t j(0); j < ptr->getNControls(); j++) {
        std::string name = getControlName(j);
        kv[std::string("jdot_" + name).c_str()] = jdotvecs.at(jdot_iter++);
        for (size_t k(0); k < ptr->getNStates(); k++)
            kv[(getDerivName(k) + "_" + name).c_str()] = xdotvec.at(dx_iter++);
        for (size_t k(0); k < ptr->getConstraints()->size(); k++)
            kv[(getPathConstraintName(k)  + "_" + name).c_str()] =
                    pathvec.at(p_iter++);
    }
    return kv;
}

// API

void eDymos::setup() {
    this->setAlg();
    this->setProb();

    // Call "setup" after changing the _prob variable
    // OpenMDAO: MPI processors allocated, model hierarchy is created, etc.
    this->_alg.attr("setup")("check"_a = false);

    this->setGuess();

    // Set mesh refinement tolerance
    _alg.attr("model").attr("traj").attr("phases").attr("phase0")
                   .attr("set_refine_options")("refine"_a = mesh_refine_,
                           "tol"_a = mesh_tol_);
}

void eDymos::solve() {
    _dm.attr("run_problem")(_alg, "refine"_a = mesh_refine_,
            "refine_iteration_limit"_a = max_mesh_iter_);

    // Get score from Dymos
    py::list jval_list = _alg.attr("get_val")(
            "traj.phase0.timeseries.states:jval");
    double j_tf = (*(jval_list.end()-1)).cast<double>();
    this->setScore(j_tf);

    // Get state and control trajectories from Dymos
    this->getTraj();


    /*  Plot results
    py::object plt = py::module::import("matplotlib.pyplot");
    py::object plotsol = py::module::import(
                "dymos.examples.plotting").attr("plot_results");
    py::object exp_out = _sol.attr("simulate")();
    py::list li;
    li.append(py::make_tuple("traj.phase0.timeseries.time",
            "traj.phase0.timeseries.states:x0", "t", "x0"));
    li.append(py::make_tuple("traj.phase0.timeseries.time",
                "traj.phase0.timeseries.controls:u0", "t", "x1"));
    li.append(py::make_tuple("traj.phase0.timeseries.time",
                "traj.phase0.timeseries.p0", "t", "p0"));
    plotsol(li,
            "title"_a = "Path0",
            "p_sol"_a = _alg,
            "p_sim"_a = exp_out);
    plt.attr("show")();
    */
}

void eDymos::debug() {
    _alg.attr("driver").attr("options")["print_results"] = true;
    _alg.attr("set_solver_print")("level"_a = 2);
    if (_alg.attr("driver").attr("options")["optimizer"].cast<std::string>()
            == "SNOPT") {
        _alg.attr("driver").attr("opt_settings")["iSumm"] = 6;
    }
    if (_alg.attr("driver").attr("options")["optimizer"].cast<std::string>()
            == "IPOPT") {
        _alg.attr("driver").attr("opt_settings")["file_print_level"] = 5;
        _alg.attr("driver").attr("opt_settings")["print_level"] = 5;
    }
}

void eDymos::close() {}

// Private functions

void eDymos::setAlg() {
    _alg.attr("set_solver_print")("level"_a = 0);
    _alg.attr("driver") =  _om.attr("pyOptSparseDriver")();
    _alg.attr("driver").attr("options")["optimizer"] = getOptimizer().c_str();
    _alg.attr("driver").attr("options")["print_results"] = false;
    if (_alg.attr("driver").attr("options")["optimizer"].cast<std::string>()
                == "SNOPT") {
        _alg.attr("driver").attr("options")["user_terminate_signal"] =
                    nullptr;
        _alg.attr("driver").attr(
            "opt_settings")["Major iterations limit"] = getMaxIter();
        _alg.attr("driver").attr(
            "opt_settings")["Minor iterations limit"] = getMaxIter();
        _alg.attr("driver").attr("opt_settings")["Verify level"] = 0;
    } else if (_alg.attr("driver")
            .attr("options")["optimizer"].cast<std::string>() == "IPOPT")  {
        _alg.attr("driver").attr(
            "opt_settings")["max_iter"] = getMaxIter();
        _alg.attr("driver").attr(
                "opt_settings")["linear_solver"] = "mumps";
        _alg.attr("driver").attr(
                "opt_settings")["nlp_scaling_method"] = "gradient-based";
        _alg.attr("driver").attr(
                "opt_settings")["hessian_approximation"] = "limited-memory";
        _alg.attr("driver").attr("opt_settings")["file_print_level"] = 0;
        _alg.attr("driver").attr("opt_settings")["print_level"] = 0;
    }
    if (with_coloring_)
        _alg.attr("driver").attr("declare_coloring")();
    _alg.attr("model").attr("linear_solver") =  _om.attr("DirectSolver")();
    _alg.attr("model").attr("nonlinear_solver") =  _om.attr("NewtonSolver")(
            "solve_subsystems"_a = false);
}

void eDymos::setProb() {
    // Define the OpenMDAO problem
    py::object scope = py::module::import("__main__").attr("__dict__");

    // Try to set the number of segments based on the number of time steps
    this->num_segments_ = (this->num_segments_ > (this->getNSteps()+1)/order_) ?
            this->num_segments_ : (this->getNSteps()+1)/order_;
    py::tuple seg_ends = py::module::import(
            "dymos.utils.lgl").attr("lgl")(num_segments_ + 1);
    _prob = _sol.attr("add_phase")("phase0", _dm.attr("Phase")(
                "ode_class"_a = py::eval("eDymosODE", scope),
                        //  py::module::import("edymosode").attr("eDymosODE"),
                "ode_init_kwargs"_a =
                py::dict(  // @suppress("Symbol is not resolved")
                    "edymos"_a = reinterpret_cast<void*>(this),
                    "num_states"_a = static_cast<int>(this->getNStates()),
                    "num_controls"_a = static_cast<int>(this->getNControls()),
                    "num_path_constraints"_a =
                            static_cast<int>(this->getConstraints()->size())),
            "transcription"_a = _dm.attr("Radau")(
                    "num_segments"_a = num_segments_,
                    "segment_ends"_a = *seg_ends.begin(),
                    "order"_a = order_,
                    "compressed"_a = compressed_)));

    // Set the time variable
    double tf = getDt() * getNSteps();
    _prob.attr("set_time_options")(
            "fix_initial"_a = true, "fix_duration"_a = true,
            // "initial_bounds"_a = py::make_tuple(0., 0.),
            // "duration_bounds"_a = py::make_tuple(tf, tf),
            "units"_a = nullptr, "targets"_a = "t");

    // Set the objective variable as a state
    _prob.attr("add_state")("jval", "rate_source"_a = "jdot",
        "fix_initial"_a = true, "fix_final"_a = false, "units"_a = nullptr,
        "solve_segments"_a = true, "lower"_a = 0.0, "targets"_a = "jval");

    // Set the states
    state_t::iterator it_xlo = this->getXlower().begin();
    state_t::iterator it_xup = this->getXupper().begin();
    state_t::iterator it_x0 = this->getX0().begin();
    state_t::iterator it_xf = this->getXf().begin();
    state_t::iterator it_xtol = this->getXtol().begin();
    for (size_t j(0); j < this->getNStates(); j++)  {
        _prob.attr("add_state")(getStateName(j).c_str(),
                "rate_source"_a = getDerivName(j).c_str(),
                "lower"_a = *(it_xlo),
                "upper"_a = *(it_xup),
                "fix_initial"_a = true,
                "fix_final"_a = false,
                "solve_segments"_a = true,
                "targets"_a = getStateName(j).c_str());
        //  _prob.attr("add_boundary_constraint")(getStateName(j).c_str(),
        //        "loc"_a = "initial", "equals"_a = *(it_x0));
        _prob.attr("add_boundary_constraint")(getStateName(j).c_str(),
                        "loc"_a = "final",
                        "upper"_a = *(it_xf) + *(it_xtol),
                        "lower"_a = *(it_xf) - *(it_xtol));
        it_x0++; it_xf++; it_xtol++; it_xlo++; it_xup++;
    }

    // Set the controls
    state_t::iterator it_ulo = this->getUlower().begin();
    state_t::iterator it_uup = this->getUupper().begin();
    for (size_t j(0); j < this->getNControls(); j++) {
        _prob.attr("add_control")(getControlName(j).c_str(),
                "continuity"_a = true,
                "rate_continuity"_a = true,
                "lower"_a = *(it_ulo++),
                "upper"_a = *(it_uup++),
                "targets"_a = getControlName(j).c_str());
       _prob.attr("add_timeseries_output")("name"_a = getControlName(j).c_str(),
               "units"_a = nullptr);
    }

    // Set the path constraints
    for (size_t j(0); j < this->getConstraints()->size(); j++) {
        _prob.attr("add_path_constraint")(
                "name"_a = getPathConstraintName(j).c_str(),
                "upper"_a = 0.,
                "lower"_a = nullptr,
                "units"_a = nullptr);
        _prob.attr("add_timeseries_output")(
                "name"_a = getPathConstraintName(j).c_str(),
                "units"_a = nullptr);
    }

    // Set objective variable that will be minimized
    _prob.attr("add_objective")("jval", "loc"_a = "final",
            "scaler"_a = (this->isMaximized() ? -1 : 1));
}

void eDymos::setGuess() {
    // Set initial guesses
    std::string xprefix = "traj.phase0.states:";
    std::string uprefix = "traj.phase0.controls:";
    _alg["traj.phase0.t_initial"] = 0.;
    _alg["traj.phase0.t_duration"] = getDt() * getNSteps();

    _alg["traj.phase0.states:jval"] = _prob.attr("interpolate")(
            "xs"_a = py::make_tuple(0., getDt() * getNSteps()),
            "ys"_a = py::make_tuple(0.0, 1.0), "nodes"_a = "state_input");

    state_t::iterator it_x0 = this->getX0().begin();
    state_t::iterator it_xf = this->getXf().begin();
    for (size_t j(0); j < this->getNStates(); j++) {
        _alg[(xprefix + getStateName(j)).c_str()] =
                _prob.attr("interpolate")(
                "xs"_a = py::make_tuple(0., getDt() * getNSteps()),
                "ys"_a = py::make_tuple(*(it_x0++), *(it_xf++)),
                "nodes"_a = "state_input");
    }
    state_t::iterator it_ulo = this->getUlower().begin();
    state_t::iterator it_uup = this->getUupper().begin();
    for (size_t j(0); j < this->getNControls(); j++)
        _alg[(uprefix + getControlName(j)).c_str()] =
                _prob.attr("interpolate")(
                        "xs"_a = py::make_tuple(0., getDt() * getNSteps()),
                        "ys"_a = py::make_tuple(*(it_uup++), *(it_ulo++)),
                        "nodes"_a = "control_input");
}

void eDymos::getTraj() {
    py::list tvals = _alg.attr("get_val")("traj.phase0.timeseries.time");

    traj_t* xtraj = this->getXtraj();
    xtraj->clear();

    traj_t* utraj = this->getUtraj();
    utraj->clear();

    std::string xprefix = "traj.phase0.timeseries.states:";
    std::string uprefix = "traj.phase0.timeseries.controls:";
    size_t i(0);
    for (auto tval : tvals) {
        double t = tval.cast<double>();
        state_t x, u;
        for (size_t j(0); j < this->getNStates(); j++) {
            py::list xvals = _alg.attr("get_val")(
                    (xprefix + getStateName(j)).c_str());
            x.push_back(xvals[i].cast<double>());
        }
        xtraj->push_back(traj_elem_t(t, x));

        for (size_t j(0); j < this->getNControls(); j++) {
            py::list uvals = _alg.attr("get_val")(uprefix + getControlName(j));
            u.push_back(uvals[i].cast<double>());
        }
        utraj->push_back(traj_elem_t(t, u));
        i++;
    }
}

std::string eDymos::getOptimizer() const {
    return optimizer_;
}

void eDymos::setOptimizer(std::string optimizer) {
    optimizer_ = optimizer;
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
        
        self.add_input('t', val=np.ones(nn), desc='time', units=None)
        self.add_input('jval', val=np.ones(nn), desc='objective', units=None)
        self.add_output('jdot', val=np.ones(nn), desc='derivative of objective',
                        units=None)
        self.declare_partials(of='jdot', wrt='t', rows=r, cols=c, val=0.0)
        self.declare_partials(of='jdot', wrt='jval', rows=r, cols=c, val=0.0)

        for i in range(ns):
            name = 'x' + str(i)
            descr = 'state: ' + name
            self.add_input(name, val=np.ones(nn), desc=descr, units=None)
            self.declare_partials(of='jdot', wrt=name, rows=r, cols=c)
            
        for i in range(nc):
            name = 'u' + str(i)
            descr = 'control: ' + name
            self.add_input(name, val=np.ones(nn), desc=descr, units=None)
            self.declare_partials(of='jdot', wrt=name, rows=r, cols=c)
            
        for i in range(ns):           
            dname = 'x' + str(i) + 'dot'
            self.add_output(dname, val=np.ones(nn),
                        desc='output', units=None)
            self.declare_partials(of=dname, wrt='t', rows=r, cols=c, val=0.0)
            self.declare_partials(of=dname, wrt='jval', rows=r, cols=c, val=0.0)
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
            self.add_output(pname, val=np.ones(nn),
                        desc='output', units=None)
            self.declare_partials(of=pname, wrt='t', rows=r, cols=c, val=0.0)
            self.declare_partials(of=pname, wrt='jval', rows=r, cols=c, val=0.0)
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
        data = {'t':inputs['x0']}
        for i in range(ns):
            name = 'x' + str(i)
            data[name] = inputs[name]
        for i in range(nc):
            name = 'u' + str(i)
            data[name] = inputs[name]

        result = ed.compute(edymos, data)
        
        outputs['jdot'] = result['jdot']
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
            xname = 'x%i'%i
            data[xname] = inputs[xname]
        for i in range(nc):
            uname = 'u%i'%i
            data[uname] = inputs[uname]

        result_partials = ed.compute_partials(edymos, data)
        
        for i in range(ns):
            xname = 'x%i'%i
            partials['jdot', xname] = result_partials['jdot_' + xname]
            for j in range(ns):
                dname = 'x%idot'%j
                partials[dname, xname] = result_partials[dname + '_'+ xname]
            for j in range(npc):
                pname = 'p%i'%j
                partials[pname, xname] = result_partials[pname + '_'+ xname]
        for i in range(nc):
            uname = 'u%i'%i
            partials['jdot', uname] = result_partials['jdot_'+ uname]
            for j in range(ns):
                dname = 'x%idot'%j
                partials[dname, uname] = result_partials[dname + '_'+ uname]
            for j in range(npc):
                pname = 'p%i'%j
                partials[pname, uname] = result_partials[pname + '_'+ uname]
            )", scope);
}

// Getters and Setters

const pybind11::object& eDymos::getAlg() const {
    return _alg;
}

const pybind11::object& eDymos::getDm() const {
    return _dm;
}

const pybind11::object& eDymos::getNp() const {
    return _np;
}

const pybind11::object& eDymos::getOm() const {
    return _om;
}

const pybind11::object& eDymos::getProb() const {
    return _prob;
}

const pybind11::object& eDymos::getSol() const {
    return _sol;
}

bool eDymos::isCompressed() const {
    return compressed_;
}

void eDymos::setCompressed(const bool compressed) {
    compressed_ = compressed;
}

bool eDymos::isWithColoring() const {
    return with_coloring_;
}

void eDymos::setWithColoring(const bool withColoring) {
    with_coloring_ = withColoring;
}

int eDymos::getMaxMeshIter() const {
    return max_mesh_iter_;
}

void eDymos::setMaxMeshIter(const int maxMeshIter) {
    max_mesh_iter_ = maxMeshIter;
}

bool eDymos::isMeshRefine() const {
    return mesh_refine_;
}

void eDymos::setMeshRefine(const bool meshRefine) {
    mesh_refine_ = meshRefine;
}

double eDymos::getMeshTol() const {
    return mesh_tol_;
}

void eDymos::setMeshTol(const double meshTol) {
    mesh_tol_ = meshTol;
}

int eDymos::getNumSegments() const {
    return num_segments_;
}

void eDymos::setNumSegments(const int numSegments) {
    num_segments_ = numSegments;
}

int eDymos::getOrder() const {
    return order_;
}

void eDymos::setOrder(const int order) {
    order_ = order;
}

int eDymos::getMaxIter() const {
    return max_iter_;
}

void eDymos::setMaxIter(const int maxIter) {
    max_iter_ = maxIter;
}

} /* namespace ETOL */
