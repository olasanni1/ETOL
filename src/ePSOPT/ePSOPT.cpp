/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 6 May 2020
 * @version 1.0.0
 * @brief A PSOPT version 5.0.0 implementation for ETOL
 * @section DESCRIPTION
 * A <a href="http://www.psopt.org/">PSOPT 4.0.1</a> implementation for the
 * pure virtual methods in the TrajectoryOptimization class. It casts the
 * trajectory optimization problem as a Nonlinear Linear Programming (NLP) type.
 ******************************************************************************/

#include <iostream>
#include <execution>
#include <numeric>
#include <ETOL/ePSOPT.hpp>
namespace ETOL {

#ifdef PSTL_USE_PARALLEL_POLICIES
constexpr std::execution::parallel_policy EXEC_POLICY_SEQ{};
constexpr std::execution::parallel_unsequenced_policy EXEC_POLICY_UNSEQ{};
#else
constexpr std::execution::sequenced_policy EXEC_POLICY_SEQ{};
constexpr std::execution::unsequenced_policy EXEC_POLICY_UNSEQ{};
#endif

// Constructors
/**
 * Pointer to this class' object is stored in PSOPT's problem data structure
 */
ePSOPT::ePSOPT() : ePSOPT::TrajectoryOptimizer() {
    _problem.name = "ETOL Problem";
    _problem.outfilename = "ePSOPT.txt";
    _problem.nphases = 1;
    _problem.nlinkages = 0;
    ::psopt_level1_setup(this->_problem);
    _problem.user_data = reinterpret_cast<void*>(this);
}

// API
/**
 * Sets all initial guesses to 0, except the time guess. The time guess is a
 * equally set from 0 to dt*nstep. Configures the algorithm to use IPOPT to
 * solve a NLP problem that is formed from enforcing constraints at Legendre
 * nodes and with derivative computed with automatic differentiation.
 */
void ePSOPT::setup() {
    this->_problem.phases(1).nstates = this->getNStates();
    this->_problem.phases(1).ncontrols = this->getNControls();
    this->_problem.phases(1).nevents = 2 * this->getNStates();
    this->_problem.phases(1).nodes = (RowVectorXi(1) <<
            (this->getNSteps() + 1)).finished();

    if (this->_problem.phases(1).guess.controls.rows() < 1)
        this->_problem.phases(1).guess.controls = ::zeros(getNStates(),
                                                            getNSteps()+1);
    if (this->_problem.phases(1).guess.states.rows() < 1)
        this->_problem.phases(1).guess.states = ::zeros(getNControls(),
                                                        getNSteps()+1);
    if (this->_problem.phases(1).guess.time.rows() < 1)
        this->_problem.phases(1).guess.time = ::linspace(0.,
                getNSteps()*getDt(), getNSteps()+1);

    _problem.phases(1).npath = this->_parameters.size();

    ::psopt_level2_setup(_problem, _algorithm);

    _algorithm.nlp_method           = "IPOPT";
    _algorithm.scaling              = "automatic";
    _algorithm.derivatives          = "automatic";
    _algorithm.hessian              = "exact";
    _algorithm.nlp_iter_max         = 200;
    _algorithm.nlp_tolerance        = 1.e-6;
    _algorithm.collocation_method   = "Legendre";
    _algorithm.mesh_refinement      = "automatic";
    _algorithm.mr_max_iterations    = 10;
    _algorithm.ode_tolerance        = 1.e-4;
    _algorithm.print_level          = 0;

    this->addBounds();

    _problem.integrand_cost         = &ePSOPT::integrand_cost;
    _problem.endpoint_cost          = &ePSOPT::endpoint_cost;
    _problem.dae                    = &ePSOPT::dae;
    _problem.events                 = &ePSOPT::events;
    _problem.linkages               = &ePSOPT::linkages;
}

void ePSOPT::solve() {
    ::psopt(_solution, _problem, _algorithm);
    if (_solution.error_flag) {
        std::cout << "!!!!!Problem failed!!!!!" << std::endl
            << _solution.error_msg << std::endl;
    } else {
        this->setScore(this->isMaximized() ? -_solution.cost : _solution.cost);
        getTraj();
    }

    return;
}

/**
 * Enables IPOPT console messages. This feature is disabled by setup().
 * Therefore, call this function after setup(), but before solve()
 */
void ePSOPT::debug() {
    _algorithm.print_level = 5;
}

/**
 * Not used
 */
void ePSOPT::close() {}

// Getters and Setters

Prob* ePSOPT::getProblem() {
    return &_problem;
}

Alg* ePSOPT::getAlgorithm() {
    return &_algorithm;
}

Sol* ePSOPT::getSolution() {
    return &_solution;
}

// Private functions

void ePSOPT::addBounds() {
    paramset_t::iterator param = this->_parameters.begin();
    state_t::iterator xlow(getXlower().begin()), xup(getXupper().begin()),
            ulow(getUlower().begin()), uup(getUupper().begin()),
            x0(getX0().begin()), xf(getXf().begin()), xtol(getXtol().begin());

    size_t offset = this->getNStates();
    for (size_t i= 0; i < this->getNStates(); i++) {
        _problem.phases(1).bounds.lower.states(i) = *(xlow++);
        _problem.phases(1).bounds.upper.states(i) = *(xup++);

        _problem.phases(1).bounds.lower.events(i) = *x0;
        _problem.phases(1).bounds.upper.events(i) = *(x0++);

        _problem.phases(1).bounds.lower.events(i+offset) = *xf - *xtol;
        _problem.phases(1).bounds.upper.events(i+offset) = *(xf++) + *(xtol++);
    }
    for (size_t i= 0; i < this->getNControls(); i++) {
        _problem.phases(1).bounds.lower.controls(i) = *(ulow++);
        _problem.phases(1).bounds.upper.controls(i) = *(uup++);
    }
    for (size_t i= 0; i < this->_parameters.size(); i++) {
        _problem.phases(1).bounds.lower.path(i) = param->second.lbnd;
        _problem.phases(1).bounds.upper.path(i) = (param++)->second.ubnd;
    }
    _problem.phases(1).bounds.lower.StartTime = 0.;
    _problem.phases(1).bounds.upper.StartTime = 0.;
    _problem.phases(1).bounds.lower.EndTime = getNSteps() * getDt();
    _problem.phases(1).bounds.upper.EndTime = getNSteps() * getDt();
}

void ePSOPT::getTraj() {
    MatrixXd tmat         = _solution.get_time_in_phase(1);
    MatrixXd xmat         = _solution.get_states_in_phase(1);
    MatrixXd umat         = _solution.get_controls_in_phase(1);

    traj_t* xtraj = this->getXtraj();
    traj_t* utraj = this->getUtraj();

    xtraj->resize(tmat.cols());
    utraj->resize(tmat.cols());

    std::vector<size_t> t_idx(tmat.cols());
    std::iota(t_idx.begin(), t_idx.end(), 0);
    std::for_each(EXEC_POLICY_UNSEQ, t_idx.begin(), t_idx.end(),
            [&tmat, &xmat, &umat, &xtraj, &utraj](const size_t &j) {
        double t = tmat(0, j);
        state_t x(xmat.rows()), u(umat.rows());

        std::vector<size_t> x_idx(xmat.rows());
        std::iota(x_idx.begin(), x_idx.end(), 0);
        std::transform(EXEC_POLICY_UNSEQ, x_idx.cbegin(), x_idx.cend(),
                x.begin(), [&xmat, &j](const size_t &i){
            return xmat(i, j);
        });

        std::vector<size_t> u_idx(umat.rows());
        std::iota(u_idx.begin(), u_idx.end(), 0);
        std::transform(EXEC_POLICY_UNSEQ, u_idx.cbegin(), u_idx.cend(),
                u.begin(), [&umat, &j](const size_t &i){
            return umat(i, j);
        });

        xtraj->at(j) = traj_elem_t(t, x);
        utraj->at(j) = traj_elem_t(t, u);
    });
}

// PSOPT required functions !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

adouble ePSOPT::integrand_cost(adouble* states, adouble* controls,
                        adouble* parameters, adouble& time, adouble* xad,
                        int iphase, Workspace* workspace) {
    adouble f_val;
    scalar_t fout;

    ePSOPT* ptr = reinterpret_cast<ePSOPT *>(workspace->problem->user_data);

    vector_t x(ptr->getNStates());
    std::vector<size_t> x_idx(ptr->getNStates());
    std::iota(x_idx.begin(), x_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, x_idx.cbegin(), x_idx.cend(), x.begin(),
            [&states](const size_t &i) -> scalar_t {
        return(&states[i]);
    });

    vector_t u(ptr->getNControls());
    std::vector<size_t> u_idx(ptr->getNControls());
    std::iota(u_idx.begin(), u_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, u_idx.cbegin(), u_idx.cend(), u.begin(),
            [&controls](const size_t &i) -> scalar_t {
        return(&controls[i]);
    });

    vector_t params = {std::string()};
    std::vector<std::string> pnames = {std::string("")};
    fout = (*ptr->_objective)(x, u, params, pnames, time, ptr->getDt());
    try {
        f_val = std::any_cast<adouble>(fout);
    } catch(std::bad_any_cast& e) {
        ptr->_eAny = &e;
        cout << "Error in PSOPT Integrand Cost" << std::endl;
        ptr->errorHandler();
    }

    if (ptr->isMaximized())
        f_val = -1.0 * f_val;

    return f_val;
}

void ePSOPT::dae(adouble* derivatives, adouble* path, adouble* states,
             adouble* controls, adouble* parameters, adouble& t,
             adouble* xad, int iphase, Workspace* workspace) {
    ePSOPT* ptr = reinterpret_cast<ePSOPT *>(workspace->problem->user_data);
    adouble *tval  = &t;
    std::vector<std::string> pnames = {std::string("")};
    vector_t params = {std::string()};

    vector_t x(ptr->getNStates());
    std::vector<size_t> x_idx(ptr->getNStates());
    std::iota(x_idx.begin(), x_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, x_idx.cbegin(), x_idx.cend(), x.begin(),
            [&states](const size_t &i) -> scalar_t {
        return(&states[i]);
    });

    vector_t u(ptr->getNControls());
    std::vector<size_t> u_idx(ptr->getNControls());
    std::iota(u_idx.begin(), u_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, u_idx.cbegin(), u_idx.cend(), u.begin(),
            [&controls](const size_t &i) -> scalar_t {
        return(&controls[i]);
    });

    for (size_t i(1); i < ptr->getXrhorizon(); i++) {
        double delay = ptr->getDt()*i;
        for (int j(0); j < ptr->getNStates(); j++) {
            adouble state_prev[1];
            get_delayed_state(state_prev, j, iphase, t, delay, xad, workspace);
            x.push_back(state_prev);
        }
    }

    for (size_t i(1); i <= ptr->getUrhorizon(); i++) {
        double delay = ptr->getDt()*i;
        for (int j(1); j <= ptr->getNControls(); j++) {
            adouble control_prev[1];
            get_delayed_control(control_prev, j, iphase, t, delay, xad,
                                    workspace);
            u.push_back(control_prev);
        }
    }

    std::vector<size_t> p_idx(ptr->_constraints.size());
    std::iota(p_idx.begin(), p_idx.end(), 0);
    size_t j(0);

    try {
        // Must be sequential for ADOLC
        std::for_each(std::execution::seq, x_idx.cbegin(), x_idx.cend(),
                [&derivatives, &ptr, &tval, &x, &u, &pnames, &params](
                        const size_t &i) {
            derivatives[i] = std::any_cast<adouble>(
                    (**(ptr->_gradient.begin() + i))(
                            x, u, params, pnames, tval, ptr->getDt()));
        });
        // Must be sequential for ADOLC
        std::for_each(std::execution::seq, p_idx.cbegin(), p_idx.cend(),
                [&ptr, &path, &tval, &x, &u, &pnames, &params, &j](
                        const auto &i) {
            fout_psopt_t fout = std::any_cast<fout_psopt_t>(
                    (**(ptr->_constraints.begin() + i))(
                            x, u, params, pnames, tval, ptr->getDt()));
            for (auto &val : fout)
                path[j++] = val;
        });
    } catch(std::bad_any_cast& e) {
        ptr->_eAny = &e;
        cout << "Error in PSOPT DAE" << std::endl;
        ptr->errorHandler();
    }
}

/**
 * Sets the initial state and final state constraints
 */
void ePSOPT::events(adouble* e, adouble* initial_states,
        adouble* final_states, adouble* parameters, adouble& t0, adouble& tf,
        adouble* xad, int iphase, Workspace* workspace) {
    size_t i(0), j(0);
    ePSOPT* ptr = reinterpret_cast<ePSOPT *>(workspace->problem->user_data);
    for (; i < ptr->getNStates(); i++)
        e[ i ] = initial_states[i];
    for (; i < 2 * ptr->getNStates(); i++)
        e[ i ] = final_states[j++];
}

/**
 * Does nothing because this PSOPT interface creates a single phase problem
 */
void ePSOPT::linkages(adouble* linkages, adouble* xad,
        Workspace* workspace) {}

/**
 * Always returns 0 because Lagrange form of the optimal control problem is used
 */
adouble ePSOPT::endpoint_cost(adouble* initial_states, adouble* final_states,
                          adouble* parameters, adouble& t0, adouble& tf,
                          adouble* xad, int iphase, Workspace* workspace) {
    return 0.0;
}

} /* namespace ETOL */
