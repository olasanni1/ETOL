/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 20 April 2020
 * @version 1.0.0
 * @brief A Gurobi interface
 * @section DESCRIPTION
 * A Gurobi implementation for the pure virtual methods in the
 * TrajectoryOptimizer class. It casts the trajectory optimization problem as
 * a Mixed-Integer Linear Programming (MILP) problem.
 ******************************************************************************/

#include <map>
#include <memory>
#include <iostream>
#include <cassert>
#include <execution>
#include <algorithm>
#include <numeric>
#include <string>
#include <ETOL/eGurobi.hpp>

namespace ETOL {
// Constructors

#ifdef PSTL_USE_PARALLEL_POLICIES
constexpr std::execution::parallel_policy EXEC_POLICY_SEQ{};
constexpr std::execution::parallel_unsequenced_policy EXEC_POLICY_UNSEQ{};
#else
constexpr std::execution::sequenced_policy EXEC_POLICY_SEQ{};
constexpr std::execution::unsequenced_policy EXEC_POLICY_UNSEQ{};
#endif

eGurobi::eGurobi()
    : eGurobi::TrajectoryOptimizer(), env_(NULL), model_(NULL),
      _nConstr(ATOMIC_VAR_INIT(0)), _eGRB(NULL), x0_changed_(true),
      xf_changed_(true), reset_(true) {
    try {
        this->env_.reset(new GRBEnv());
        this->model_.reset(new GRBModel(*env_));
    } catch(GRBException& e) {
        this->_eGRB = &e;
        this->errorHandler();
    }
}

// Virtual Members

eGurobi::~eGurobi() {}

// Static members

std::string eGurobi::getStateName(const size_t& tIdx, const size_t& sIdx) {
    return("x_" + std::to_string(tIdx) + "_" + std::to_string(sIdx));
}

std::string eGurobi::getControlName(const size_t& tIdx, const size_t& sIdx) {
    return("u_" + std::to_string(tIdx) + "_" + std::to_string(sIdx));
}

std::string eGurobi::getParamName(const std::string& name,
        const size_t& kIdx) {
    return(name + "_" + std::to_string(kIdx));
}

char eGurobi::getGRBType(var_t varType) {
    switch (varType) {
        case var_t::CONTINUOUS:
            return GRB_CONTINUOUS;
        case var_t::INTERGER:
            return GRB_INTEGER;
        case var_t::BINARY:
            return GRB_BINARY;
        default :
            assert(true);  // Type not implemented
    }
    return GRB_CONTINUOUS;
}

// Getters and Setters

const size_t eGurobi::getNConstr() const {
    return _nConstr;
}

// API

void eGurobi::setup() {
    // Vector size must match number of states
    // Fill unused vector entries with nan
    try {
        if (reset_) {
            this->model_.reset(new GRBModel(*env_));
            this->_nConstr = 0;
            this->createVars();
            this->addX0();
            this->addXf();
            this->addDyn();
            this->addConstr();
            this->addObj();
            this->model_->set(GRB_IntParam_Method, method_);
            this->model_->set(GRB_IntParam_OutputFlag, 0);
            this->model_->set(GRB_IntParam_UpdateMode, 0);
        } else {
            if (x0_changed_) {
                this->changeX0();
            }
            if (xf_changed_) {
                this->changeXf();
            }
        }
        reset_ = x0_changed_ = xf_changed_ = false;
    } catch(GRBException& e) {
        this->_eGRB = &e;
        this->errorHandler();
    }
}

void eGurobi::solve() {
    try {
        this->model_->optimize();
        if (this->model_->get(GRB_IntAttr_SolCount) > 0) {
            setScore(model_->get(::GRB_DoubleAttr_ObjVal));
            this->model_->update();
            this->getTraj();
        }
    } catch(GRBException& e) {
        this->_eGRB = &e;
        this->errorHandler();
    }
}

void eGurobi::debug() {
    this->model_->write("debug.lp");
    this->model_->set(GRB_IntParam_OutputFlag, 1);
    std::cout << "Debug information written to 'debug.lp'" << std::endl;
}

void eGurobi::close() {}

// Protected functions

GRBLinExpr eGurobi::_objfunc(const size_t &tIdx) {
    std::any f_val;
    GRBLinExpr expr;
    vector_t x(getNStates());
    vector_t u(getNControls());
    std::vector<size_t> x_idx(getNStates());
    std::vector<size_t> u_idx(getNControls());

    std::iota(x_idx.begin(), x_idx.end(), 0);

    std::transform(EXEC_POLICY_UNSEQ,
            x_idx.begin(), x_idx.end(), x.begin(),
       [this, &tIdx](const size_t& j) -> scalar_t {
       return this->model_->getVarByName(getStateName(tIdx, j));
    });

    std::iota(u_idx.begin(), u_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, u_idx.begin(), u_idx.end(), u.begin(),
       [this, &tIdx](const size_t& j) -> scalar_t {
       return this->model_->getVarByName(getControlName(tIdx, j));
    });

    f_val = (*this->_objective)(
            x, u, this->_params, this->_pnames, tIdx, this->getDt());
    try {
        expr = std::any_cast<GRBLinExpr>(f_val);
    } catch(std::bad_any_cast& e) {
        this->_eAny = &e;
        errorHandler();
    }
    return expr;
}

GRBLinExpr eGurobi::_dynConstr(const size_t &tIdx, const size_t &sIdx) {
    assert(tIdx >= getXrhorizon());
    assert(tIdx >= getUrhorizon());
    std::any f_val;
    GRBLinExpr expr;
    std::vector<size_t> x_idx(getNStates() * (getXrhorizon()+1));
    std::vector<size_t> u_idx(getNControls() * (getUrhorizon()+1));
    vector_t x(getNStates() * (getXrhorizon()+1));
    vector_t u(getNControls() * (getUrhorizon()+1));

    std::iota(x_idx.begin(), x_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, x_idx.begin(), x_idx.end(), x.begin(),
            [this, &tIdx](const size_t& j) -> scalar_t {
       return this->model_->getVarByName(getStateName(
               tIdx - j / this->getNStates(), j % this->getNStates()));
    });

    std::iota(u_idx.begin(), u_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, u_idx.begin(), u_idx.end(), u.begin(),
            [this, &tIdx](const size_t& j) -> scalar_t {
       return this->model_->getVarByName(getControlName(
               tIdx - j / this->getNControls(), j % this->getNControls()));
    });

    f_t* f = _gradient.at(sIdx);
    f_val = (*f)(x, u, this->_params, this->_pnames, tIdx, this->getDt());
    try {
        expr = std::any_cast<GRBLinExpr>(f_val);
    } catch(std::bad_any_cast& e) {
        this->_eAny = &e;
        errorHandler();
    }
    return expr;
}

fout_grb_t eGurobi::_pathConstr(const size_t &tIdx, const size_t &kIdx) {
    std::any p_val;
    fout_grb_t fout;
    vector_t x(getNStates());
    vector_t u(getNControls());
    std::vector<size_t> x_idx(getNStates());
    std::vector<size_t> u_idx(getNControls());

    std::iota(x_idx.begin(), x_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, x_idx.begin(), x_idx.end(), x.begin(),
       [this, &tIdx](const size_t& j) -> scalar_t {
       return this->model_->getVarByName(getStateName(tIdx, j));
    });

    std::iota(u_idx.begin(), u_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, u_idx.begin(), u_idx.end(), u.begin(),
       [this, &tIdx](const size_t& j) -> scalar_t {
       return this->model_->getVarByName(getControlName(tIdx, j));
    });

    f_t* p = this->_constraints.at(kIdx);
    p_val = (*p)(x, u, this->_params, this->_pnames, tIdx, this->getDt());
    try {
        fout = std::any_cast<fout_grb_t>(p_val);
    } catch(std::bad_any_cast& e) {
        this->_eAny = &e;
        errorHandler();
    }
    return fout;
}

/**
 * Prints a Gurobi error message to the console
 */
void eGurobi::errorHandler() {
    TrajectoryOptimizer::errorHandler();
    if (this->_eGRB != NULL) {
        std::cerr << "Gurobi Error: " << this->_eGRB->getErrorCode()
                << std::endl;
        std::cerr << this->_eGRB->getMessage() << std::endl;
        assert(false);
    }
}

// Private functions
/**
 * State variables names are x_time#_state#. Control variable names are
 * u_time#_state#. Path constraint variable names are varName_time#
 *
 * These loops must be sequential to avoid data race condition and to align
 * params and pnames indices
 */
void eGurobi::createVars() {
    state_t::iterator l_it, u_it;
    state_var_t::iterator vt_it;
    try {
        for (size_t k(0); k <= this->getNSteps(); k++) {
            // x_time#_state#
            l_it = this->getXlower().begin();
            u_it = this->getXupper().begin();
            vt_it = this->getXvartype().begin();
            for (size_t j(0); j < this->getNStates(); j++) {
                this->model_->addVar(*(l_it++), *(u_it++), 0.0,
                        getGRBType(*(vt_it++)), getStateName(k, j));
            }

            // u_time#_state#
            l_it = this->getUlower().begin();
            u_it = this->getUupper().begin();
            vt_it = this->getUvartype().begin();
            for (size_t j(0); j < this->getNControls(); j++) {
                this->model_->addVar(*(l_it++), *(u_it++), 0.0,
                        getGRBType(*(vt_it++)), getControlName(k, j));
            }

            // varName_time#
            double t = static_cast<double>(k) * this->getDt();
            for (auto param : this->_parameters) {
                if ((t >= param.second.tStart) && (t <= param.second.tStop)) {
                    this->model_->addVar(
                            param.second.lbnd, param.second.ubnd, 0.0,
                            getGRBType(param.second.varType),
                            getParamName(param.first, k));
                }
            }
            std::function<GRBVar(const std::string&)> getVar_cb =
                    [this](const std::string &varname) -> GRBVar {
                return this->model_->getVarByName(varname);;
            };
            getVar_ = std::make_shared<std::function<GRBVar(
                    const std::string &)>>(getVar_cb);
            this->_params.push_back(getVar_.get());
            this->_pnames.push_back("getVar");
        }
        this->model_->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::addX0() {
    try {
        GRBLinExpr expr = GRBLinExpr();
        double coeff = 1.0;
        this->_x0_constraint_names.clear();
        for (size_t i(0); i < this->getRhorizon(); i++) {
            for (size_t j(0); j < this->getNStates(); j++) {
                if (!std::isnan(this->getX0().at(j))) {
                    this->_x0_constraint_names.push_back(
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                    GRBVar var = this->model_->getVarByName(getStateName(i, j));
                    GRBVar* varptr = &var;
                    expr.addTerms(&coeff, varptr, 1);
                    this->model_->addConstr(
                            expr, GRB_EQUAL, this->getX0().at(j),
                            this->_x0_constraint_names.back());
                    expr.clear();
                }
            }
        }
        this->model_->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::addXf() {
    GRBLinExpr expr = GRBLinExpr();
    double coeff = 1.0;
    size_t i = this->getNSteps();
    std::string n(std::to_string(this->getNSteps()) + "_");
    this->_xf_upper_constraint_names.clear();
    this->_xf_lower_constraint_names.clear();
    try {
        for (size_t j(0); j < this->getNStates(); j++) {
            if (!std::isnan(this->getXf().at(j))) {
                this->_xf_upper_constraint_names.push_back(
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                this->_xf_lower_constraint_names.push_back(
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                GRBVar var = this->model_->getVarByName(
                        getStateName(i, j));
                expr.addTerms(&coeff, &var, 1);
                this->model_->addConstr(expr, GRB_LESS_EQUAL,
                        this->getXf().at(j) + this->getXtol().at(j),
                        this->_xf_upper_constraint_names.back());
                this->model_->addConstr(expr, GRB_GREATER_EQUAL,
                        this->getXf().at(j) - this->getXtol().at(j),
                        this->_xf_lower_constraint_names.back());
                expr.clear();
            }
        }
        this->model_->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::addConstr() {
    try {
        for (size_t k(0); k < this->_constraints.size(); k++) {
            for (size_t i(0); i <= this->getNSteps(); i++) {
                fout_grb_t fout = this->_pathConstr(i, k);
                for (auto grb : fout) {
                    this->model_->addConstr(grb.expr, grb.sense, grb.rhs,
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                }
            }
        }
        this->model_->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::addDyn() {
    try {
        for (size_t i = this->getRhorizon(); i <= this->getNSteps(); i++) {
            for (size_t j(0); j < this->getNStates(); j++)
                this->model_->addConstr(this->_dynConstr(i, j), GRB_EQUAL, 0.0,
                        "c" + std::to_string((size_t)(this->_nConstr++)));
        }
        this->model_->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::addObj() {
    try {
        std::vector<size_t> t_idx(getNSteps() + 1);

        std::iota(t_idx.begin(), t_idx.end(), 0);

        std::vector<GRBLinExpr> linExpr(getNSteps() + 1);

        std::transform(EXEC_POLICY_UNSEQ, t_idx.begin(), t_idx.end(),
                linExpr.begin(), [this](const size_t& i) -> GRBLinExpr {
            return(this->_objfunc(i));
        });

        GRBLinExpr sum_expr = GRBLinExpr();
        for_each(linExpr.begin(), linExpr.end(),
            [&sum_expr](const auto &expr) {
            sum_expr += expr;
        });

        if (this->isMaximized())
            this->model_->setObjective(sum_expr, GRB_MAXIMIZE);
        else
            this->model_->setObjective(sum_expr, GRB_MINIMIZE);

        this->model_->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::getTraj() {
    try {
        std::vector<size_t> t_idx(getNSteps()+1);
        std::vector<size_t> x_idx(getNStates());
        std::vector<size_t> u_idx(getNControls());

        std::iota(t_idx.begin(), t_idx.end(), 0);
        std::iota(x_idx.begin(), x_idx.end(), 0);
        std::iota(u_idx.begin(), u_idx.end(), 0);

        auto x_cb = [this](const auto &i, const auto &j) -> double {
            try {
                GRBVar var = this->model_->getVarByName(getStateName(i, j));
                double val = var.get(GRB_DoubleAttr_X);
                return val;
            } catch (GRBException& ex) {
                std::cerr << "Gurobi Error: " << ex.getErrorCode()
                                << std::endl;
                std::cerr << ex.getMessage() << std::endl;
                std::cerr << "Varname: " << getStateName(i, j) << std::endl;
            }
            return std::numeric_limits<double>::infinity();
        };
        auto u_cb = [this](const auto &i, const auto &j) -> double {
            try {
                GRBVar var = this->model_->getVarByName(getControlName(i, j));
                double val = var.get(GRB_DoubleAttr_X);
                return val;
            } catch (GRBException& ex) {
                std::cerr << "Gurobi Error: " << ex.getErrorCode()
                                << std::endl;
                std::cerr << ex.getMessage() << std::endl;
                std::cerr << "Varname: " << getControlName(i, j)<< std::endl;
            }
            return std::numeric_limits<double>::infinity();
        };

        traj_t* xtraj = this->getXtraj();
        xtraj->resize(getNSteps()+1);

        // Occasional error if outer loop is parallel
        std::transform(t_idx.begin(), t_idx.end(),
                xtraj->begin(), [this, &x_idx, &x_cb](const auto & i) -> traj_elem_t {
            double t = static_cast<double>(i) * this->getDt();
            state_t x(this->getNStates());
            std::transform(EXEC_POLICY_UNSEQ, x_idx.begin(), x_idx.end(),
                    x.begin(), [&i, &x_cb](const auto &j) -> double {
                return x_cb(i, j);
            });
            return (traj_elem_t(t, x));
        });

        traj_t* utraj = this->getUtraj();
        utraj->resize(getNSteps()+1);

        // Occasional error if outer loop is parallel
        std::transform(t_idx.begin(), t_idx.end(),
                utraj->begin(), [this, &u_idx, &u_cb](const auto & i) -> traj_elem_t {
            double t = static_cast<double>(i) * this->getDt();
            state_t u(getNControls());
            std::transform(EXEC_POLICY_UNSEQ, u_idx.begin(), u_idx.end(),
                    u.begin(), [&i, &u_cb](const auto &j) -> double {
                return u_cb(i, j);
            });
            return (traj_elem_t(t, u));
        });
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::changeX0() {
    auto it = this->getX0().begin();
    try {
        for_each(_x0_constraint_names.cbegin(), _x0_constraint_names.cend(),
                [this, &it](const std::string& name) {
            this->model_->getConstrByName(name).set(
                    GRB_DoubleAttr_RHS, *(it++));
        });
        this->model_->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::changeXf() {
    auto it_xf = this->getXf().begin();
    auto it_xtol = this->getXtol().begin();
    for_each(_xf_upper_constraint_names.cbegin(),
            _xf_upper_constraint_names.cend(),
        [this, &it_xf, &it_xtol](const std::string& name) {
        this->model_->getConstrByName(name).set(GRB_DoubleAttr_RHS,
                *(it_xf++) + *(it_xtol++));
    });
    it_xf = this->getXf().begin();
    it_xtol = this->getXtol().begin();
    for_each(_xf_lower_constraint_names.cbegin(),
            _xf_lower_constraint_names.cend(),
        [this, &it_xf, &it_xtol](const std::string& name) {
        this->model_->getConstrByName(name).set(GRB_DoubleAttr_RHS,
                *(it_xf++) - *(it_xtol++));
    });
    this->model_->update();
}

// Overriding Setters

void eGurobi::setDt(double dt) {
    if (this->getDt() != dt) {
        reset_ = true;
        TrajectoryOptimizer::setDt(dt);
    }
}

void eGurobi::setNSteps(const size_t nSteps) {
    if (this->getNSteps() != nSteps) {
        reset_ = true;
        TrajectoryOptimizer::setNSteps(nSteps);
    }
}

void eGurobi::setXvartype(const state_var_t &xvartype) {
    if (!std::equal(this->getXvartype().cbegin(),
            this->getXvartype().cend(), xvartype.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setXvartype(xvartype);
    }
}

void eGurobi::setX0(const state_t& x0) {
    // Skip comparison and reset if array size is different
    if (this->getX0().size() == x0.size()) {
        if (!std::equal(this->getX0().cbegin(), this->getX0().cend(),
                x0.cbegin())) {
            // Only change the xf values in the Gurobi model
            x0_changed_ = true;
            TrajectoryOptimizer::setX0(x0);
        }
        return;
    }
    // Vector size change requires a reset
    TrajectoryOptimizer::setX0(x0);
    reset_ = true;
    return;
}

void eGurobi::setXf(const state_t& xf) {
    // Skip comparison and reset if array size is different
    if (this->getXf().size() == xf.size()) {
        if (!std::equal(this->getXf().cbegin(), this->getXf().cend(),
                xf.cbegin())) {
            // Only change the xf values in the Gurobi model
            xf_changed_ = true;
            TrajectoryOptimizer::setXf(xf);
        }
        return;
    }
    // Vector size change requires a reset
    TrajectoryOptimizer::setXf(xf);
    reset_ = true;
    return;
}

void eGurobi::setXtol(const state_t &xtol) {
    // Skip comparison and reset if array size is different
    if (this->getXtol().size() == xtol.size()) {
        if (!std::equal(this->getXtol().cbegin(), this->getXtol().cend(),
                xtol.cbegin())) {
            // Only change the xf values in the Gurobi model
            xf_changed_ = true;
            TrajectoryOptimizer::setXtol(xtol);
        }
        return;
    }
    // Vector size change requires a reset
    TrajectoryOptimizer::setXtol(xtol);
    reset_ = true;
    return;
}

void eGurobi::setXupper(const state_t &xupper) {
    if (!std::equal(this->getXupper().cbegin(),
            this->getXupper().cend(), xupper.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setXupper(xupper);
    }
}

void eGurobi::setXlower(const state_t &xlower) {
    if (!std::equal(this->getXlower().cbegin(),
            this->getXlower().cend(), xlower.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setXlower(xlower);
    }
}

void eGurobi::setXrhorizon(const size_t nx4dyn) {
    if (this->getXrhorizon() != nx4dyn) {
        reset_ = true;
        TrajectoryOptimizer::setXrhorizon(nx4dyn);
    }
}

void eGurobi::setUvartype(const state_var_t &uvartype) {
    if (!std::equal(this->getUvartype().cbegin(),
            this->getUvartype().cend(), uvartype.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setUvartype(uvartype);
    }
}

void eGurobi::setUupper(const state_t &uupper) {
    if (!std::equal(this->getUupper().cbegin(),
            this->getUupper().cend(), uupper.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setUupper(uupper);
    }
}

void eGurobi::setUlower(const state_t &ulower) {
    if (!std::equal(this->getUlower().cbegin(),
            this->getUlower().cend(), ulower.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setUlower(ulower);
    }
}

void eGurobi::setUrhorizon(const size_t nu4dyn) {
    if (this->getUrhorizon() != nu4dyn) {
        reset_ = true;
        TrajectoryOptimizer::setUrhorizon(nu4dyn);
    }
}

void eGurobi::setObjective(f_t* objective) {
    reset_ = true;
    TrajectoryOptimizer::setObjective(objective);
}

void eGurobi::setGradient(std::vector<f_t*> gradient) {
    reset_ = true;
    TrajectoryOptimizer::setGradient(gradient);
}

void eGurobi::setConstraints(std::vector<f_t*> constraints) {
    reset_ = true;
    TrajectoryOptimizer::setConstraints(constraints);
}

void eGurobi::setMethod(int method) {
    method_ = method;
}

}  /* namespace ETOL */
