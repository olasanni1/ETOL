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

#include <ETOL/eGurobi.hpp>
#include <iostream>
#include <cassert>
#include <execution>
#include <algorithm>
#include <numeric>
#include <string>

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
    : eGurobi::TrajectoryOptimizer(), env_(GRBEnv()), model_(GRBModel(env_)),
      _nConstr(ATOMIC_VAR_INIT(0)), _eGRB(NULL)  {
    this->_env = &env_;
    this->_model = &model_;
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
    assert(this->getX0().size() == this->getNStates());
    assert(this->getXf().size() == this->getNStates());
    assert(this->getXlower().size() == this->getNStates());
    assert(this->getXupper().size() == this->getNStates());
    assert(this->getXvartype().size() == this->getNStates());

    try {
        this->_model->reset();
        this->createVars();
        this->addX0();
        this->addXf();
        this->addDyn();
        this->addConstr();
        this->addObj();
    } catch(GRBException& e) {
        this->_eGRB = &e;
        this->errorHandler();
    }
}

void eGurobi::solve() {
    try {
        this->_model->optimize();
        if (this->_model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            setScore(_model->get(::GRB_DoubleAttr_ObjVal));
            this->getTraj();
        }
    } catch(GRBException& e) {
        this->_eGRB = &e;
        this->errorHandler();
    }
}

void eGurobi::debug() {
    this->_model->write("debug.lp");
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
       return (*this->_model).getVarByName(getStateName(tIdx, j));
    });

    std::iota(u_idx.begin(), u_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, u_idx.begin(), u_idx.end(), u.begin(),
       [this, &tIdx](const size_t& j) -> scalar_t {
       return (*this->_model).getVarByName(getControlName(tIdx, j));
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
       return (*this->_model).getVarByName(getStateName(
               tIdx - j / this->getNStates(), j % this->getNStates()));
    });

    std::iota(u_idx.begin(), u_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, u_idx.begin(), u_idx.end(), u.begin(),
            [this, &tIdx](const size_t& j) -> scalar_t {
       return (*this->_model).getVarByName(getControlName(
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
       return (*this->_model).getVarByName(getStateName(tIdx, j));
    });

    std::iota(u_idx.begin(), u_idx.end(), 0);
    std::transform(EXEC_POLICY_UNSEQ, u_idx.begin(), u_idx.end(), u.begin(),
       [this, &tIdx](const size_t& j) -> scalar_t {
       return (*this->_model).getVarByName(getControlName(tIdx, j));
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
        std::string msg = this->_eGRB->getMessage();
        fprintf(stderr, "%s", msg.c_str());
        assert(true);
        exit(EXIT_FAILURE);
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
                this->_model->addVar(*(l_it++), *(u_it++), 0.0,
                        getGRBType(*(vt_it++)), getStateName(k, j));
            }

            // u_time#_state#
            l_it = this->getUlower().begin();
            u_it = this->getUupper().begin();
            vt_it = this->getUvartype().begin();
            for (size_t j(0); j < this->getNControls(); j++) {
                this->_model->addVar(*(l_it++), *(u_it++), 0.0,
                        getGRBType(*(vt_it++)), getControlName(k, j));
            }

            // varName_time#
            double t = static_cast<double>(k) * this->getDt();
            for (auto param : this->_parameters) {
                if ((t >= param.second.tStart) && (t <= param.second.tStop)) {
                    GRBVar var;
                    var = (this->_model->addVar(
                            param.second.lbnd, param.second.ubnd, 0.0,
                            getGRBType(param.second.varType),
                            getParamName(param.first, k)));
                    this->_params.push_back(var);
                    this->_pnames.push_back(getParamName(param.first, k));
                }
            }
        }
        this->_model->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::addX0() {
    try {
        GRBLinExpr expr = GRBLinExpr();
        double coeff = 1.0;
        for (size_t i(0); i < this->getRhorizon(); i++) {
            for (size_t j(0); j < this->getNStates(); j++) {
                if (!std::isnan(this->getX0().at(j))) {
                    GRBVar var = this->_model->getVarByName(getStateName(i, j));
                    GRBVar* varptr = &var;
                    expr.addTerms(&coeff, varptr, 1);
                    this->_model->addConstr(
                            expr, GRB_EQUAL, this->getX0().at(j),
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                    expr.clear();
                }
            }
        }
        this->_model->update();
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
    try {
        for (size_t j(0); j < this->getNStates(); j++) {
            if (!std::isnan(this->getXf().at(j))) {
                GRBVar var = this->_model->getVarByName(
                        getStateName(i, j));
                expr.addTerms(&coeff, &var, 1);
                this->_model->addConstr(expr, GRB_LESS_EQUAL,
                        this->getXf().at(j) + this->getXtol().at(j),
                        "c" + std::to_string((size_t)(this->_nConstr++)));
                this->_model->addConstr(expr, GRB_GREATER_EQUAL,
                            this->getXf().at(j) - this->getXtol().at(j),
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                expr.clear();
            }
        }
        this->_model->update();
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
                    this->_model->addConstr(grb.expr, grb.sense, grb.rhs,
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                }
            }
        }
        this->_model->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::addDyn() {
    try {
        for (size_t i = this->getRhorizon(); i <= this->getNSteps(); i++) {
            for (size_t j(0); j < this->getNStates(); j++)
                this->_model->addConstr(this->_dynConstr(i, j), GRB_EQUAL, 0.0,
                        "c" + std::to_string((size_t)(this->_nConstr++)));
        }
        this->_model->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::addObj() {
    try {
        std::vector<size_t> t_idx(getNSteps());

        std::iota(t_idx.begin(), t_idx.end(), 0);

        std::vector<GRBLinExpr> linExpr(getNSteps());

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
            this->_model->setObjective(sum_expr, GRB_MAXIMIZE);
        else
            this->_model->setObjective(sum_expr, GRB_MINIMIZE);

        this->_model->update();
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

        traj_t* xtraj = this->getXtraj();
        xtraj->resize(getNSteps()+1);
        std::transform(EXEC_POLICY_UNSEQ, t_idx.begin(), t_idx.end(),
                xtraj->begin(), [this, &x_idx](const auto & i) -> traj_elem_t {
            double t = static_cast<double>(i) * this->getDt();
            state_t x(this->getNStates());
            std::transform(EXEC_POLICY_UNSEQ, x_idx.begin(), x_idx.end(),
                    x.begin(), [this, &i](const auto &j) -> double {
                GRBVar var = this->_model->getVarByName(getStateName(i, j));
                return(var.get(GRB_DoubleAttr_X));
            });
            return (traj_elem_t(t, x));
        });

        traj_t* utraj = this->getUtraj();
        utraj->resize(getNSteps()+1);
        std::transform(EXEC_POLICY_UNSEQ, t_idx.begin(), t_idx.end(),
                utraj->begin(), [this, &u_idx](const auto & i) -> traj_elem_t {
            double t = static_cast<double>(i) * this->getDt();
            state_t u(getNControls());
            std::transform(EXEC_POLICY_UNSEQ, u_idx.begin(), u_idx.end(),
                    u.begin(), [this, &i](const auto &j) -> double {
                GRBVar var = this->_model->getVarByName(getControlName(i, j));
                return(var.get(GRB_DoubleAttr_X));
            });
            return (traj_elem_t(t, u));
        });
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

}  /* namespace ETOL */
