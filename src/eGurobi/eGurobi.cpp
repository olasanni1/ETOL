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

#include <iostream>
#include <cassert>
#include <ETOL/eGurobi.hpp>

namespace ETOL {
// Constructors

eGurobi::eGurobi()
    : eGurobi::TrajectoryOptimizer(), env_(GRBEnv()), model_(GRBModel(env_)),
      _nConstr(ATOMIC_VAR_INIT(0)), _eGRB(NULL), x0_changed_(true),
      xf_changed_(true), reset_(true) {
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

std::string eGurobi::getParamName(const std::string name,
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
        if (reset_) {
            this->model_.reset();
            this->_nConstr = 0;
            this->createVars();
            this->addX0();
            this->addXf();
            this->addDyn();
            this->addConstr();
            this->addObj();
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
    vector_t x;
    vector_t u;
    for (size_t j(0); j < this->_nStates; j++) {
        GRBVar var = (*this->_model).getVarByName(getStateName(tIdx, j));
        x.push_back(var);
    }
    for (size_t j(0); j < this->_nControls; j++) {
        GRBVar var = (*this->_model).getVarByName(getControlName(tIdx, j));
        u.push_back(var);
    }

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
    std::any f_val;
    GRBLinExpr expr;
    vector_t x;
    vector_t u;
    for (size_t k(0); k <= this->getXrhorizon(); k++) {
        for (size_t j(0); j < this->_nStates; j++) {
            size_t i = tIdx-k;
            GRBVar var = (*this->_model).getVarByName(getStateName(i, j));
            x.push_back(var);
        }
    }
    for (size_t k(0); k <= this->getUrhorizon(); k ++) {
        for (size_t j(0) ; j < this->_nControls; j++) {
            size_t i = tIdx-k;
            GRBVar var = (*this->_model).getVarByName(getControlName(i, j));
            u.push_back(var);
        }
    }

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
    vector_t x;
    vector_t u;
    for (size_t j(0); j < this->_nStates; j++) {
        GRBVar var = (*this->_model).getVarByName(getStateName(tIdx, j));
        x.push_back(var);
    }
    for (size_t j(0); j < this->_nControls; j++) {
        GRBVar var = (*this->_model).getVarByName(getControlName(tIdx, j));
        u.push_back(var);
    }

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
            for (auto param : this->_parmaeters) {
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
        this->_x0_constraint_names.clear();
        for (size_t i(0); i < this->getRhorizon(); i++) {
            for (size_t j(0); j < this->getNStates(); j++) {
                if (!std::isnan(this->getX0().at(j))) {
                    this->_x0_constraint_names.push_back(
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                    GRBVar var = this->_model->getVarByName(getStateName(i, j));
                    GRBVar* varptr = &var;
                    expr.addTerms(&coeff, varptr, 1);
                    this->_model->addConstr(
                            expr, GRB_EQUAL, this->getX0().at(j),
                            this->_x0_constraint_names.back());
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
    this->_xf_upper_constraint_names.clear();
    this->_xf_lower_constraint_names.clear();
    try {
        for (size_t j(0); j < this->getNStates(); j++) {
            if (!std::isnan(this->getXf().at(j))) {
                this->_xf_upper_constraint_names.push_back(
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                this->_xf_lower_constraint_names.push_back(
                            "c" + std::to_string((size_t)(this->_nConstr++)));
                GRBVar var = this->_model->getVarByName(
                        getStateName(i, j));
                expr.addTerms(&coeff, &var, 1);
                this->_model->addConstr(expr, GRB_LESS_EQUAL,
                        this->getXf().at(j) + this->getXtol().at(j),
                        this->_xf_upper_constraint_names.back());
                this->_model->addConstr(expr, GRB_GREATER_EQUAL,
                        this->getXf().at(j) - this->getXtol().at(j),
                        this->_xf_lower_constraint_names.back());
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
        GRBLinExpr expr = this->_objfunc(0);
        for (size_t i(1); i <= this->getNSteps(); i++)
            expr += this->_objfunc(i);

        if (this->isMaximized())
            this->_model->setObjective(expr, GRB_MAXIMIZE);
        else
            this->_model->setObjective(expr, GRB_MINIMIZE);

        this->_model->update();
    } catch (GRBException& ex) {
        this->_eGRB = &ex;
        errorHandler();
    }
}

void eGurobi::getTraj() {
    try    {
        traj_t* xtraj = this->getXtraj();
        xtraj->clear();
        for (size_t i(0); i <= this->getNSteps(); i++) {
            state_t x;
            double t = static_cast<double>(i) * this->getDt();
            for (size_t j(0); j < this->getNStates(); j++) {
                GRBVar var = this->_model->getVarByName(getStateName(i, j));
                x.push_back(var.get(GRB_DoubleAttr_X));
            }
            xtraj->push_back(traj_elem_t(t, x));
        }

        traj_t* utraj = this->getUtraj();
        utraj->clear();
        for (size_t i(0); i <= this->getNSteps(); i++) {
            state_t u;
            double t = static_cast<double>(i) * this->getDt();
            for (size_t j(0); j < this->getNControls(); j++) {
                GRBVar var = this->_model->getVarByName(getControlName(i, j));
                u.push_back(var.get(GRB_DoubleAttr_X));
            }
            utraj->push_back(traj_elem_t(t, u));
        }
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
            this->model_.getConstrByName(name).set(GRB_DoubleAttr_RHS, *(it++));
            this->model_.update();
        });
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
        this->model_.getConstrByName(name).set(GRB_DoubleAttr_RHS,
                *(it_xf++) + *(it_xtol++));
        this->model_.update();
    });
    it_xf = this->getXf().begin();
    it_xtol = this->getXtol().begin();
    for_each(_xf_lower_constraint_names.cbegin(),
            _xf_lower_constraint_names.cend(),
        [this, &it_xf, &it_xtol](const std::string& name) {
        this->model_.getConstrByName(name).set(GRB_DoubleAttr_RHS,
                *(it_xf++) - *(it_xtol++));
        this->model_.update();
    });
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


}  /* namespace ETOL */
