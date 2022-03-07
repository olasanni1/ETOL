/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 16 October 2020
 * @version 1.0.0
 * @brief A SCIP implementation for ETOL
 * @section DESCRIPTION
 * A <a href="https://scipopt.org/#scipoptsuite">SCIP 7.0.1</a> implementation
 * for the pure virtual methods in the TrajectoryOptimization class. It casts
 * the trajectory optimization problem as a Mixed-Integer Linear Programming
 * (MILP) type.
 ******************************************************************************/

#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include <iostream>
#include <numeric>
#include <atomic>
#include <ETOL/eSCIP.hpp>

namespace ETOL {
// Constructor
eSCIP::eSCIP() : eSCIP::TrajectoryOptimizer(),
        scip_(NULL), x_(NULL), u_(NULL), p_(NULL),
        x0_changed_(true), xf_changed_(true), reset_(true), configured_(false),
        nConstr_(ATOMIC_VAR_INIT(0)) {
    SCIP_CALL_ABORT(SCIPcreate(&scip_));
    SCIP_CALL_ABORT(SCIPincludeDefaultPlugins(scip_));
    SCIP_CALL_ABORT(SCIPcreateProbBasic(scip_, "eSCIP") );
}

// Destructor
eSCIP::~eSCIP() {
    close();
    SCIP_CALL_ABORT(SCIPfree(&scip_));
}

// API
void eSCIP::setup() {
    if (reset_ || x0_changed_ || xf_changed_)
        SCIP_CALL_ABORT(SCIPfreeTransform(scip_));

    if (reset_) {
        this->close();
        this->createVars();
        this->configX0();
        this->configXf();
        this->addDyn();
        this->addConstr();
        this->addObj();
    } else {
        if (x0_changed_) {
            this->configX0();
        }
        if (xf_changed_) {
            this->configXf();
        }
    }
    configured_ = true;
    reset_ = x0_changed_ = xf_changed_ = false;
}

void eSCIP::solve() {
    SCIP_CALL_ABORT(SCIPsolve(scip_));
    if (SCIPgetNSols(scip_)) {
        this->setScore(SCIPgetSolOrigObj(scip_, SCIPgetBestSol(scip_)));
        this->getTraj();
    }
}

void eSCIP::debug() {
    SCIP_CALL_ABORT(SCIPwriteOrigProblem(scip_, "debug.lp", "lp", false));
}

void eSCIP::close() {
    if (configured_) {
        releaseVars(x_, getNStates() * (getNSteps() + 1));
        releaseVars(u_, getNControls() * (getNSteps() + 1));
        for (size_t i = 0; i < params_.size(); i++) {
            if (pnames_.at(i) != std::string("getVar")) {
                SCIP_VAR* var = std::any_cast<SCIP_VAR*>(params_[i]);
                SCIP_CALL_ABORT(SCIPreleaseVar(scip_, &var));
            }
        }
        params_.clear();
        pnames_.clear();
        SCIPfreeMemoryArray(scip_, &x_);
        SCIPfreeMemoryArray(scip_, &u_);
        SCIPfreeMemoryArray(scip_, &p_);
        getVar_.reset();
        configured_ = false;
        reset_ = true;
    }
}


std::string eSCIP::getStateName(const size_t& tIdx, const size_t& sIdx) {
    return("x_" + std::to_string(tIdx) + "_" + std::to_string(sIdx));
}

std::string eSCIP::getControlName(const size_t& tIdx, const size_t& sIdx) {
    return("u_" + std::to_string(tIdx) + "_" + std::to_string(sIdx));
}

std::string eSCIP::getParamName(const std::string name, const size_t& tIdx) {
    return(name + "_" + std::to_string(tIdx));
}

std::string eSCIP::getParamName(std::string name, const size_t& tIdx,
        const size_t& sIdx) {
    return(name + "_" + std::to_string(tIdx) + "_" + std::to_string(sIdx));
}

std::string eSCIP::getParamName(std::string name, const size_t& Idx1,
        const size_t& Idx2, const size_t& Idx3) {
    return(name + "_" + std::to_string(Idx1) + "_" + std::to_string(Idx2) +
            "_" + std::to_string(Idx3));
}

SCIP_VARTYPE eSCIP::getSCIPType(var_t varType) {
    switch (varType) {
        case var_t::CONTINUOUS:
            return SCIP_VARTYPE_CONTINUOUS;
        case var_t::INTERGER:
            return SCIP_VARTYPE_INTEGER;
        case var_t::BINARY:
            return SCIP_VARTYPE_BINARY;
        default :
            assert(true);  // Type not implemented
    }
    return SCIP_VARTYPE_CONTINUOUS;
}

// Protected functions

scip_expr_t eSCIP::objfunc(const size_t &tIdx) {
    std::any f_val;

    vector_t xhist = getStates(tIdx);
    vector_t uhist = getControls(tIdx);

    vector_t x(xhist.begin(), xhist.begin() + getNStates());
    vector_t u(uhist.begin(), uhist.begin() + getNControls());

    f_val = (*this->_objective)(x, u, params_, pnames_, tIdx, getDt());
    try {
        return std::any_cast<scip_expr_t>(f_val);
    } catch(std::bad_any_cast& e) {
        this->_eAny = &e;
        errorHandler();
    }
    return scip_expr_t();
}

scip_expr_t eSCIP::dynConstr(const size_t &tIdx, const size_t &sIdx) {
    std::any f_val;

    vector_t x = getStates(tIdx);
    vector_t u = getControls(tIdx);

    f_t* f = _gradient.at(sIdx);
    f_val = (*f)(x, u, params_, pnames_, tIdx, getDt());
    try {
        return std::any_cast<scip_expr_t>(f_val);
    } catch(std::bad_any_cast& e) {
        this->_eAny = &e;
        errorHandler();
    }
    return scip_expr_t();
}

fout_scip_t eSCIP::pathConstr(const size_t &tIdx, const size_t &kIdx) {
    std::any p_val;

    vector_t xhist = getStates(tIdx);
    vector_t uhist = getControls(tIdx);

    vector_t x(xhist.begin(), xhist.begin() + getNStates());
    vector_t u(uhist.begin(), uhist.begin() + getNControls());

    f_t* p = this->_constraints.at(kIdx);
    p_val = (*p)(x, u, params_, pnames_, tIdx, getDt());
    try {
        return std::any_cast<fout_scip_t>(p_val);
    } catch(std::bad_any_cast& e) {
        this->_eAny = &e;
        errorHandler();
    }
    return fout_scip_t();
}

SCIP_VAR* eSCIP::getVarByName(const std::string& name) {
    return SCIPfindVar(scip_, name.c_str());
}

// private members

void eSCIP::createVars() {
    state_t::iterator lo_it, up_it;
    state_var_t::iterator vt_it;
    SCIP_CALL_ABORT(SCIPallocMemoryArray(scip_, &x_, getNStates() *
            (getNSteps() + 1)));
    SCIP_CALL_ABORT(SCIPallocMemoryArray(scip_, &u_, getNControls() *
            (getNSteps() + 1)));
    SCIP_CALL_ABORT(SCIPallocMemoryArray(scip_, &p_, _parameters.size() *
            (getNSteps() + 1)));

    size_t xidx, uidx, pidx;
    xidx = uidx = pidx = 0;
    for (size_t i = 0; i <= this->getNSteps(); i++) {
        // x_time#_state#
        lo_it = this->getXlower().begin();
        up_it = this->getXupper().begin();
        vt_it = this->getXvartype().begin();

        for (size_t j = 0; j < this->getNStates(); j++) {
            SCIP_CALL_ABORT(SCIPcreateVarBasic(scip_, &x_[xidx],
                    getStateName(i, j).c_str(), *(lo_it++), *(up_it++), 0.0,
                    getSCIPType(*(vt_it++))));
            SCIP_CALL_ABORT(SCIPaddVar(scip_, x_[xidx++]));
        }
        // u_time#_state#
        lo_it = this->getUlower().begin();
        up_it = this->getUupper().begin();
        vt_it = this->getUvartype().begin();
        for (int j = 0; j < this->getNControls(); j++) {  // ignore u_0
            SCIP_CALL_ABORT(SCIPcreateVarBasic(scip_, &u_[uidx],
                    getControlName(i, j).c_str(), *(lo_it++), *(up_it++), 0.0,
                    getSCIPType(*(vt_it++))));
            SCIP_CALL_ABORT(SCIPaddVar(scip_, u_[uidx++]));
        }
        // varName_time#
        double t = i * this->getDt();
        for (const auto& param : this->_parameters) {
            if ((t >= param.second.tStart) && (t <= param.second.tStop)) {
                SCIP_CALL_ABORT(SCIPcreateVarBasic(scip_, &p_[pidx],
                        getParamName(param.first, i).c_str(),
                        param.second.lbnd, param.second.ubnd, 0.0,
                        getSCIPType(param.second.varType)));
                SCIP_CALL_ABORT(SCIPaddVar(scip_, p_[pidx]));
                params_.push_back(p_[pidx++]);
                pnames_.push_back(std::string(getParamName(param.first, i)));
            }
        }

        if (!getVar_) {
            getVar_ = std::make_shared<std::function<SCIP_Var*(
                    const std::string&)>>([this](const std::string &varname)
                            -> SCIP_VAR* {
                return this->getVarByName(varname);
            });
        }
        params_.push_back(getVar_.get());
        pnames_.push_back(std::string("getVar"));
    }
}

/**
 * Set X0 by setting lower and upper bound to the same value
 *
 */
void eSCIP::configX0() {
    size_t j = 0;
    for (auto &x0 : this->getX0()) {
        SCIP_VAR* var = getVarByName(getStateName(0, j++));
        SCIP_CALL_ABORT(SCIPchgVarLb(scip_, var, x0));
        SCIP_CALL_ABORT(SCIPchgVarUb(scip_, var, x0));
    }
}

void eSCIP::configXf() {
    size_t j = 0;
    auto it_tol = this->getXtol().cbegin();
    for (auto &xf : this->getXf()) {
        SCIP_VAR* var = getVarByName(getStateName(getNSteps(), j++));
        SCIP_CALL_ABORT(SCIPchgVarLb(scip_, var, xf - *it_tol));
        SCIP_CALL_ABORT(SCIPchgVarUb(scip_, var, xf + *it_tol++));
    }
}

void eSCIP::addConstr() {
    SCIP_CONS* cons;
    for (size_t k(0); k < this->_constraints.size(); k++) {
        for (size_t i(0); i <= this->getNSteps(); i++) {
            fout_scip_t fout = this->pathConstr(i, k);
            for (scip_t scip : fout) {
                SCIP_CALL_ABORT(SCIPcreateConsBasicLinear(scip_, &cons,
                        std::string("c" + std::to_string(
                                (size_t)(this->nConstr_++))).c_str(),
                                0, NULL, NULL,
                    scip.lhs == -ESCIP_INF ? -SCIPinfinity(scip_) : scip.lhs,
                    scip.rhs == ESCIP_INF ? SCIPinfinity(scip_) : scip.rhs));
                for (auto &var : scip.expr) {
                    SCIP_CALL_ABORT(SCIPaddCoefLinear(scip_, cons, var.first,
                            var.second));
                }
                SCIP_CALL_ABORT(SCIPaddCons(scip_, cons));
                SCIP_CALL_ABORT(SCIPreleaseCons(scip_, &cons));
            }
        }
    }
}

void eSCIP::addDyn() {
    SCIP_CONS* cons;
    for (size_t i = this->getRhorizon(); i <= this->getNSteps(); i++) {
        for (size_t j(0); j < this->getNStates(); j++) {
            scip_expr_t expr = dynConstr(i, j);
            SCIP_CALL_ABORT(SCIPcreateConsBasicLinear(scip_, &cons,
                    std::string("c" +
                            std::to_string((size_t)(this->nConstr_++))).c_str(),
                            0, NULL, NULL, 0., 0.));
            for (auto &var : expr) {
                SCIP_CALL_ABORT(SCIPaddCoefLinear(scip_, cons, var.first,
                        var.second));
            }
            SCIP_CALL_ABORT(SCIPaddCons(scip_, cons));
            SCIP_CALL_ABORT(SCIPreleaseCons(scip_, &cons));
        }
    }
}

void eSCIP::addObj() {
    for (size_t i(1); i <= this->getNSteps(); i++) {
        scip_expr_t expr = objfunc(i);
        for (scip_var_t &var : expr) {
            SCIP_CALL_ABORT(SCIPchgVarObj(scip_, var.first, var.second));
        }
    }

    if (this->isMaximized())
        SCIP_CALL_ABORT(SCIPsetObjsense(scip_, SCIP_OBJSENSE_MAXIMIZE));
    else
        SCIP_CALL_ABORT(SCIPsetObjsense(scip_, SCIP_OBJSENSE_MINIMIZE));
}

void eSCIP::getTraj() {
    try {
        SCIP_Var* var;
        SCIP_SOL* sol = SCIPgetBestSol(scip_);
        traj_t* xtraj = this->getXtraj();
        xtraj->clear();
        for (int i = 0; i <= this->getNSteps(); i++) {
            state_t x_sol;
            double t = i * this->getDt();
            for (int j = 0; j < this->getNStates(); j++) {
                var = this->getVarByName(this->getStateName(i, j));
                x_sol.push_back(SCIPgetSolVal(scip_, sol, var));
            }
            xtraj->push_back(traj_elem_t(t, x_sol));
        }

        traj_t* utraj = this->getUtraj();
        utraj->clear();
        for (int i = 0; i <= this->getNSteps(); i++) {
            state_t u_sol;
            double t = i * this->getDt();
            for (int j = 0; j < this->getNControls(); j++) {
                var = this->getVarByName(this->getControlName(i, j));
                u_sol.push_back(SCIPgetSolVal(scip_, sol, var));
            }
            utraj->push_back(traj_elem_t(t, u_sol));
        }
    } catch (...) {
        std::cout << "Exception occurred at getTraj";
        exit(EXIT_FAILURE);
    }
}

vector_t eSCIP::getStates(const size_t &tIdx) {
    vector_t x;
    for (size_t k(0); k <= this->getXrhorizon(); k ++) {
        size_t i = tIdx-k;
        for (size_t j(0); j < getNStates(); j++) {
            x.push_back(getVarByName((getStateName(i, j))));
        }
    }
    return x;
}

vector_t eSCIP::getControls(const size_t &tIdx) {
    vector_t u;
    for (size_t k(0); k <= this->getUrhorizon(); k ++) {
        size_t i = tIdx-k;
        for (size_t j(0); j < getNControls(); j++) {
            u.push_back(getVarByName((getControlName(i, j))));
        }
    }
    return u;
}

void eSCIP::releaseVars(SCIP_VAR** var, const size_t &len) {
    for (size_t i = 0; i < len; i++)
        SCIP_CALL_ABORT(SCIPreleaseVar(scip_, &var[i]));
}

// Overriding Setters

void eSCIP::setDt(double dt) {
    if (this->getDt() != dt) {
        reset_ = true;
        TrajectoryOptimizer::setDt(dt);
    }
}

void eSCIP::setNSteps(const size_t nSteps) {
    if (this->getNSteps() != nSteps) {
        reset_ = true;
        TrajectoryOptimizer::setNSteps(nSteps);
    }
}

void eSCIP::setXvartype(const state_var_t &xvartype) {
    if (!std::equal(this->getXvartype().cbegin(),
            this->getXvartype().cend(), xvartype.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setXvartype(xvartype);
    }
}

void eSCIP::setX0(const state_t& x0) {
    // Skip comparison and reset if array size is different
    if (this->getX0().size() == x0.size()) {
        if (!std::equal(this->getX0().cbegin(), this->getX0().cend(),
                x0.cbegin())) {
            // Only change the xf values in the SCIP model
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

void eSCIP::setXf(const state_t& xf) {
    // Skip comparison and reset if array size is different
    if (this->getXf().size() == xf.size()) {
        if (!std::equal(this->getXf().cbegin(), this->getXf().cend(),
                xf.cbegin())) {
            // Only change the xf values in the SCIP model
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

void eSCIP::setXtol(const state_t &xtol) {
    // Skip comparison and reset if array size is different
    if (this->getXtol().size() == xtol.size()) {
        if (!std::equal(this->getXtol().cbegin(), this->getXtol().cend(),
                xtol.cbegin())) {
            // Only change the xf values in the SCIP model
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

void eSCIP::setXupper(const state_t &xupper) {
    if (!std::equal(this->getXupper().cbegin(),
            this->getXupper().cend(), xupper.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setXupper(xupper);
    }
}

void eSCIP::setXlower(const state_t &xlower) {
    if (!std::equal(this->getXlower().cbegin(),
            this->getXlower().cend(), xlower.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setXlower(xlower);
    }
}

void eSCIP::setXrhorizon(const size_t nx4dyn) {
    if (this->getXrhorizon() != nx4dyn) {
        reset_ = true;
        TrajectoryOptimizer::setXrhorizon(nx4dyn);
    }
}

void eSCIP::setUvartype(const state_var_t &uvartype) {
    if (!std::equal(this->getUvartype().cbegin(),
            this->getUvartype().cend(), uvartype.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setUvartype(uvartype);
    }
}

void eSCIP::setUupper(const state_t &uupper) {
    if (!std::equal(this->getUupper().cbegin(),
            this->getUupper().cend(), uupper.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setUupper(uupper);
    }
}

void eSCIP::setUlower(const state_t &ulower) {
    if (!std::equal(this->getUlower().cbegin(),
            this->getUlower().cend(), ulower.cbegin())) {
        reset_ = true;
        TrajectoryOptimizer::setUlower(ulower);
    }
}

void eSCIP::setUrhorizon(const size_t nu4dyn) {
    if (this->getUrhorizon() != nu4dyn) {
        reset_ = true;
        TrajectoryOptimizer::setUrhorizon(nu4dyn);
    }
}

void eSCIP::setObjective(f_t* objective) {
    reset_ = true;
    TrajectoryOptimizer::setObjective(objective);
}

void eSCIP::setGradient(std::vector<f_t*> gradient) {
    reset_ = true;
    TrajectoryOptimizer::setGradient(gradient);
}

void eSCIP::setConstraints(std::vector<f_t*> constraints) {
    reset_ = true;
    TrajectoryOptimizer::setConstraints(constraints);
}


} /* namespace ETOL */
