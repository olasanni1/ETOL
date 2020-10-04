/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Thanakorn Khamvilai
 * @author Thanakorn Khamvilai <thanakornkhamvilai@gmail.com>
 * @date 6 May 2020
 * @version 1.0.0
 * @brief A GLPK implementation for ETOL
 * @section DESCRIPTION
 * A <a href="https://www.gnu.org/software/glpk/">GLPK 4.32</a> implementation
 * for the pure virtual methods in the TrajectoryOptimization class. It casts
 * the trajectory optimization problem as a Mixed-Integer Linear Programming
 * (MILP) type.
 ******************************************************************************/

#include <re2/re2.h>
#include <iostream>
#include <cassert>
#include <ETOL/eGLPK.hpp>

namespace ETOL {
// Constructors
eGLPK::eGLPK() : eGLPK::TrajectoryOptimizer() {
    this->_model = glp_create_prob();
    glp_init_iocp(&this->_params);
    this->_params.presolve = GLP_ON;
    this->_var_cnt = 0;
    this->_const_cnt = 0;
    this->_ind_cnt = 0;
    this->_status_mip = 0;
    this->_status_relax = 0;
}

// Virtual Members
eGLPK::~eGLPK() {}

// API
void eGLPK::setup() {
    assert(this->getX0().size() == this->getNStates());
    assert(this->getXf().size() == this->getNStates());
    assert(this->getXlower().size() == this->getNStates());
    assert(this->getXupper().size() == this->getNStates());
    assert(this->getXvartype().size() == this->getNStates());

    try {
        this->x.resize(this->getNStates());
        this->u.resize(this->getNControls());

        this->createVars();
        this->addX0();
        this->addXf();
        this->addDyn();
        this->addConstr();
        this->addObj();
        glp_load_matrix(this->_model, this->_ind_cnt, &this->_ia[0],
                &this->_ja[0], &this->_ar[0]);
    } catch (...) {
        std::cout << "Exception occurred at setup";
        exit(EXIT_FAILURE);
    }
}

void eGLPK::solve() {
    try {
        this->_status_relax = glp_intopt(this->_model, &this->_params);
        this->_status_mip = glp_mip_status(this->_model);
        if (this->_status_mip == GLP_OPT) {
            this->write_sol();
            this->setScore(glp_mip_obj_val(this->_model));
            this->getTraj();
        }
    } catch (...) {
        std::cout << "Exception occurred at solve";
        exit(EXIT_FAILURE);
    }
}

void eGLPK::debug() {
    this->write_lp();
}

void eGLPK::close() {
    glp_delete_prob(this->_model);
    glp_free_env();
}

// static functions
int eGLPK::getGLPKType(var_t varType) {
    switch (varType) {
        case var_t::CONTINUOUS:
            return GLP_CV;
        case var_t::INTERGER:
            return     GLP_IV;
        case var_t::BINARY:
            return GLP_BV;
        default :
            assert(true);  // Type not implemented
    }
    return GLP_CV;
}

std::string eGLPK::getStateName(const size_t& tIdx, const size_t& sIdx) {
    return("x_" + std::to_string(tIdx) + "_" + std::to_string(sIdx));
}

std::string eGLPK::getControlName(const size_t& tIdx, const size_t& sIdx) {
    return("u_" + std::to_string(tIdx) + "_" + std::to_string(sIdx));
}

std::string eGLPK::getParamName(const std::string name, const size_t& tIdx) {
    return(name + "_" + std::to_string(tIdx));
}

std::string eGLPK::getParamName(std::string name, const size_t& tIdx,
        const size_t& sIdx) {
    return(name + "_" + std::to_string(tIdx) + "_" + std::to_string(sIdx));
}

std::string eGLPK::getParamName(std::string name, const size_t& Idx1,
        const size_t& Idx2, const size_t& Idx3) {
    return(name + "_" + std::to_string(Idx1) + "_" + std::to_string(Idx2) +
            "_" + std::to_string(Idx3));
}

std::vector<closure_t> eGLPK::createObstacles(TrajectoryOptimizer *t) {
    double tspan = t->getDt() * t->getNSteps();
    std::vector<closure_t> obstacles;

    int i = 0;
    for (const auto& obs : *t->getObstacles()) {
        std::vector<seg_t> lowers, uppers;

        // Compute slopes of obstacle
        TrajectoryOptimizer::calcSlopes(obs, &lowers, &uppers);
        obstacles.emplace_back(lowers, uppers);

        int j = 0;
        for (const auto& lower : lowers) {
            for (int k = 0; k < lower.size(); k++)
                t->addParams({std::pair<PARAM_PAIR>(
                        getParamName("bot", i, j, k),
                        {var_t::BINARY, 0., 1., 0., tspan})});

            seg_t upper = uppers.at(j);
            for (int k = 0; k < upper.size(); k++)
                t->addParams({std::pair<PARAM_PAIR>(
                        getParamName("top", i, j, k),
                        {var_t::BINARY, 0., 1., 0., tspan})});
            j++;
        }
        i++;
    }
    return obstacles;
}

std::vector<track_t> eGLPK::createTracks(TrajectoryOptimizer *t,
                                                int NSIDES) {
    double tspan = t->getDt() * t->getNSteps();
    std::vector<track_t> tracks;

    assert(NSIDES > 2);
    assert((NSIDES % 2) == 0);

    int i = 0;
    for (auto track : *t->getTracks()) {
        int j = 0;
        traj_t::iterator it;
        it = track.trajectory.begin();
        for (const auto& elem : track.trajectory) {
            double t0, tf;
            t0 = elem.first;
            std::advance(it, 1);
            if (it != track.trajectory.end())
                tf = (*it).first;
            else
                tf = tspan;
            param_configs_t configs = {var_t::BINARY, 0., 1., t0, tf};
            for (int k = 0; k < NSIDES; k++ )
                t->addParams({std::pair<PARAM_PAIR>(
                                        getParamName("d", i, j, k), configs)});
            j++;
        }
        tracks.push_back(track);
        i++;
    }
    return tracks;
}

std::vector<std::pair<std::vector<glpk_t>, glpk_bnd_t>>
eGLPK::addObstacleSides(seg_t seg, int sign, const std::string& name,
        int ind_i, int ind_j, int tIdx, std::vector<std::string> pnames,
        int BigM) {
    int ind_k = 0;
    std::vector<glpk_t> abs_obs;
    std::vector<std::pair<std::vector<glpk_t>, glpk_bnd_t>> abs_obsv;
    for (auto side : seg) {
        std::string r(eGLPK::getParamName(
                eGLPK::getParamName(name, ind_i, ind_j, ind_k), tIdx));
        bool found(false);
        int i = 0;
        for (const auto& pname : pnames) {
            if (re2::RE2::FullMatch(pname, r)) {
                abs_obs.emplace_back(pname, tIdx, -1, sign*BigM);
                found = true;
                break;
            }
            i++;
        }
        if (!found)
            break;
        double m = seg.at(ind_k).second.slope;
        abs_obs.emplace_back("x", tIdx, 0, -m);  // x
        abs_obs.emplace_back("x", tIdx, 1, 1);  // y

        double xa = std::get<0>(seg.at(ind_k).first);
        double ya = std::get<1>(seg.at(ind_k).first);
        double rhs = ya - m * xa;
        if (sign < 0)
            abs_obsv.emplace_back(abs_obs, glpk_bnd_t(GLPK_UNBOUND, rhs));
        else
            abs_obsv.emplace_back(abs_obs, glpk_bnd_t(rhs, GLPK_UNBOUND));
        abs_obs.clear();
        ind_k++;
    }
    return abs_obsv;
}

std::vector<std::pair<std::vector<glpk_t>, glpk_bnd_t>>
eGLPK::addObstacleSum(const std::string &name, int ind_i,
        int ind_j, int tIdx, std::vector<std::string> pnames, int NSIDES) {
    std::vector<glpk_t> abs;
    std::vector<std::pair<std::vector<glpk_t>, glpk_bnd_t>> abs_v;
    std::string r(eGLPK::getParamName(name, ind_i, ind_j) + "_[0-9]*_" +
                                            std::to_string(tIdx) + "$");
    for (const auto& pname : pnames) {
        if (re2::RE2::FullMatch(pname, r)) {
            abs.emplace_back(pname, tIdx, -1, 1);
        }
    }
    if (NSIDES > 0)
        abs_v.emplace_back(abs, glpk_bnd_t(GLPK_UNBOUND, NSIDES-0.5));
    else
        abs_v.emplace_back(abs, glpk_bnd_t(GLPK_UNBOUND, abs.size()-0.5));
    return abs_v;
}

// Other functions
int eGLPK::getVarByName(const std::string& name) {
    return this->_var_map[name];
}

void eGLPK::read_lp(const char* lp_file) {
    glp_read_lp(this->_model, NULL, lp_file);
}

void eGLPK::write_lp() {
    glp_write_lp(this->_model, NULL, "debug_glpk.lp");
}

void eGLPK::write_sol() {
    glp_write_mip(this->_model, "sol_glpk_compact.txt");
    glp_print_mip(this->_model, "sol_glpk_verbose.txt");
}

void eGLPK::solve_lp(const char *lp_file) {
    this->read_lp(lp_file);
    this->debug();
    this->solve();
    this->write_sol();
    this->close();
}

// private members
void eGLPK::createVars() {
    state_t::iterator l_it, u_it;
    state_var_t::iterator vt_it;
    try {
        for (int i = 0; i <= this->getNSteps(); i++) {
            // x_time#_state#
            l_it = this->getXlower().begin();
            u_it = this->getXupper().begin();
            vt_it = this->getXvartype().begin();
            for (int j = 0; j < this->getNStates(); j++) {
                this->_var_cnt += 1;
                this->_var_map.insert(std::pair<std::string, int>(
                        this->getStateName(i, j), this->_var_cnt));
                glp_add_cols(this->_model, 1);
                glp_set_col_name(this->_model, this->_var_cnt,
                                    this->getStateName(i, j).c_str());
                glp_set_col_kind(this->_model, this->_var_cnt,
                                    this->getGLPKType(*(vt_it++)));
                glp_set_col_bnds(this->_model, this->_var_cnt, GLP_DB,
                                    *(l_it++), *(u_it++));
            }

            // u_time#_state#
            l_it = this->getUlower().begin();
            u_it = this->getUupper().begin();
            vt_it = this->getUvartype().begin();
            for (int j = 0; j < this->getNControls(); j++) {  // ignore u_0
                this->_var_cnt += 1;
                this->_var_map.insert(std::pair<std::string, int>(
                        this->getControlName(i, j), this->_var_cnt));
                glp_add_cols(this->_model, 1);
                glp_set_col_name(this->_model, this->_var_cnt,
                        this->getControlName(i, j).c_str());
                glp_set_col_kind(this->_model, this->_var_cnt,
                        this->getGLPKType(*(vt_it++)));
                glp_set_col_bnds(this->_model, this->_var_cnt, GLP_DB,
                                    *(l_it++), *(u_it++));
            }

            // varName_time#
            double t = i * this->getDt();
            for (const auto& param : this->_parameters) {
                if ((t >= param.second.tStart) && (t <= param.second.tStop)) {
                    this->_var_cnt += 1;
                    this->_var_map.insert(std::pair<std::string, int>(
                        this->getParamName(param.first, i), this->_var_cnt));
                    glp_add_cols(this->_model, 1);
                    glp_set_col_name(this->_model, this->_var_cnt,
                            this->getParamName(param.first, i).c_str());
                    glp_set_col_kind(this->_model, this->_var_cnt,
                            this->getGLPKType(param.second.varType));
                    glp_set_col_bnds(this->_model, this->_var_cnt, GLP_DB,
                            param.second.lbnd, param.second.ubnd);
                    this->params.push_back(this->_var_cnt);
                    this->pnames.push_back(this->getParamName(param.first, i));
                }
            }
        }
    } catch (...) {
        std::cout << "Exception occurred at createVars";
        exit(EXIT_FAILURE);
    }
}

void eGLPK::addX0() {
    try {
        for (int i = 0; i < this->getRhorizon(); i++) {
            for (int j = 0; j < this->getNStates(); j++) {
                if (!std::isnan(this->getX0().at(j))) {
                    this->_ind_cnt += 1;
                    this->_const_cnt += 1;
                    glp_add_rows(this->_model, 1);
                    glp_set_row_bnds(this->_model, this->_const_cnt, GLP_FX,
                            this->getX0().at(j), this->getX0().at(j));
                    this->_ia.push_back(this->_const_cnt);
                    this->_ja.push_back(this->getVarByName(
                                                    this->getStateName(i, j)));
                    this->_ar.push_back(1);
                }
            }
        }
    } catch (...) {
        std::cout << "Exception occurred at addX0";
        exit(EXIT_FAILURE);
    }
}

void eGLPK::addXf() {
    try {
        int i = this->getNSteps();
        for (int j = 0; j < this->getNStates(); j++) {
            if (!std::isnan(this->getXf().at(j))) {
                this->_ind_cnt += 1;
                this->_const_cnt += 1;
                glp_add_rows(this->_model, 1);
                glp_set_row_bnds(this->_model, this->_const_cnt, GLP_UP,
                                 this->getXf().at(j) - this->getXtol().at(j),
                                 this->getXf().at(j) + this->getXtol().at(j));
                this->_ia.push_back(this->_const_cnt);
                this->_ja.push_back(this->getVarByName(
                                        this->getStateName(i, j)));
                this->_ar.push_back(1);

                this->_ind_cnt += 1;
                this->_const_cnt += 1;
                glp_add_rows(this->_model, 1);
                glp_set_row_bnds(this->_model, this->_const_cnt, GLP_LO,
                                 this->getXf().at(j) - this->getXtol().at(j),
                                 this->getXf().at(j) + this->getXtol().at(j));
                this->_ia.push_back(this->_const_cnt);
                this->_ja.push_back(this->getVarByName(
                                        this->getStateName(i, j)));
                this->_ar.push_back(1);
            }
        }
    } catch (...) {
        std::cout << "Exception occurred at addXf";
        exit(EXIT_FAILURE);
    }
}

void eGLPK::addConstr() {
    try {
        std::any f_val;
        f_t* f;
        std::vector<std::pair<std::vector<glpk_t>, glpk_bnd_t>> Constr;
        for (int c = 0; c < this->_constraints.size(); c++) {
            f = this->_constraints.at(c);
            for (int k = 0; k <= this->getNSteps(); k++) {
                f_val = (*f)(this->x, this->u, this->params, this->pnames, k,
                        this->getDt());
                Constr = std::any_cast<std::vector<std::pair
                        <std::vector<glpk_t>, glpk_bnd_t>>>(f_val);
                for (int i = 0; i < Constr.size(); i++) {
                    if (Constr[i].second._lb != GLPK_UNBOUND) {
                        this->_const_cnt += 1;
                        glp_add_rows(this->_model, 1);
                        glp_set_row_bnds(this->_model, this->_const_cnt,
                                GLP_LO, Constr[i].second._lb,
                                         Constr[i].second._ub);
                        for (auto &j : Constr[i].first) {
                            this->_ind_cnt += 1;
                            this->_ia.push_back(this->_const_cnt);
                            if (j._sIdx >= 0) {
                                this->_ja.push_back(
                                        this->getVarByName(
                                                this->getParamName(j._name,
                                                        j._tIdx, j._sIdx)));
                            } else {
                                this->_ja.push_back(
                                        this->getVarByName(j._name));
                            }
                            this->_ar.push_back(j._obj_coeff);
                        }
                    }
                    if (Constr[i].second._ub != GLPK_UNBOUND) {
                        this->_const_cnt += 1;
                        glp_add_rows(this->_model, 1);
                        glp_set_row_bnds(this->_model, this->_const_cnt,
                                GLP_UP, Constr[i].second._lb,
                                         Constr[i].second._ub);
                        for (auto &j : Constr[i].first) {
                            this->_ind_cnt += 1;
                            this->_ia.push_back(this->_const_cnt);
                            if (j._sIdx >= 0) {
                                this->_ja.push_back(
                                        this->getVarByName(
                                                this->getParamName(j._name,
                                                        j._tIdx, j._sIdx)));
                            } else {
                                this->_ja.push_back(
                                        this->getVarByName(j._name));
                            }
                            this->_ar.push_back(j._obj_coeff);
                        }
                    }
                }
            }
        }
    } catch (...) {
        std::cout << "Exception occurred at addConstr";
        exit(EXIT_FAILURE);
    }
}

void eGLPK::addDyn() {
    try {
        std::any f_val;
        f_t* f;
        std::vector<glpk_t> Dyn;
        for (int i = 0; i < this->getNStates(); i++) {
            f = this->_gradient.at(i);
            for (int k = this->getRhorizon(); k <= this->getNSteps(); k++) {
                f_val = (*f)(this->x, this->u, this->params, this->pnames, k,
                        this->getDt());
                Dyn = std::any_cast<std::vector<glpk_t>>(f_val);
                this->_const_cnt += 1;
                glp_add_rows(this->_model, 1);
                glp_set_row_bnds(this->_model, this->_const_cnt, GLP_FX, 0, 0);
                for (auto &j : Dyn) {
                    this->_ind_cnt += 1;
                    this->_ia.push_back(this->_const_cnt);
                    this->_ja.push_back(this->getVarByName(
                            this->getParamName(j._name, j._tIdx, j._sIdx)));
                    this->_ar.push_back(j._obj_coeff);
                }
            }
        }
    } catch (...) {
        std::cout << "Exception occurred at addDyn";
        exit(EXIT_FAILURE);
    }
}

void eGLPK::addObj() {
    try {
        std::any f_val;
        std::vector<glpk_t> Obj;
        for (int k = 1; k <= this->getNSteps(); k++) {
            f_val = (*this->_objective)(this->x, this->u, this->params,
                    this->pnames, k, this->getDt());
            Obj = std::any_cast<std::vector<glpk_t>>(f_val);
            for (auto &i : Obj) {
                glp_set_obj_coef(this->_model,
                        this->getVarByName(this->getParamName(i._name,
                                i._tIdx, i._sIdx)), i._obj_coeff);
            }
        }
        if (this->isMaximized())
            glp_set_obj_dir(this->_model, GLP_MAX);
        else
            glp_set_obj_dir(this->_model, GLP_MIN);
    } catch (...) {
        std::cout << "Exception occurred at addObj";
        exit(EXIT_FAILURE);
    }
}

void eGLPK::getTraj() {
    try {
        int var;
        traj_t* xtraj = this->getXtraj();
        xtraj->clear();
        for (int i = 0; i <= this->getNSteps(); i++) {
            state_t x_sol;
            double t = i * this->getDt();
            for (int j = 0; j < this->getNStates(); j++) {
                var = this->getVarByName(this->getStateName(i, j));
                x_sol.push_back(glp_mip_col_val(this->_model, var));
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
                u_sol.push_back(glp_mip_col_val(this->_model, var));
            }
            utraj->push_back(traj_elem_t(t, u_sol));
        }
    } catch (...) {
        std::cout << "Exception occurred at getTraj";
        exit(EXIT_FAILURE);
    }
}

} /* namespace ETOL */
