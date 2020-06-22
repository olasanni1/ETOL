/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 3 May 2020
 * @version 1.0.1
 * @brief Creates an animation of the trajectory
 * @section DESCRIPTION
 * Solves the following MILP
 *    minimize    sum(u_k_2 + u_k_3)                 for k = 0 to NSteps
 *    subject to  x_k = x_(k-1) + dt * u_k_0         for k = 0 to Nsteps
 *                y_k = y_(k-1) + dt * u_k_1         for k = 0 to Nsteps
 *                u_k_2 = |u_k_0|                    for k = 0 to Nsteps
 *                u_k_3 = |u_k_1|                    for k = 0 to Nsteps
 *                (x_k, y_k) notin Obstacle            for k = 0 to Nsteps
 *                (x_k, y_k) notin Moving Obstacle    for k = 0 to Nsteps
 ******************************************************************************/

#include <re2/re2.h>
#include <iostream>
#include <cassert>
#include <ETOL/eGurobi.hpp>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
#define BIGM            1000
#define NSIDES          4

std::vector<ETOL::closure_t> obstacles;
std::vector<ETOL::track_t> tracks;

// Function prototypes
ETOL::scalar_t objFunction(F_ARGS);
ETOL::scalar_t dxConstraint(F_ARGS);
ETOL::scalar_t dyConstraint(F_ARGS);
ETOL::scalar_t absConstraint(F_ARGS);
ETOL::f_t obsConstraint(ETOL::TrajectoryOptimizer*);
ETOL::f_t saaConstraint(ETOL::TrajectoryOptimizer*);
std::string paramName(const std::string, const size_t&, const size_t&,
                        const size_t&);

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage: %s <ETOL configuration xml filepath>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    ETOL::TrajectoryOptimizer* t;
    ETOL::eGurobi tg = ETOL::eGurobi();
    t = &tg;

    t->loadConfigs(argv[1]);
    t->printConfigs();

    t->setMaximize(false);
    ETOL::f_t obj = &objFunction;
    t->setObjective(&obj);

    ETOL::f_t dx = &dxConstraint;
    ETOL::f_t dy = &dyConstraint;
    t->setGradient({&dx, &dy});

    ETOL::f_t absv, obs, saa;
    absv = &absConstraint,
    obs = obsConstraint(t);
    saa = saaConstraint(t);
    t->setConstraints({&absv, &obs, &saa});

    // Setup
    t->setup();

    // Solve
    t->solve();

    // Result
    std::cout << "\n!!!!!!!!!!!!!!!!!Results!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "Minimization Score:\t" << t->getScore() << std::endl;
    std::cout << "Animating..." << std::endl;
    // If toFile = TRUE, then ffmpeg must be installed
    ETOL::TrajectoryOptimizer::animate2D(t->getXtraj(), 1, false,
            "animation.mp4", t->getObstacles(), t->getTracks());

    // Release resources e.g. memory
    t->close();

    return EXIT_SUCCESS;
}

ETOL::scalar_t objFunction(F_ARGS) {
    GRBLinExpr expr = GRBLinExpr();
    try {
        const double coeff[] = {1.0, 1.0};
        GRBVar uk2  = std::any_cast<GRBVar>(u.at(2));      // u2 at t = k
        GRBVar uk3  = std::any_cast<GRBVar>(u.at(3));      // u3 at t = k
        GRBVar vars[] = {uk2, uk3};
        expr.addTerms(coeff, vars, 2);
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return expr;
}

ETOL::scalar_t dxConstraint(F_ARGS) {
    GRBLinExpr expr = GRBLinExpr();
    try {
        double dt_val = std::any_cast<double>(dt);
        const double coeff[] = {-1.0, 1.0, dt_val};
        GRBVar xk  = std::any_cast<GRBVar>(x.at(0));      // x at t = k
        GRBVar xk1 = std::any_cast<GRBVar>(x.at(2));     // x at t = k-1
        GRBVar uk  = std::any_cast<GRBVar>(u.at(0));      // u at t = k
        GRBVar vars[] = {xk, xk1, uk};
        expr.addTerms(coeff, vars, 3);
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return expr;
}

ETOL::scalar_t dyConstraint(F_ARGS) {
    GRBLinExpr expr = GRBLinExpr();
    try {
        double dt_val = std::any_cast<double>(dt);
        const double coeff[] = {-1.0, 1.0, dt_val};
        GRBVar yk  = std::any_cast<GRBVar>(x.at(1));      // y at t = k
        GRBVar yk1 = std::any_cast<GRBVar>(x.at(3));     // y at t = k-1
        GRBVar uk  = std::any_cast<GRBVar>(u.at(1));      // u at t = k
        GRBVar vars[] = {yk, yk1, uk};
        expr.addTerms(coeff, vars, 3);
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return expr;
}

ETOL::scalar_t absConstraint(F_ARGS) {
    ETOL::fout_grb_t fout;
    try {
        GRBLinExpr expr = GRBLinExpr();
        GRBVar uk0  = std::any_cast<GRBVar>(u.at(0));      // x at t = k
        GRBVar uk1  = std::any_cast<GRBVar>(u.at(1));      // x at t = k
        GRBVar uk2  = std::any_cast<GRBVar>(u.at(2));      // x at t = k
        GRBVar uk3  = std::any_cast<GRBVar>(u.at(3));      // x at t = k
        double coeffs0[] = {1.0, 1.0};
        GRBVar vars0[]   = {uk2, uk0};
        double coeffs1[] = {1.0, -1.0};
        GRBVar vars1[]   = {uk3, uk1};

        expr.addTerms(coeffs0, vars0, 2);
        fout.push_back({expr, GRB_GREATER_EQUAL, 0.});
        expr.clear();

        expr.addTerms(coeffs0, vars1, 2);
        fout.push_back({expr, GRB_GREATER_EQUAL, 0.});
        expr.clear();

        expr.addTerms(coeffs1, vars0, 2);
        fout.push_back({expr, GRB_GREATER_EQUAL, 0.});
        expr.clear();

        expr.addTerms(coeffs1, vars1, 2);
        fout.push_back({expr, GRB_GREATER_EQUAL, 0.});
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return fout;
}

ETOL::f_t obsConstraint(ETOL::TrajectoryOptimizer* t) {
    double tspan = t->getDt() * t->getNSteps();

    // Add ancillary variables for obstacle constraint
    // obstacles must be global variables to work in callback
    size_t i(0);
    for (auto obs : *t->getObstacles()) {
        std::vector<ETOL::seg_t> lowers, uppers;

        // Compute slopes of obstacle
        ETOL::TrajectoryOptimizer::calcSlopes(obs, &lowers, &uppers);
        obstacles.push_back(ETOL::closure_t(lowers, uppers));

        size_t j(0);
        for (auto lower : lowers) {
            for (size_t k(0); k < lower.size(); k++)
                t->addParams({std::pair<PARAM_PAIR>(
                        paramName("bot", i, j, k),
                        {ETOL::var_t::BINARY, 0., 1., 0., tspan})});

            ETOL::seg_t upper = uppers.at(j);
            for (size_t k(0); k < upper.size(); k++)
                t->addParams({std::pair<PARAM_PAIR>(
                        paramName("top", i, j, k),
                        {ETOL::var_t::BINARY, 0., 1., 0., tspan})});

            j++;
        }
        i++;
    }

    ETOL::f_t o;
    o = [](F_ARGS){
        ETOL::fout_grb_t fout;
        try  {
            GRBVar xk  = std::any_cast<GRBVar>(x.at(0));      // x at t = k
            GRBVar yk  = std::any_cast<GRBVar>(x.at(1));     // y at t = k
            size_t tIdx = std::any_cast<size_t>(k);
            std::string t_str = std::to_string(tIdx);
            size_t ii(0);
            for (auto bd : obstacles) {
                size_t jj(0);
                for (auto lower : bd.first) {
                    size_t kk(0);
                    for (auto side : lower) {
                        ETOL::grb_t grb;
                        GRBLinExpr expr1 = GRBLinExpr();
                        GRBVar b;
                        std::string r(ETOL::eGurobi::getParamName(
                                paramName("bot", ii, jj, kk), tIdx));

                        bool found(false);
                        size_t i(0);
                        for (auto name : pnames) {
                            if (re2::RE2::FullMatch(name, r)) {
                                b = std::any_cast<GRBVar>(params.at(i));
                                found = true;
                                break;
                            }
                            i++;
                        }

                        if (!found)
                            break;

                        double xa = std::get<0>(lower.at(kk).first);
                        double ya = std::get<1>(lower.at(kk).first);
                        double m = lower.at(kk).second.slope;
                        double coeffs[] = {1., -m, -BIGM};
                        GRBVar vars[] = {yk, xk, b};
                        expr1.addTerms(coeffs, vars, 3);

                        double rhs = ya - m * xa;
                        grb = {expr1, GRB_LESS_EQUAL, rhs};
                        fout.push_back(grb);
                        kk++;
                    }

                    kk = 0;
                    ETOL::seg_t upper = bd.second.at(jj);
                    for (auto side : upper) {
                        ETOL::grb_t grb;
                        GRBLinExpr expr2 = GRBLinExpr();
                        GRBVar b;
                        std::string r(ETOL::eGurobi::getParamName(
                                paramName("top", ii, jj, kk), tIdx));

                        bool found(false);
                        size_t i(0);
                        for (auto name : pnames) {
                            if (re2::RE2::FullMatch(name, r)) {
                                b = std::any_cast<GRBVar>(params.at(i));
                                found = true;
                                break;
                            }
                            i++;
                        }

                        if (!found)
                            break;

                        double xa =  std::get<0>(upper.at(kk).first);
                        double ya =  std::get<1>(upper.at(kk).first);
                        double m = upper.at(kk).second.slope;
                        double coeffs[] = {1., -m, BIGM};
                        GRBVar vars[] = {yk, xk, b};
                        expr2.addTerms(coeffs, vars, 3);
                        double rhs =  ya - m *xa;
                        grb = {expr2, GRB_GREATER_EQUAL, rhs};
                        fout.push_back(grb);
                        kk++;
                    }

                    GRBLinExpr expr3 = GRBLinExpr();
                    std::vector<GRBVar> bvars;
                    std::string r("^(?:bot|top)_"+ std::to_string(ii) +
                            "_" + std::to_string(jj) + "_[0-9]*_" + t_str +
                                    "$");
                    ETOL::vector_t::iterator p_it;
                    p_it = params.begin();
                    for (auto name : pnames) {
                        if (re2::RE2::FullMatch(name, r))
                            bvars.push_back(std::any_cast<GRBVar>(*p_it));

                        p_it++;
                    }

                    int nelem = static_cast<int>(bvars.size());
                    if (nelem != 0) {
                        std::vector<double> coeff_vec(nelem, 1.);
                        expr3.addTerms(coeff_vec.data(), bvars.data(), nelem);
                        fout.push_back({expr3, GRB_LESS_EQUAL,
                            static_cast<double>(nelem)-0.5});
                    }
                    jj++;
                }
                ii++;
            }
        }catch(std::bad_any_cast& e) {
            std::cout << e.what() << std::endl;;
            exit(EXIT_FAILURE);
        }
        return fout;
    };
    return o;
}

ETOL::f_t saaConstraint(ETOL::TrajectoryOptimizer* t) {
    ETOL::f_t saa;
    double tspan = t->getDt() * t->getNSteps();

    assert(NSIDES > 2);
    assert((NSIDES % 2) == 0);

    // Add ancillary variables for sense and avoid constraint
    // tracks must be global variables to work in callback
    size_t i(0);
    for (auto track : *t->getTracks()) {
        size_t j(0);
        ETOL::traj_t::iterator it;
        it = track.trajectory.begin();
        for (auto elem : track.trajectory) {
            double t0, tf;
            t0 = elem.first;
            std::advance(it, 1);
            if (it != track.trajectory.end())
                tf = (*it).first;
            else
                tf = tspan;
            ETOL::param_configs_t configs = {ETOL::var_t::BINARY,
                                                    0., 1., t0, tf};
            for (size_t k(0); k < NSIDES; k++ )
                t->addParams({std::pair<PARAM_PAIR>(
                        paramName("d", i, j, k), configs)});
            j++;
        }
        tracks.push_back(track);
        i++;
    }

    saa = [](F_ARGS){
        ETOL::fout_grb_t fout;
        try {
            double delt = std::any_cast<double>(dt);
            size_t tIdx = std::any_cast<size_t>(k);
            double t = delt * static_cast<double>(tIdx);
            GRBVar xk  = std::any_cast<GRBVar>(x.at(0));      // x at t = k
            GRBVar yk  = std::any_cast<GRBVar>(x.at(1));     // y at t = k
            size_t ii(0);
            for (auto track : tracks) {
                double radius = track.radius;
                std::vector<double> tvec;
                for_each(track.trajectory.begin(), track.trajectory.end(),
                        [&tvec](const ETOL::traj_elem_t data){
                    double value = data.first;
                    tvec.push_back(value);
                });
                std::vector<double>::iterator tlo_it;
                tlo_it = std::lower_bound(tvec.begin(), tvec.end(), t);
                if (tlo_it != tvec.begin() && *tlo_it > t)
                    tlo_it = std::prev(tlo_it);
                size_t jj = distance(tvec.begin(), tlo_it);

                ETOL::traj_t::iterator curr, next;
                curr = track.trajectory.begin();
                std::advance(curr, jj);
                next = curr;
                if (tlo_it != prev(tvec.end()))
                    std::advance(next, 1);

                double delt =  next->first - curr->first;
                double rhs;
                for (size_t kk(0); kk < NSIDES; kk++) {
                    ETOL::grb_t grb;
                    GRBLinExpr expr1 = GRBLinExpr();
                    GRBVar b;
                    std::string r(ETOL::eGurobi::getParamName(
                            paramName("d", ii, jj, kk), tIdx));

                    size_t i(0);
                    for (auto name : pnames) {
                        if (re2::RE2::FullMatch(name, r)) {
                            b = std::any_cast<GRBVar>(params.at(i));
                            break;
                        }
                        i++;
                    }

                    double c0 = cos(2. * M_PI * static_cast<double>(kk) /
                            static_cast<double>(NSIDES));
                    double c1 = sin(2. * M_PI * static_cast<double>(kk) /
                            static_cast<double>(NSIDES));

                    double mx(0), my(0);
                    double valx0 = curr->second.at(0);
                    double valy0 = curr->second.at(1);
                    double valx1 = next->second.at(0);
                    double valy1 = next->second.at(1);

                    if (delt != 0) {
                        mx = (valx1 - valx0)/ delt;
                        my = (valy1 - valy0)/ delt;
                    }
                    double cx = valx0 + mx*(t - *tlo_it);
                    double cy = valy0 + my*(t - *tlo_it);
                    rhs =  c0*cx + c1*cy + radius;

                    double coeffs[] = {c0, c1, BIGM};
                    GRBVar vars[] = {xk, yk, b};
                    expr1.addTerms(coeffs, vars, 3);
                    grb = {expr1, GRB_GREATER_EQUAL, rhs};
                    fout.push_back(grb);
                }

                GRBLinExpr expr2 = GRBLinExpr();
                std::vector<GRBVar> bvars;
                std::string r("^d_" + std::to_string(ii) + "_" +
                        std::to_string(jj) + "_[0-9]*_" +
                        std::to_string(tIdx) + "$");
                ETOL::vector_t::iterator p_it;
                p_it = params.begin();

                for (auto name : pnames) {
                    if (re2::RE2::FullMatch(name, r))
                        bvars.push_back(std::any_cast<GRBVar>(*p_it));

                    p_it++;
                }
                int nelem = static_cast<int>(bvars.size());
                std::vector<double> coeff_vec(nelem, 1.);
                expr2.addTerms(coeff_vec.data(), bvars.data(), nelem);
                fout.push_back({expr2, GRB_LESS_EQUAL, NSIDES - 0.5});
                ii++;
            }
        } catch(std::bad_any_cast& e) {
            std::cout << e.what() << std::endl;;
            exit(EXIT_FAILURE);
        }
        return fout;
    };


    return saa;
}

std::string paramName(const std::string name, const size_t& i, const size_t& j,
                    const size_t& k) {
    return (name + "_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
            std::to_string(k));
}
