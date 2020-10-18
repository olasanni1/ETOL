/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 16 October 2020
 * @version 1.0.1
 * @brief Loads a ETOL Config XML file and uses SCIP to solve a ETOL
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
#include <numeric>
#include <utility>
#include <ETOL/eSCIP_Types.hpp>
#include <ETOL/eSCIP.hpp>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
#define BIGM            1000
#define NSIDES             4

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
    ETOL::eSCIP ts = ETOL::eSCIP();
    t = &ts;

    t->loadConfigs(argv[1]);
    t->printConfigs();

    t->setMaximize(false);
    ETOL::f_t obj = &objFunction;
    t->setObjective(&obj);

    ETOL::f_t dx = &dxConstraint;
    ETOL::f_t dy = &dyConstraint;
    t->setGradient({&dx, &dy});

    ETOL::f_t absv, obs, saa;
    absv = &absConstraint;
    obs = obsConstraint(t);
    saa = saaConstraint(t);
    t->setConstraints({&absv, &obs, &saa});

    // Setup
    t->setup();

    // Solve
    t->solve();

    // Results
    std::cout << "\n!!!!!!!!!!!!!!!!!Results!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "Minimization Score:\t" << t->getScore() << std::endl;
    std::cout << "Plotting..." << std::endl;
    ETOL::TrajectoryOptimizer::plotXY_wExclZones(t->getXtraj(),
            t->getObstacles());

    // Release resources e.g. memory
    t->close();

    return EXIT_SUCCESS;
}

ETOL::scalar_t objFunction(F_ARGS) {
    try {
        SCIP_VAR* uk2  = std::any_cast<SCIP_VAR*>(u.at(2));      // u2 at t = k
        SCIP_VAR* uk3  = std::any_cast<SCIP_VAR*>(u.at(3));      // u3 at t = k
        ETOL::scip_expr_t expr =
                std::vector<std::pair<SCIP_VAR*, SCIP_Real>>(
                {std::make_pair(uk2, 1.0), std::make_pair(uk3, 1.0)});
        return expr;
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return std::vector<std::pair<SCIP_VAR*, SCIP_Real>>({});
}

ETOL::scalar_t dxConstraint(F_ARGS) {
    try {
        double dt_val = std::any_cast<double>(dt);
        SCIP_VAR* xk  = std::any_cast<SCIP_VAR*>(x.at(0));      // x at t = k
        SCIP_VAR* xk1  = std::any_cast<SCIP_VAR*>(x.at(2));     // x at t = k-1
        SCIP_VAR* uk  = std::any_cast<SCIP_VAR*>(u.at(0));  // u0 at t = k
        ETOL::scip_expr_t expr =
                std::vector<std::pair<SCIP_VAR*, SCIP_Real>>(
                {std::make_pair(xk, -1.0), std::make_pair(xk1, 1.0),
                 std::make_pair(uk, dt_val)});
        return expr;
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return std::vector<std::pair<SCIP_VAR*, SCIP_Real>>({});
}

ETOL::scalar_t dyConstraint(F_ARGS) {
    try {
        double dt_val = std::any_cast<double>(dt);
        SCIP_VAR* xk  = std::any_cast<SCIP_VAR*>(x.at(1));      // x at t = k
        SCIP_VAR* xk1  = std::any_cast<SCIP_VAR*>(x.at(3));     // x at t = k-1
        SCIP_VAR* uk  = std::any_cast<SCIP_VAR*>(u.at(1));      // u at t = k
        ETOL::scip_expr_t expr =
                std::vector<std::pair<SCIP_VAR*, SCIP_Real>>(
                {std::make_pair(xk, -1.0), std::make_pair(xk1, 1.0),
                 std::make_pair(uk, dt_val)});
        return expr;
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return std::vector<std::pair<SCIP_VAR*, SCIP_Real>>({});
}

ETOL::scalar_t absConstraint(F_ARGS) {
    try {
        SCIP_VAR* uk0 = std::any_cast<SCIP_VAR*>(u.at(0));
        SCIP_VAR* uk1 = std::any_cast<SCIP_VAR*>(u.at(1));
        SCIP_VAR* uk2 = std::any_cast<SCIP_VAR*>(u.at(2));
        SCIP_VAR* uk3 = std::any_cast<SCIP_VAR*>(u.at(3));
        ETOL::fout_scip_t fout = {
        {
            {std::make_pair(uk2, 1.0), std::make_pair(uk0, 1.0)},
            0.0, ESCIP_INF},
        {
            {std::make_pair(uk2, 1.0), std::make_pair(uk0, -1.0)},
            0.0, ESCIP_INF},
        {
            {std::make_pair(uk3, 1.0), std::make_pair(uk1, 1.0)},
            0.0, ESCIP_INF},
        {
            {std::make_pair(uk3, 1.0), std::make_pair(uk1, -1.0)},
            0.0, ESCIP_INF}
        };
        return fout;
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return ETOL::fout_scip_t();
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
        ETOL::fout_scip_t fout;
        try  {
            SCIP_VAR* xk = std::any_cast<SCIP_VAR*>(x.at(0));   // x at t = k
            SCIP_VAR* yk = std::any_cast<SCIP_VAR*>(x.at(1));   // y at t = k
            size_t tIdx = std::any_cast<size_t>(k);
            std::string t_str = std::to_string(tIdx);
            size_t ii(0);
            for (auto bd : obstacles) {
                size_t jj(0);
                for (auto lower : bd.first) {
                    size_t kk(0);
                    for (auto side : lower) {
                        SCIP_VAR* b;
                        std::string r(ETOL::eSCIP::getParamName(
                                paramName("bot", ii, jj, kk), tIdx));

                        bool found(false);
                        size_t i(0);
                        for (auto name : pnames) {
                            if (re2::RE2::FullMatch(name, r)) {
                                b = std::any_cast<SCIP_VAR*>(params.at(i));
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
                        double rhs = ya - m * xa;
                        fout.push_back({
                            {std::make_pair(yk, 1.0), std::make_pair(xk, -m),
                                std::make_pair(b, -BIGM)},
                            -ESCIP_INF, rhs});
                        kk++;
                    }

                    kk = 0;
                    ETOL::seg_t upper = bd.second.at(jj);
                    for (auto side : upper) {
                        SCIP_VAR* b;
                        std::string r(ETOL::eSCIP::getParamName(
                                paramName("top", ii, jj, kk), tIdx));

                        bool found(false);
                        size_t i(0);
                        for (auto name : pnames) {
                            if (re2::RE2::FullMatch(name, r)) {
                                b = std::any_cast<SCIP_VAR*>(params.at(i));
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
                        double rhs =  ya - m *xa;
                        fout.push_back({
                            {std::make_pair(yk, 1.0), std::make_pair(xk, -m),
                                std::make_pair(b, BIGM)},
                            rhs, ESCIP_INF});
                        kk++;
                    }

                    std::vector<SCIP_VAR*> bvars;
                    std::string r("^(?:bot|top)_"+ std::to_string(ii) +
                            "_" + std::to_string(jj) + "_[0-9]*_" + t_str +
                                    "$");
                    ETOL::vector_t::const_iterator p_it;
                    p_it = params.begin();
                    for (auto &name : pnames) {
                        if (re2::RE2::FullMatch(name, r))
                            bvars.push_back(std::any_cast<SCIP_VAR*>(*p_it));

                        p_it++;
                    }

                    int nelem = static_cast<int>(bvars.size());
                    if (nelem != 0) {
                        ETOL::scip_expr_t expr = {};
                        for (auto &b : bvars) {
                            expr.push_back(std::make_pair(b, 1.));
                        }
                        double rhs = static_cast<double>(nelem)-0.5;
                        fout.push_back({expr, -ESCIP_INF, rhs});
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
        ETOL::fout_scip_t fout;
        try {
            double delt = std::any_cast<double>(dt);
            size_t tIdx = std::any_cast<size_t>(k);
            double t = delt * static_cast<double>(tIdx);
            SCIP_VAR* xk = std::any_cast<SCIP_VAR*>(x.at(0));   // x at t = k
            SCIP_VAR* yk = std::any_cast<SCIP_VAR*>(x.at(1));   // y at t = k
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
                    SCIP_VAR* b;
                    std::string r(ETOL::eSCIP::getParamName(
                            paramName("d", ii, jj, kk), tIdx));

                    size_t i(0);
                    for (auto name : pnames) {
                        if (re2::RE2::FullMatch(name, r)) {
                            b = std::any_cast<SCIP_VAR*>(params.at(i));
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
                    fout.push_back({
                        {std::make_pair(xk, c0), std::make_pair(yk, c1),
                            std::make_pair(b, BIGM)},
                        rhs, ESCIP_INF});
                }

                std::vector<SCIP_VAR*> bvars;
                std::string r("^d_" + std::to_string(ii) + "_" +
                        std::to_string(jj) + "_[0-9]*_" +
                        std::to_string(tIdx) + "$");
                ETOL::vector_t::const_iterator p_it;
                p_it = params.begin();

                for (auto name : pnames) {
                    if (re2::RE2::FullMatch(name, r))
                        bvars.push_back(std::any_cast<SCIP_VAR*>(*p_it));

                    p_it++;
                }
                int nelem = static_cast<int>(bvars.size());
                if (nelem) {
                    ETOL::scip_expr_t expr = {};
                    for (auto &b : bvars) {
                        expr.push_back(std::make_pair(b, 1.));
                    }
                    double rhs = static_cast<double>(NSIDES)-0.5;
                    fout.push_back({expr, -ESCIP_INF, rhs});
                }
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
