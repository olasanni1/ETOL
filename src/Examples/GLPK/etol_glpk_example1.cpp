/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Thanakorn Khamvilai
 * @author Thanakorn Khamvilai <thanakornkhamvilai@gmail.com>
 * @date 6 May 2020
 * @version 1.0.0
 * @brief Loads a ETOL Config XML file and uses GLPK to solve a ETOL
 * @section DESCRIPTION
 * Solves the following MILP
 *    minimize    sum(u_k_2 + u_k_3)                 for k = 0 to NSteps
 *    subject to  x_k = x_(k-1) + dt * u_k_0         for k = 0 to Nsteps
 *                y_k = y_(k-1) + dt * u_k_1         for k = 0 to Nsteps
 *                u_k_2 = |u_k_0|                    for k = 0 to Nsteps
 *                u_k_3 = |u_k_1|                    for k = 0 to Nsteps
 *                (x_k, y_k) notin Obstacle          for k = 0 to Nsteps
 *                (x_k, y_k) notin Moving Obstacle   for k = 0 to Nsteps
 ******************************************************************************/

#include <re2/re2.h>
#include <iostream>
#include <ETOL/eGLPK.hpp>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
#define BIGM            1000
#define NSIDES            4

std::vector<ETOL::closure_t> obstacles;
std::vector<ETOL::track_t> tracks;

// Function prototypes
ETOL::scalar_t objFunction(F_ARGS);
ETOL::scalar_t dxConstraint(F_ARGS);
ETOL::scalar_t dyConstraint(F_ARGS);
ETOL::scalar_t absConstraint(F_ARGS);
ETOL::f_t obsConstraint(ETOL::TrajectoryOptimizer*);
ETOL::f_t saaConstraint(ETOL::TrajectoryOptimizer*);

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage: %s <ETOL configuration xml filepath>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    std::cout << "GLPK" << std::endl;

    ETOL::TrajectoryOptimizer* t;
    ETOL::eGLPK tg = ETOL::eGLPK();
    t = &tg;

    t->loadConfigs(argv[1]);
    // t->printConfigs();

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

    // Debug
    t->debug();

    // Solve
    t->solve();

    // Result
    std::cout << "\n!!!!!!!!!!!!!!!!!Results!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "Minimization Score:\t" << t->getScore() << std::endl;
    std::cout << "Plotting..." << std::endl;
    ETOL::TrajectoryOptimizer::plotXY_wExclZones(t->getXtraj(),
                                                    t->getObstacles());
//    ETOL::TrajectoryOptimizer::animate2D(t->getXtraj(), 1, false,
//            "animation.mp4", t->getObstacles(), t->getTracks());

    t->plotX(0);
    t->plotX(1);
    t->plotU(0);
    t->plotU(1);

    // Release resources e.g. memory
    t->close();

    return EXIT_SUCCESS;
}

ETOL::scalar_t objFunction(F_ARGS) {
    int tIdx = std::any_cast<int>(k);
    std::vector<ETOL::glpk_t> obj;
    obj.emplace_back("u", tIdx, 2, 1);
    obj.emplace_back("u", tIdx, 3, 1);
    return obj;
}

ETOL::scalar_t dxConstraint(F_ARGS) {
    int tIdx = std::any_cast<int>(k);
    std::vector<ETOL::glpk_t> dx;
    dx.emplace_back("x", tIdx, 0, -1);
    dx.emplace_back("x", tIdx-1, 0, 1);
    dx.emplace_back("u", tIdx, 0, std::any_cast<double>(dt));
    return dx;
}

ETOL::scalar_t dyConstraint(F_ARGS) {
    int tIdx = std::any_cast<int>(k);
    std::vector<ETOL::glpk_t> dy;
    dy.emplace_back("x", tIdx, 1, -1);
    dy.emplace_back("x", tIdx-1, 1, 1);
    dy.emplace_back("u", tIdx, 1, std::any_cast<double>(dt));
    return dy;
}

ETOL::scalar_t absConstraint(F_ARGS) {
    int tIdx = std::any_cast<int>(k);
    std::vector<ETOL::glpk_t> abs_c;
    std::vector<std::pair<std::vector<ETOL::glpk_t>, ETOL::glpk_bnd_t>>
                                                                        abs_cv;

    abs_c.emplace_back("u", tIdx, 0, 1);
    abs_c.emplace_back("u", tIdx, 2, 1);
    abs_cv.emplace_back(abs_c, ETOL::glpk_bnd_t(0.0, GLPK_UNBOUND));
    abs_c.clear();

    abs_c.emplace_back("u", tIdx, 0, -1);
    abs_c.emplace_back("u", tIdx, 2, 1);
    abs_cv.emplace_back(abs_c, ETOL::glpk_bnd_t(0.0, GLPK_UNBOUND));
    abs_c.clear();

    abs_c.emplace_back("u", tIdx, 1, 1);
    abs_c.emplace_back("u", tIdx, 3, 1);
    abs_cv.emplace_back(abs_c, ETOL::glpk_bnd_t(0.0, GLPK_UNBOUND));
    abs_c.clear();

    abs_c.emplace_back("u", tIdx, 1, -1);
    abs_c.emplace_back("u", tIdx, 3, 1);
    abs_cv.emplace_back(abs_c, ETOL::glpk_bnd_t(0.0, GLPK_UNBOUND));
    abs_c.clear();

    return abs_cv;
}

ETOL::f_t obsConstraint(ETOL::TrajectoryOptimizer* t) {
    obstacles = ETOL::eGLPK::createObstacles(t);
    ETOL::f_t o = [](F_ARGS) {
        int tIdx = std::any_cast<int>(k);
        std::vector<ETOL::glpk_t> abs_obs;
        std::vector<std::pair<std::vector<ETOL::glpk_t>,
            ETOL::glpk_bnd_t>> abs_obsv, abs_obsv_bot, abs_obsv_top,
                                    abs_obsv_sum;
        int ii = 0;
        for (auto bd : obstacles) {
            int jj = 0;
            for (const auto& lower : bd.first) {
                abs_obsv_bot = ETOL::eGLPK::addObstacleSides(lower, -1,
                                            "bot", ii, jj, tIdx, pnames, BIGM);
                abs_obsv.insert(abs_obsv.end(),
                        abs_obsv_bot.begin(), abs_obsv_bot.end());

                ETOL::seg_t upper = bd.second.at(jj);
                abs_obsv_top = ETOL::eGLPK::addObstacleSides(upper, 1,
                                            "top", ii, jj, tIdx, pnames, BIGM);
                abs_obsv.insert(abs_obsv.end(),
                            abs_obsv_top.begin(), abs_obsv_top.end());

                abs_obsv_sum = ETOL::eGLPK::addObstacleSum("^(?:bot|top)",
                                                    ii, jj, tIdx, pnames, 0);
                abs_obsv.insert(abs_obsv.end(),
                            abs_obsv_sum.begin(), abs_obsv_sum.end());
                jj++;
            }
            ii++;
        }
        return abs_obsv;
    };
    return o;
}

ETOL::f_t saaConstraint(ETOL::TrajectoryOptimizer* t) {
    tracks = ETOL::eGLPK::createTracks(t, NSIDES);
    ETOL::f_t saa = [](F_ARGS){
        auto delta = std::any_cast<double>(dt);
        int tIdx = std::any_cast<int>(k);
        double t = delta * tIdx;
        std::vector<ETOL::glpk_t> abs_saa;
        std::vector<std::pair<std::vector<ETOL::glpk_t>,
                    ETOL::glpk_bnd_t>> abs_saav, abs_saav_sum;
        int ii = 0;
        for (auto track : tracks) {
            double radius = track.radius;
            std::vector<double> tvec;
            for_each(track.trajectory.begin(), track.trajectory.end(), [&tvec]
                    (const ETOL::traj_elem_t& data){
                double value = data.first;
                tvec.push_back(value);
            });
            std::vector<double>::iterator tlo_it;
            tlo_it = std::lower_bound(tvec.begin(), tvec.end(), t);
            if (tlo_it != tvec.begin() && *tlo_it > t)
                tlo_it = std::prev(tlo_it);
            size_t jj = distance(tvec.begin(), tlo_it);

            ETOL::traj_t::iterator  curr, next;
            curr = track.trajectory.begin();
            std::advance(curr, jj);
            next = curr;
            if (tlo_it != prev(tvec.end()))
                std::advance(next, 1);

            double delt =  next->first - curr->first;
            double rhs;
            for (int kk = 0; kk < NSIDES; kk++) {
                std::string r(ETOL::eGLPK::getParamName(
                    ETOL::eGLPK::getParamName("d", ii, jj, kk), tIdx));
                int i = 0;
                for (const auto& pname : pnames) {
                    if (re2::RE2::FullMatch(pname, r)) {
                        abs_saa.emplace_back(pname, tIdx, -1, BIGM);
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

                abs_saa.emplace_back("x", tIdx, 0, c0);  // x
                abs_saa.emplace_back("x", tIdx, 1, c1);  // y

                abs_saav.emplace_back(abs_saa,
                        ETOL::glpk_bnd_t(rhs, GLPK_UNBOUND));
                abs_saa.clear();
            }
            abs_saav_sum = ETOL::eGLPK::addObstacleSum("^d", ii, jj, tIdx,
                                                                pnames, NSIDES);
            abs_saav.insert(abs_saav.end(), abs_saav_sum.begin(),
                                                        abs_saav_sum.end());
            ii++;
        }
        return abs_saav;
    };
    return saa;
}
