/******************************************************************************!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 6 June 2020
 * @version 1.0.0
 * @brief Loads a ETOL Config XML file and uses Dymos as the eSolver
 * @section DESCRIPTION
 *    minimize    integral (u0(t) *u0(t) +  u1(t) *u1(t)) from t0 to tf
 *    subject to  dx/dt = u0(t)                     for t in {t0,tf}
 *                dy/dt = u1(t)                     for t in {t0,tf}
 *                <x(t) y(t)> notin Obstacle        for t in {t0,tf}
 *                <x(t),y(t)> notin Moving Obstacle for t in {t0,tf}
 ******************************************************************************/

#include <pybind11/pybind11.h>
#include <utility>
#include <iostream>
#include <ETOL/eDymos.hpp>

namespace py = pybind11;
using namespace py::literals;   // to bring in the `_a` literal   // NOLINT

// Global Data
size_t exz_comp_idx(0), exz_partial_idx(0);
size_t mexz_comp_idx(0), mexz_partial_idx(0);
std::vector<std::tuple<double, double, double, double>> exz;
/**< A vector of ellipse tuples (xc, yc, radius, rotation angle) */
std::vector<std::tuple<double, ETOL::state_t, std::vector<ETOL::state_t>>> mexz;
/**< A vector of tuple (radius, time vector, vector of state time histories) */

bool editAlgo(ETOL::TrajectoryOptimizer *t);
ETOL::scalar_t objFunction(F_ARGS);
ETOL::scalar_t dxConstraint(F_ARGS);
ETOL::scalar_t dyConstraint(F_ARGS);
ETOL::scalar_t obsConstraint(F_ARGS);
ETOL::scalar_t saaConstraint(F_ARGS);
void setExz(ETOL::TrajectoryOptimizer *t);
void setMexz(ETOL::TrajectoryOptimizer *t);
double linear_interpolation(const double&, const ETOL::state_t&,
        const ETOL::state_t&);

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage: %s <ETOL configuration xml filepath>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    ETOL::TrajectoryOptimizer* t;
    ETOL::eDymos td = ETOL::eDymos();
    t = &td;

    t->loadConfigs(argv[1]);
    printf("\nLoaded Configs\n");
    t->printConfigs();

    // Set the objective function the is minimized
    ETOL::f_t obj = &objFunction;
    t->setObjective(&obj);

    // Set state time derivatives
    ETOL::f_t dx = &dxConstraint;
    ETOL::f_t dy = &dyConstraint;
    t->setGradient({&dx, &dy});

    // Add exclusion zone constraints
    setExz(t);
    ETOL::f_t obs_func = &obsConstraint;
    std::vector<ETOL::f_t*> con(exz.size(), &obs_func);

    // Add moving exclusion zone constraints
    setMexz(t);
    ETOL::f_t saa_func = &saaConstraint;
    std::vector<ETOL::f_t*> mexz_con(mexz.size(), &saa_func);
    con.insert(con.end(), mexz_con.begin(), mexz_con.end());

    // Set the constraints
    t->setConstraints(con);

    // Setup
    t->setup();

    if (!editAlgo(t)) {
        printf("Edit eDymos failed");
        return EXIT_FAILURE;
    }

    // Debug
    t->debug();

    // Solve
    t->solve();

    // Results
    printf("\n!!!!!!!!!!!!!!!!!Results!!!!!!!!!!!!!!!!!\n");
    printf("Minimization Score:\t%f\n", t->getScore());
    printf("State variables saved in %s\n", ETOL::TrajectoryOptimizer::save(
            t->getXtraj(), "state_dymos1.csv").c_str());
    printf("Control variables saved in %s\n", ETOL::TrajectoryOptimizer::save(
                t->getUtraj(), "control_dymos1.csv").c_str());
//    printf("Animating...\n");
//    fflush(stdout);
//    ETOL::TrajectoryOptimizer::animate2D(t->getXtraj(), 20, false,
//            "animate.mp4", t->getObstacles(), t->getTracks(), 0, 1, "Animate");
    t->close();
    printf("\n!!!!!!!!!!!!!!Graceful Exit!!!!!!!!!!!!!!\n");
    return EXIT_SUCCESS;
}

// Changes the optimizer's state variable starting point
bool editAlgo(ETOL::TrajectoryOptimizer *t) {
    ETOL::eDymos* ptr = dynamic_cast<ETOL::eDymos*>(t);
    if (ptr) {
        double tf = ptr->getDt() * ptr->getNSteps();
        double x0_t0 = ptr->getX0().front();
        double x0_tf = ptr->getXf().front();
        double x1_t0 = ptr->getX0().back();
        double x1_tf = ptr->getXf().back();
        std::string xprefix = "traj.phase0.states:";
        ptr->getAlg()[(xprefix + "x0").c_str()] =
                ptr->getProb().attr("interpolate")(
                "xs"_a = py::make_tuple(0.0, tf/2., tf),
                "ys"_a = py::make_tuple(x0_t0, x0_t0, x0_tf),
                "nodes"_a = "state_input");
        ptr->getAlg()[(xprefix + "x1").c_str()] =
                ptr->getProb().attr("interpolate")(
                "xs"_a = py::make_tuple(0.0, tf/2., tf),
                "ys"_a = py::make_tuple(x1_t0, x1_tf, x1_tf),
                "nodes"_a = "state_input");
        return true;
    }
    return false;
}

ETOL::scalar_t objFunction(F_ARGS) {
    bool compute_partials = (!pnames.at(0).empty());

    size_t length = compute_partials ? x.size() + u.size() : 1;

    std::vector<double> result(length, 0.);

    try {
        double u0  = std::any_cast<double>(u.at(0));
        double u1  = std::any_cast<double>(u.at(1));
        if (!compute_partials) {
            result.at(0) = u0*u0 + u1*u1;
        } else {
            result.at(2) = 2*u0;
            result.at(3) = 2*u1;
        }
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return result;
}

ETOL::scalar_t dxConstraint(F_ARGS) {
    bool compute_partials = (!pnames.at(0).empty());

    size_t length = compute_partials ? x.size() + u.size() : 1;

    std::vector<double> result(length, 0.);
    try {
        double u0 = std::any_cast<double>(u.at(0));
        if (!compute_partials) {
            result.at(0) = u0;
        } else {
            result.at(2) = 1.;
        }
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return result;
}

ETOL::scalar_t dyConstraint(F_ARGS) {
    bool compute_partials = (!pnames.at(0).empty());

    size_t length = compute_partials ? x.size() + u.size() : 1;

    std::vector<double> result(length, 0.);
    try {
        double u1 = std::any_cast<double>(u.at(1));
        if (!compute_partials) {
            result.at(0) = u1;
        } else {
            result.at(3) = 1.;
        }
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return result;
}

ETOL::scalar_t obsConstraint(F_ARGS) {
    bool compute_partials = (!pnames.at(0).empty());

    size_t length = compute_partials ? x.size() + u.size() : 1;

    std::vector<double> result(length, 0.);
    try {
        double xk  = std::any_cast<double>(x.at(0));
        double yk  = std::any_cast<double>(x.at(1));
        if (!compute_partials) {
            double xc = std::get<0>(exz.at(exz_comp_idx));
            double yc = std::get<1>(exz.at(exz_comp_idx));
            double radsq = std::get<2>(exz.at(exz_comp_idx));
            double tt = std::get<3>(exz.at(exz_comp_idx));

            double dx = xk - xc;
            double dy = yk - yc;
            double delx = cos(tt) * dx  - sin(tt) * dy;
            double dely = sin(tt) * dx  + cos(tt) * dy;
            double asq = radsq;
            double bsq = .2 * radsq;

            result.at(0) = asq*bsq - (bsq*pow(delx, 2.) +asq*pow(dely, 2.));

            result.at(0) = std::exp(result.at(0)) - 1;

            exz_comp_idx = ++exz_comp_idx % exz.size();
        } else {
            double xc = std::get<0>(exz.at(exz_partial_idx));
            double yc = std::get<1>(exz.at(exz_partial_idx));
            double radsq = std::get<2>(exz.at(exz_partial_idx));
            double tt = std::get<3>(exz.at(exz_partial_idx));

            double dx = xk - xc;
            double dy = yk - yc;
            double delx = cos(tt) * dx  - sin(tt) * dy;
            double dely = sin(tt) * dx  + cos(tt) * dy;
            double asq = radsq;
            double bsq = .2 * radsq;

            result.at(0) = -2. * (bsq * delx * cos(tt) + asq * dely * sin(tt));
            result.at(1) = -2. * (-bsq * delx * sin(tt) + asq * dely * cos(tt));

            double val = std::exp(asq*bsq -
                                    (bsq*pow(delx, 2.) + asq*pow(dely, 2.)));

            result.at(0) = std::exp(val)* result.at(0);
            result.at(1) = std::exp(val)* result.at(1);

            exz_partial_idx = ++exz_partial_idx % exz.size();
        }
    } catch(std::bad_any_cast& e) {
        std::cout << "Error in obs function" << std::endl;
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return result;
}

ETOL::scalar_t saaConstraint(F_ARGS) {
    bool compute_partials = (!pnames.at(0).empty());

    size_t length = compute_partials ? x.size() + u.size() : 1;

    std::vector<double> result(length, 0.);
    try {
        double tk  = std::any_cast<double>(k);
        double xk  = std::any_cast<double>(x.at(0));
        double yk  = std::any_cast<double>(x.at(1));
        if (!compute_partials) {
            double rad = std::get<0>(mexz.at(mexz_comp_idx));
            ETOL::state_t& tvec = std::get<1>(mexz.at(mexz_comp_idx));
            ETOL::state_t& xvec = std::get<2>(mexz.at(mexz_comp_idx)).at(0);
            ETOL::state_t& yvec = std::get<2>(mexz.at(mexz_comp_idx)).at(1);

            double xc = linear_interpolation(tk, tvec, xvec);
            double yc = linear_interpolation(tk, tvec, yvec);

            double dx = xk - xc;
            double dy = yk - yc;
            double distsq = dx*dx + dy*dy;

            result.at(0) = std::exp(rad*rad - distsq) -1;

            mexz_comp_idx = ++mexz_comp_idx % mexz.size();
        } else {
            double rad = std::get<0>(mexz.at(mexz_comp_idx));
            ETOL::state_t& tvec = std::get<1>(mexz.at(mexz_partial_idx));
            ETOL::state_t& xvec = std::get<2>(mexz.at(mexz_partial_idx)).at(0);
            ETOL::state_t& yvec = std::get<2>(mexz.at(mexz_partial_idx)).at(1);

            double xc = linear_interpolation(tk, tvec, xvec);
            double yc = linear_interpolation(tk, tvec, yvec);

            double dx = xk - xc;
            double dy = yk - yc;
            double distsq = dx*dx + dy*dy;
            result.at(0) = -2.*dx*std::exp(rad*rad - distsq);
            result.at(1) = -2.*dy*std::exp(rad*rad - distsq);

            mexz_partial_idx = ++mexz_partial_idx % mexz.size();
        }
    } catch(std::bad_any_cast& e) {
        std::cout << "Error in saa function" << std::endl;
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return result;
}

void setExz(ETOL::TrajectoryOptimizer* t) {
    exz_comp_idx = 0;
    exz_partial_idx = 0;
    exz = {};
    for (auto obs : *t->getObstacles_Raw()) {
        std::list<ETOL::corner_t>::iterator curr = obs.begin();
        std::list<ETOL::corner_t>::iterator next = std::next(curr, 1);
        for (size_t i=0; i < obs.size(); i++) {
            double xa = curr->at(0);
            double ya = curr->at(1);
            double xb = next->at(0);
            double yb = next->at(1);
            double xc = (xb + xa)/2.0;
            double m = (yb - ya)/(xb-xa);
            double yc = ya + m*(xc - xa);
            double radsq = pow(xc - xa, 2.0) + pow(yc - ya, 2.0);
            double tt = -1.0 * atan2(yc - ya, xc - xa);
            std::tuple<double, double, double, double> par({xc, yc, radsq, tt});
            exz.push_back(par);
            curr++;
            next++;
            if (next == obs.end())
                next = obs.begin();
        }
    }
}

void setMexz(ETOL::TrajectoryOptimizer * t) {
    mexz_comp_idx = 0;
    mexz_partial_idx = 0;
    mexz = {};
    for_each(t->getTracks()->begin(), t->getTracks()->end(),
    [](ETOL::track_t& track) {
        double radius = track.radius;
        ETOL::state_t tvec = {};
        std::vector<ETOL::state_t> trajs(track.trajectory.at(0).second.size(),
                ETOL::state_t());
        size_t i(0);
        for_each(track.trajectory.begin(), track.trajectory.end(),
        [&tvec, &trajs, &i](ETOL::traj_elem_t& elem){
            if (i == 0)
                tvec.push_back(elem.first);
            size_t j(0);
            for_each(elem.second.begin(), elem.second.end(),
            [&trajs, &j](double& x){
                    trajs.at(j++).push_back(x);
            });
        });
        mexz.push_back({radius, tvec, trajs});
    });
}

double linear_interpolation(const double& tval, const ETOL::state_t& tvec,
        const ETOL::state_t& xvec) {
    int j(0);

    if (tval < tvec.front()) {
        j = 0;
    } else if (tval > tvec.back()) {
        j = tvec.size() -2;
    } else {
        for (size_t i(0); i < (tvec.size()-1); i++) {
            if (tval >= tvec.at(i) && tval <= tvec.at(i+1)) {
                j = i;
            }
        }
    }

    return (xvec.at(j) + (tval - tvec.at(j)) * (xvec.at(j+1) - xvec.at(j)) /
            (tvec.at(j+1) - tvec.at(j)));
}
