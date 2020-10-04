/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 5 May 2020
 * @version 1.0.1
 * @brief Loads a ETOL Config XML file and uses PSOPT to solve a ETOL
 * @section DESCRIPTION
 * Solves the following OCP
 *    minimize    integral (u0(t) *u0(t) +  u1(t) *u1(t)) from t0 to tf
 *    subject to  dx/dt = u0(t)                     for t in {t0,tf}
 *                dy/dt = u1(t)                     for t in {t0,tf}
 *                <x(t) y(t)> notin Obstacle        for t in {t0,tf}
 *                <x(t),y(t)> notin Moving Obstacle    for t in {t0,tf}
 ******************************************************************************/

#include <iostream>
#include <ETOL/ePSOPT.hpp>

// Global Data
std::vector<ETOL::border_t>* obstacles;
std::vector<std::pair<double, MatrixXd>> tracks = {};

// Function prototypes
void editAlgo(ETOL::TrajectoryOptimizer*);
ETOL::scalar_t objFunction(F_ARGS);
ETOL::scalar_t dxdt(F_ARGS);
ETOL::scalar_t dydt(F_ARGS);
ETOL::f_t obsConstraint(ETOL::TrajectoryOptimizer*);
ETOL::f_t saaConstraint(ETOL::TrajectoryOptimizer* t);
std::string paramName(const std::string, const size_t&, const size_t&,
                        const size_t&);

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage: %s <ETOL configuration xml filepath>\n", argv[0]);
        exit(EXIT_FAILURE);
    }
    ETOL::TrajectoryOptimizer* t;
    ETOL::ePSOPT tp = ETOL::ePSOPT();
    t = &tp;

    t->loadConfigs(argv[1]);
    t->printConfigs();

    t->setMaximize(false);
    ETOL::f_t obj = &objFunction;
    t->setObjective(&obj);

    ETOL::f_t xdot = &dxdt;
    ETOL::f_t ydot = &dydt;
    t->setGradient({&xdot, &ydot});

    // Obstacle Constraint
    ETOL::f_t obs, saa;
    obs = obsConstraint(t);
    saa = saaConstraint(t);
    t->setConstraints({&obs, &saa});

    // Setup
    t->setup();
    editAlgo(t);
    t->debug();
    // Solve
    t->solve();

    // Results
    printf("\n!!!!!!!!!!!!!!!!!Results!!!!!!!!!!!!!!!!!\n");
    printf("Minimization Score:\t%f\n", t->getScore());
    printf("Animating...\n");
    ETOL::TrajectoryOptimizer::animate2D(t->getXtraj(), 10 , false,
            "animation.mp4", t->getObstacles(), t->getTracks());

    // Gracefully release resources e.g. memory, file handles, etc
    t->close();

    return EXIT_SUCCESS;
}

void editAlgo(ETOL::TrajectoryOptimizer*t) {
    ETOL::ePSOPT* ptr = dynamic_cast<ETOL::ePSOPT*>(t);
    if (ptr) {
        Alg* algo = ptr->getAlgorithm();
        algo->nlp_method                    = "IPOPT";
        algo->defect_scaling                = "jacobian-based";
        algo->ipopt_max_cpu_time            = 100;
        algo->mr_max_iterations             = 40;
        algo->ode_tolerance                 = 1.e-3;
    } else {
        std::cout << "EditAlgo only works for ePSOPT!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

ETOL::scalar_t objFunction(F_ARGS) {
    adouble obj;
    try {
        adouble u0, u1;
        u0 = *std::any_cast<adouble*>(u.at(0));
        u1 = *std::any_cast<adouble*>(u.at(1));
        obj = u0 *u0 + u1*u1;
    } catch(std::bad_any_cast& e ) {
        cout << "Error in obj" << std::endl;
        cout << e.what() << endl;;
        exit(EXIT_FAILURE);
    }
    return obj;
}

ETOL::scalar_t dxdt(F_ARGS) {
    adouble* xdot;
    try {
        xdot = std::any_cast<adouble*>(u.at(0));
    } catch(std::bad_any_cast& e ) {
        cout << "Error in dxdt" << std::endl;
        cout << e.what() << endl;;
        exit(EXIT_FAILURE);
    }
    return *xdot;
}

ETOL::scalar_t dydt(F_ARGS) {
    adouble* ydot;
    try {
        ydot = std::any_cast<adouble*>(u.at(1));
    } catch(std::bad_any_cast& e ) {
        cout << "Error in dydt" << std::endl;
        cout << e.what() << endl;;
        exit(EXIT_FAILURE);
    }
    return *ydot;
}

ETOL::f_t obsConstraint(ETOL::TrajectoryOptimizer* t) {
    ETOL::f_t obs;
    double tspan = t->getDt() * t->getNSteps();
    obstacles = t->getObstacles_Raw();
    size_t i(0);
    for (auto obs : *t->getObstacles_Raw()) {
        size_t j(0);
        for (auto bd : obs)
            t->addParams({std::pair<PARAM_PAIR>(paramName("side", i, j++, 0),
                        {ETOL::var_t::CONTINUOUS, -1000., 0., 0., tspan})});
        i++;
    }

    obs = [](F_ARGS) {
        ETOL::fout_psopt_t fout;
        try {
            adouble xk, yk;
            xk = *std::any_cast<adouble*>(x.at(0));
            yk = *std::any_cast<adouble*>(x.at(1));
            for (auto bd : *obstacles) {
                std::list<ETOL::corner_t>::iterator curr = bd.begin();
                std::list<ETOL::corner_t>::iterator next = std::next(
                                                                    curr, 1);
                for (size_t i=0; i < bd.size(); i++) {
                    double xa = curr->at(0);
                    double ya = curr->at(1);
                    double xb = next->at(0);
                    double yb = next->at(1);
                    double xc = (xb + xa)/2.;
                    double    m = (yb - ya)/(xb-xa);
                    double yc = ya + m*(xc - xa);
                    double radsq = pow(xc - xa, 2.0) + pow(yc - ya, 2.0);
                    double tt = -1.0 * atan2(yc - ya, xc - xa);

                    adouble dx = xk - xc;
                    adouble dy = yk - yc;
                    adouble delx = cos(tt) * dx  - sin(tt) * dy;
                    adouble dely = sin(tt) * dx  + cos(tt) * dy;
                    double asq = radsq;
                    double bsq = .2 * radsq;
                    adouble out;
                    out = asq*bsq - (bsq*pow(delx, 2.) +asq*pow(dely, 2.));
                    fout.push_back(out);
                    curr++;
                    next++;
                    if (next == bd.end())
                        next = bd.begin();
                }
            }
        } catch(std::bad_any_cast& e ) {
            std::cout << "Error in obs" << std::endl;
            std::cout << e.what() << std::endl;;
            exit(EXIT_FAILURE);
        }
        return fout;
    };
    return obs;
}

ETOL::f_t saaConstraint(ETOL::TrajectoryOptimizer* t) {
    ETOL::f_t saa;
    tracks.clear();
    double tspan = t->getDt() * t->getNSteps();
    // Construct a table for each track e.g.
    // time state1     state2     state 3 ...
    // 0    1        2        3
    // 1.5    2        4        5
    size_t i(0);
    for (auto track : *t->getTracks()) {
        MatrixXd tbl = zeros(track.trajectory.size(),
                    track.trajectory.front().second.size()+1);
        size_t j(0);
        for (auto data : track.trajectory) {
            size_t k(0);
            tbl(j, k++) = data.first;
            for (auto state : data.second) {
                tbl(j, k++) = state;
            }
            j++;
        }
        tracks.push_back(std::pair<double, DMatrix>(track.radius, tbl));
        t->addParams({std::pair<PARAM_PAIR>(paramName("ball", i, 0, 0),
                        {ETOL::var_t::CONTINUOUS, -1000., 0., 0., tspan})});
        i++;
    }

    saa = [](F_ARGS){
        ETOL::fout_psopt_t fout;
        try {
            adouble xk, yk;
            xk = *std::any_cast<adouble*>(x.at(0));
            yk = *std::any_cast<adouble*>(x.at(1));
            adouble tval = *std::any_cast<adouble*>(k);
            for (auto track : tracks) {
                adouble xc, yc;
                MatrixXd h1     = track.second.col(0);
                MatrixXd h2     = track.second.col(1);
                MatrixXd h3     = track.second.col(2);
                linear_interpolation(&xc, tval, h1,
                        h2, track.second.rows());
                linear_interpolation(&yc, tval, h1,
                        h3, track.second.rows());

                adouble dx = xk - xc;
                adouble dy = yk - yc;
                adouble dist = pow(dx, 2.0) + pow(dy, 2.0);
                adouble circ = dist * (-1.) + pow(track.first, 2.0);
                fout.push_back(circ);
            }
        } catch(std::bad_any_cast& e ) {
            std::cout << "Error in saa" << std::endl;
            std::cout << e.what() << endl;;
            exit(EXIT_FAILURE);
        }
        return fout;
    };

    return saa;
}

string paramName(const string name, const size_t& i, const size_t& j,
                    const size_t& k) {
    return (name + "_" + std::to_string(i) + "_" + std::to_string(j) +
            "_" + std::to_string(k));
}
