/******************************************************************************!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 6 June 2020
 * @version 1.0.0
 * @brief Loads a ETOL Config XML file and uses Dymos to solve a ETOL
 * @section DESCRIPTION
 *    minimize    integral (u0(t) *u0(t) +  u1(t) *u1(t)) from t0 to tf
 *    subject to  dx/dt = u0(t)                     for t in {t0,tf}
 *                dy/dt = u1(t)                     for t in {t0,tf}
 *                <x(t) y(t)> notin Obstacle        for t in {t0,tf}
 *                <x(t),y(t)> notin Moving Obstacle    for t in {t0,tf}
 ******************************************************************************/

#include <iostream>
#include <ETOL/eDymos.hpp>

ETOL::scalar_t objFunction(F_ARGS);
ETOL::scalar_t dxConstraint(F_ARGS);
ETOL::scalar_t dyConstraint(F_ARGS);

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

    ETOL::f_t obj = &objFunction;
    t->setObjective(&obj);

    ETOL::f_t dx = &dxConstraint;
    ETOL::f_t dy = &dyConstraint;
    t->setGradient({&dx, &dy});

    // Setup
    t->setup();

    // Debug
    t->debug();

    // Solve
    t->solve();

    t->close();

    return EXIT_SUCCESS;
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
            result.at(x.size()) = 2 * u0;
            result.at(x.size() + 1) = 2 * u0;
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
        if (!compute_partials)
            result.at(0) = std::any_cast<double>(u.at(0));
        else
            result.at(x.size()) = 1.;
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
        if (!compute_partials)
            result.at(0) = std::any_cast<double>(u.at(1));
        else
            result.at(x.size() + 1) = 1.;
    } catch(std::bad_any_cast& e) {
        std::cout << e.what() << std::endl;;
        exit(EXIT_FAILURE);
    }
    return result;
}
