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

    // Setup
    t->setup();

    // Debug
    t->debug();

    // Solve
    t->solve();

    t->close();

    return EXIT_SUCCESS;
}
