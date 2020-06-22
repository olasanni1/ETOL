/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 1 May 2020
 * @version 1.0.0
 * @brief Loads ETOL XML configs, edits it, and saves a ETOL XML file.
 * @section DESCRIPTION
 * Adds a constant to each state's initial and terminal value.
 * This is accomplished without requiring ETOL to perform an optimization!!!
 ******************************************************************************/

#include <ETOL/eGurobi.hpp>

#define XSHIFT 10.0

using ETOL::TrajectoryOptimizer;
using ETOL::eGurobi;

int main(int argc, char** argv) {
    if (!(argc == 2 || argc == 3)) {
        printf("Usage: %s <ETOL configuration xml filepath> ", argv[0]);
        printf("or <old path> <new path> \n");
        exit(EXIT_FAILURE);
    }

    TrajectoryOptimizer* t;
    eGurobi tg = eGurobi();
    t = &tg;

    t->loadConfigs(argv[1]);
    printf("\nLoaded Configs\n");
    t->printConfigs();


    // Shifts the initial and terminal values by XSHIFT
    for_each(t->getX0().begin(), t->getX0().end(), [](double &x){
        x+=XSHIFT;
    });
    for_each(t->getXf().begin(), t->getXf().end(), [](double &x){
        x+=XSHIFT;
    });

    t->saveConfigs(argv[argc - 1]);
    printf("New Configs\n");
    t->printConfigs();

    t->close();

    return EXIT_SUCCESS;
}
