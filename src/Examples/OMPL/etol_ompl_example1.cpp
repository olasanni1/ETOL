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
 * Solves the following optimization problem
 *       minimize   path length
 *       subject to x_dot = u_0, y_dot = u_1,
 *                  (x_k, y_k) Static Obstacle
 *                  (x_k, y_k) Moving Obstacle
 ******************************************************************************/

#include <ETOL/eOMPL.hpp>

//// Function prototypes
void dynConstraint(OMPL_DYN_ARGS);
ompl::base::OptimizationObjectivePtr optObjFunc(
        const ompl::base::SpaceInformationPtr& si);

int main(int argc, char** argv) {
    std::cout << "OMPL" << std::endl;

    ETOL::TrajectoryOptimizer* t;
    ETOL::eOMPL tg = ETOL::eOMPL();
    t = &tg;

    t->loadConfigs(argv[1]);
    // t->printConfigs();

    // see a list of planners here: https://ompl.kavrakilab.org/planners.html
    tg.setPlanner("SST");  // sparse stable RRT
    tg.setGradient(&dynConstraint);
    tg.pdef_->setOptimizationObjective(optObjFunc(tg.si_));

    // Setup
    t->setup();

    // Solve
    t->solve();

    // Result
    printf("\n!!!!!!!!!!!!!!!!!Results!!!!!!!!!!!!!!!!!\n");
    printf("Minimization Score:\t%f\n", t->getScore());
    printf("State variables saved in %s\n", ETOL::TrajectoryOptimizer::save(
            t->getXtraj(), "state_ompl1.csv").c_str());
    printf("Control variables saved in %s\n", ETOL::TrajectoryOptimizer::save(
            t->getUtraj(), "control_ompl1.csv").c_str());
//    std::cout << "Plotting..." << std::endl;
//    ETOL::TrajectoryOptimizer::plotXY_wExclZones(t->getXtraj(),
//            t->getObstacles());

    // Release resources e.g. memory
    t->close();
    printf("\n!!!!!!!!!!!!!!Graceful Exit!!!!!!!!!!!!!!\n");
    return 0;
}

void dynConstraint(OMPL_DYN_ARGS) {
    const double *u = control->
            as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    // Zero out xdot
    xdot.resize(x.size(), 0);

    // single integrator dynamics
    xdot[0] = u[0];
    xdot[1] = u[1];
}

ompl::base::OptimizationObjectivePtr optObjFunc(
        const ompl::base::SpaceInformationPtr& si) {
    return ompl::base::OptimizationObjectivePtr(
            new ompl::base::PathLengthOptimizationObjective(si));
}
