/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 6 May 2020
 * @version 0.1.0
 * @brief The header file for the PSOPT implementation for ETOL
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EPSOPT_HPP_
#define INCLUDE_ETOL_EPSOPT_HPP_

#include <vector>
#include <string>
#include <ETOL/ePSOPT_Types.hpp>
#include <ETOL/TrajectoryOptimizer.hpp>

namespace ETOL {

class ePSOPT : public TrajectoryOptimizer{
 public:
    /**
     * @brief Constructor for a PSOPT interface
     */
    ePSOPT();

    virtual ~ePSOPT() {}

    /**
     * @brief Constructs an optimization problem from settings
     */
    void setup();

    /**
     * @brief Solves the trajectory optimization problem
     */
    void solve();

    /**
     * @brief Generates debug information
     */
    void debug();

    /**
     * @brief Closes all dynamically allocated resources
     */
    void close();

    // Getters and Setters
    /**
     * @brief Get pointer to PSOPT algorithm
     * @return the pointer to the PSOPT algorithm
     */
    Alg* getAlgorithm();

    /**
     * @brief Get pointer to PSOPT solution
     * @return the pointer to the PSOPT solution
     */
    Sol* getSolution();

    /**
     * @brief Get pointer to the PSOPT problem
     * @return the pointer to the PSOPT problem
     */
    Prob* getProblem();

    // Static Functions
    /**
     * @brief Specifies the terminal costs
     * @param initial_states array of initial states within a phase
     * @param final_states array of final states within a phase
     * @param parameters array of static parameters within a phase
     * @param t0 initial phase time
     * @param tf final phase time
     * @param xad vector of scaled decision variables
     * @param iphase phase index (starting from 1)Phase index (starting from 1)
     * @param workspace pointer to PSOPT's workspace structure
     * @return
     */
    static adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                                adouble* parameters,
                                adouble& t0, adouble& tf,  // NOLINT
                                adouble* xad, int iphase, Workspace* workspace);

    /**
     * @brief Specifies the integrand costs
     * @param states array of instantaneous states within a phase
     * @param controls array of instantaneous controls
     * @param parameters array of static parameters within a phase
     * @param time instant of time within a phase
     * @param xad vector of scaled decision variables
     * @param iphase phase index (starting from 1)Phase index (starting from 1)
     * @param workspace pointer to PSOPT's workspace structure
     * @return
     */
    static adouble integrand_cost(adouble* states, adouble* controls,
                            adouble* parameters,
                            adouble& time,  // NOLINT
                            adouble* xad,
                            int iphase, Workspace* workspace);

    /**
     * @brief Specifies the time derivatives of the states
     * @param derivatives array of intantaneous state derivatives
     * @param path array of instantaneous path constraint values within a phase
     * @param states array of instantaneous states within a phase
     * @param controls array of instantaneous controls
     * @param parameters array of static parameters within a phase
     * @param time instant of time within a phase
     * @param xad vector of scaled decision variables
     * @param iphase phase index (starting from 1)Phase index (starting from 1)
     * @param workspace
     */
    static void dae(adouble* derivatives, adouble* path, adouble* states,
             adouble* controls, adouble* parameters,
             adouble& time,  // NOLINT
             adouble* xad, int iphase, Workspace* workspace);

    /**
     * @brief Specifies the values of the event constraint functions
     * @param e array of event constraints
     * @param initial_states array of initial states within a phase
     * @param final_states Array of final states within a phase
     * @param parameters array of static parameters within a phase
     * @param t0 initial phase time
     * @param tf final phase time
     * @param xad vector of scaled decision variables
     * @param iphase phase index (starting from 1)Phase index (starting from 1)
     * @param workspace pointer to PSOPT's workspace structure
     */
    static void events(adouble* e, adouble* initial_states,
            adouble* final_states, adouble* parameters,
            adouble& t0, adouble& tf, // NOLINT
            adouble* xad, int iphase, Workspace* workspace);

    /**
     * @brief Specifies the values of the phase linkage constraint functions
     * @param linkages array of linkage constraints
     * @param xad vector of scaled decision variables
     * @param workspace pointer to PSOPT's workspace structure
     */
    static void linkages(adouble* linkages, adouble* xad, Workspace* workspace);

 protected:
    Alg  _algorithm;                    /**< PSOPT algorithm */
    Sol  _solution;                     /**< PSOPT solution */
    Prob _problem;                      /**< PSOPT problem */

 private:
    /**
     * @brief Add the lower and upper bounds to PSOPT problem
     */
    void addBounds();

    /**
     Get the trajectory results from the PSOPT solution
     */
    void getTraj();
};

} /* namespace ETOL */

#endif  // INCLUDE_ETOL_EPSOPT_HPP_
