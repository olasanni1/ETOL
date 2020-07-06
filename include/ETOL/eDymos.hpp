/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @author Emre Yilmaz
 * @author Mark Kotwicz
 * @date 4 June 2020
 * @version 1.0.0
 * @brief The header file for the dymos implementation for ETOL
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EDYMOS_HPP_
#define INCLUDE_ETOL_EDYMOS_HPP_

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <string>
#include <ETOL/TrajectoryOptimizer.hpp>

namespace ETOL {

class __attribute__((visibility("default")))
eDymos : public pybind11::scoped_interpreter,
                    public TrajectoryOptimizer {
 public:
    eDymos();

    virtual ~eDymos() {}

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

    // Static Functions
    static std::string getStateName(const size_t& sIdx);

    static std::string getDerivName(const size_t& sIdx);

    static std::string getControlName(const size_t& sIdx);

    static std::string getPathConstraintName(const size_t& pIdx);

    static pybind11::dict dymosCompute(void*, pybind11::dict);

    static pybind11::dict dymosComputePartials(void*, pybind11::dict);

 protected:
    pybind11::object _np;               /**< Python numpy import */
    pybind11::object _om;               /**< Python openmdao.api import */
    pybind11::object _dm;               /**< Python dymos import */
    pybind11::object _prob;             /**< Dymos problem parameters */
    pybind11::object _alg;              /**< Dymos algorithm parameters */
    pybind11::object _sol;              /**< Dymos aolution set */

 private:
    void setAlg();
    void setProb();
    void setSol();
    void setGuess();
    void getTraj();
    void addODE();
};

} /* namespace ETOL */

#endif  // INCLUDE_ETOL_EDYMOS_HPP_
