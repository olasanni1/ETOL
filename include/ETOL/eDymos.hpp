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
#include <ETOL/eDymos_Types.hpp>
#include <ETOL/TrajectoryOptimizer.hpp>

namespace ETOL {


class __attribute__((visibility("default")))
eDymos : public pybind11::scoped_interpreter,
                    public TrajectoryOptimizer {
 public:
    eDymos();

    virtual ~eDymos(){};

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


    // Static Functions

 protected:

 private:
    void getTraj();

};

} /* namespace ETOL */

#endif /* INCLUDE_ETOL_EDYMOS_HPP_ */
