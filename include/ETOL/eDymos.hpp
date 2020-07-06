/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @author Emre Yilmaz <ae.emre.yilmaz@gmail.com>
 * @author Mark Kotwicz <mark.kotwicz@gmail.com>
 * @date 4 June 2020
 * @version 1.0.0
 * @brief The header file for the ETOL dymos implementation
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EDYMOS_HPP_
#define INCLUDE_ETOL_EDYMOS_HPP_

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <string>
#include <ETOL/TrajectoryOptimizer.hpp>

namespace ETOL {

/*!
 * @brief Uses Pybind11 to interface with the OpenMDAO dymos Python library
 */
class __attribute__((visibility("default")))
eDymos : public pybind11::scoped_interpreter, public TrajectoryOptimizer {
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
    /**
     * @brief Provide a consistent name for state variables
     * @param sIdx an index
     * @return name assigned to a state variable
     */
    static std::string getStateName(const size_t& sIdx);

    /**
     * @brief Provide a consistent name for state time derivatives
     * @param sIdx an index
     * @return name assigned to a state's time derivative
     */
    static std::string getDerivName(const size_t& sIdx);

    /**
     * @brief Provide a consistent name for control variables
     * @param sIdx an index
     * @return name assigned to a control variable
     */
    static std::string getControlName(const size_t& sIdx);

    /**
     * @brief Provide a consistent name for path variables
     * @param pIdx an index
     * @return name assigned to a path constraint
     */
    static std::string getPathConstraintName(const size_t& pIdx);

    /**
     * @brief A function that Dymos will use to compute outputs
     * @param h a pointer to the instantiated eDymos class
     * @param inputs a dictionary of inputs with state and control values
     * @return a dictionary of outputs that include state derivatives and path
     * constraint values
     */
    static pybind11::dict dymosCompute(void* h, pybind11::dict inputs);

    /**
     * @brief A function that Dymos will use to compute the Jacobian of outputs
     * @param h a pointer to the instantiated eDymos class
     * @param inputs a dictionary of inputs with state and control values
     * @return a dictionary of partials derivatives for state derivatives and
     * path constraints
     */
    static pybind11::dict dymosComputePartials(void* h, pybind11::dict inputs);

    // Getters and Setters
    const pybind11::object& getAlg() const;
    const pybind11::object& getDm() const;
    const pybind11::object& getNp() const;
    const pybind11::object& getOm() const;
    const pybind11::object& getProb() const;
    const pybind11::object& getSol() const;
    bool isCompressed() const;
    void setCompressed(bool compressed);
    bool isWithColoring() const;
    void setWithColoring(const bool withColoring);
    int getMaxMeshIter() const;
    void setMaxMeshIter(const int maxMeshIter);
    bool isMeshRefine() const;
    void setMeshRefine(const bool meshRefine);
    double getMeshTol() const;
    void setMeshTol(const double meshTol);
    int getNumSegments() const;
    void setNumSegments(const int numSegments);
    int getOrder() const;
    void setOrder(const int order);
    int getMaxIter() const;
    void setMaxIter(const int maxIter);
    std::string getOptimizer() const;
    void setOptimizer(std::string optimizer);

 protected:
    pybind11::object _np;           /**< Python numpy import */
    pybind11::object _om;           /**< Python openmdao.api import */
    pybind11::object _dm;           /**< Python dymos import */
    pybind11::object _prob;         /**< Dymos problem parameters */
    pybind11::object _alg;          /**< Dymos algorithm parameters */
    pybind11::object _sol;          /**< Dymos aolution set */
    std::string optimizer_;         /**< Optimizer used by PyOptSparse */
    bool mesh_refine_;              /**< Enables mesh refinement if true */
    double mesh_tol_;               /**< Mesh refinement tolerance */
    int max_mesh_iter_;             /**< Maximum number of mesh refinements */
    int max_iter_;                  /**< Maximum number of iterations */
    bool with_coloring_;            /**< Graph coloring of derivative matrix */
    bool compressed_;               /**< Compress the transcription */
    int num_segments_;              /**< Number transcription segments */
    int order_;                     /**< Transcription polynomial's order */

 private:
    /**
     * @brief Set the parameters for the optimizer
     */
    void setAlg();

    /**
     * @brief Set the parameters that define the problem
     */
    void setProb();

    /**
     * @brief Set a starting point for the optimizer
     */
    void setGuess();

    /**
     * @brief Get the state and control trajectories from Dymos
     */
    void getTraj();

    /**
     * @brief Create a python module that defines a Dymos ODE class
     */
    void addODE();
};

} /* namespace ETOL */

#endif  // INCLUDE_ETOL_EDYMOS_HPP_
