/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 20 April 2020
 * @version 1.0.0
 * @brief The header file for the Gurobi implementation for ETOL
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EGUROBI_HPP_
#define INCLUDE_ETOL_EGUROBI_HPP_

#include <vector>
#include <string>
#include <atomic>
#include <ETOL/eGurobi_Types.hpp>
#include <ETOL/TrajectoryOptimizer.hpp>


namespace ETOL {

class eGurobi : public TrajectoryOptimizer{
 public:
    /**
     * @brief Constructor for a Gurobi interface
     */
    eGurobi();

    virtual ~eGurobi();

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

    /**
     * @brief Returns the cached number of constraints
     * @return an integer number of constraints
     */
    const size_t getNConstr() const;

    /**
     * @brief Converts the variable type to a Gurobi compliant type
     * @param varType a ETOL variable type
     * @return a Gurobi variable type
     */
    static char getGRBType(var_t varType);

    /**
     * @brief Provides a consistent state variable name
     * @param tIdx a time index
     * @param sIdx a state index
     * @return a state vairable name
     */
    static std::string getStateName(const size_t& tIdx, const size_t& sIdx);

    /**
     * @brief Provides a consistent control variable name
     * @param tIdx a time index
     * @param sIdx a control index
     * @return a control variable name
     */
    static std::string getControlName(const size_t& tIdx, const  size_t& sIdx);

    /*!
     * @brief Provides a consistent name for a custom variable
     * @param name a base name for the variable
     * @param tIdx a time index
     * @return a custom variable name
     */
    static std::string getParamName(const std::string& name,
            const size_t& tIdx);



 protected:
    // Protected data
    GRBEnv* _env;                       /**< Pointer to a Gurobi environment */
    GRBModel* _model;                   /**< Pointer to a Gurobi model */
    GRBException* _eGRB;                /**< Pointer to a Gurobi exception */
    vector_t _params;                   /**< Path constraint configs */
    std::vector<std::string> _pnames;   /**< Path constraint names */
    std::atomic<size_t> _nConstr;       /**< Number of path constraints */

    // Protected functions
    /**
     * @brief Converts the objective function to a Gurobi expression
     * @param tIdx a time index
     * @return a Gurobi linear expression
     *
     * This function is called for each time index and added to objective
     * function in the Gurobi model
     */
    GRBLinExpr _objfunc(const size_t& tIdx);

    /**
     * @brief Converts the gradient function to a Gurobi expression
     * @param tIdx a time index
     * @param sIdx a state index
     * @return a Gurobi linear expression
     *
     * This function is called for each time index and added to as a nex
     * constraint to the Gurobi model
     */
    GRBLinExpr _dynConstr(const size_t& tIdx, const size_t& sIdx);

    /**
     * @brief Converts the constraint functions to a Gurobi expression
     * @param tIdx a time index
     * @param kIdx a path constraint index
     * @return parameters for a Gurobi constraint
     *
     * This function is called for each time index and path constraint. The
     * result is added as a new constraint to the Gurobi model.
     */
    fout_grb_t _pathConstr(const size_t& tIdx, const size_t& kIdx);

 private:
    // Private data
    GRBEnv env_;                    /**< Gurobi environment */
    GRBModel model_;                /**< Gurobi model */

    // Private functions
    /**
     * @brief An error handler for Gurobi exceptions
     */
    void errorHandler();

    /**
     * @brief Add all variables to the Gurobi model
     */
    void createVars();

    /**
     * @brief Add the initial state constraint to the Gurobi model
     */
    void addX0();

    /**
     * @brief Add the goal state constraint to the Gurobi model
     */
    void addXf();

    /**
     * @brief Add the path constraints to the Gurobi model
     */
    void addConstr();

    /**
     * @brief Add the state dynamics to the Gurobi model
     */
    void addDyn();

    /**
     * @brief Add the objective function to the Gurobi model
     */
    void addObj();

    /**
     * @brief Get the trajectory results from the Gurobi model
     */
    void getTraj();
};

} /* namespace ETOL */

#endif  // INCLUDE_ETOL_EGUROBI_HPP_
