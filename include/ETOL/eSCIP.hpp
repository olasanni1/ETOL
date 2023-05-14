/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 16 October 2020
 * @version 1.0.0
 * @brief The header file for ETOL's SCIP implementation
 ******************************************************************************/

#ifndef INCLUDE_ETOL_ESCIP_HPP
#define INCLUDE_ETOL_ESCIP_HPP

#include <scip/scip.h>

#include <limits>
#include <vector>
#include <string>
#include <atomic>
#include <memory>
#include <ETOL/eSCIP_Types.hpp>
#include <ETOL/TrajectoryOptimizer.hpp>

namespace ETOL {
class eSCIP : public TrajectoryOptimizer {
 public:
    /**
     * @brief Constructor for a SCIP implementation of ETOL
     */
    eSCIP();

    /**
     * @brief Destructor for a SCIP implementation of ETOL
     */
    virtual ~eSCIP();

    // API
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

    // static functions
    /**
     * @brief Converts the variable type to a SCIP compliant type
     * @param varType a ETOL variable type
     * @return a SCIP variable type
     */
    static SCIP_VARTYPE getSCIPType(var_t varType);
    /**
     * @brief Provides a consistent state variable name
     * @param tIdx a time index
     * @param sIdx a state index
     * @return a state variable name
     */
    static std::string getStateName(const size_t& tIdx, const size_t& sIdx);

    /**
     * @brief Provides a consistent control variable name
     * @param tIdx a time index
     * @param sIdx a control index
     * @return a control variable name
     */
    static std::string getControlName(const size_t& tIdx, const size_t& sIdx);

    /**
     * @brief Provides a consistent name for a custom variable
     * @param name a base name for the variable
     * @param tIdx a time index
     * @return a custom variable name
     */
    static std::string getParamName(std::string name, const size_t& tIdx);

    /**
     *
     * @brief Provides a consistent name for a custom variable
     * @param name a base name for the variable
     * @param tIdx a time index
     * @param sIdx a state index
     * @return a custom variable name
     */
    static std::string getParamName(std::string name, const size_t& tIdx,
            const size_t& sIdx);

    /**
     *
     * @brief Provides a consistent name for a custom variable
     * @param name a base name for the variable
     * @param Idx1 a time index
     * @param Idx2 a state index
     * @param Idx3 an index
     * @return
     */
    static std::string getParamName(std::string name, const size_t& Idx1,
            const size_t& Idx2, const size_t& Idx3);

    SCIP* getModel() const;

    void setDt(double dt);
    void setNSteps(const size_t nSteps);
    void setXvartype(const state_var_t &xvartype);
    void setX0(const state_t& x0);
    void setXf(const state_t& x0);
    void setXtol(const state_t &xtol);
    void setXupper(const state_t &xupper);
    void setXlower(const state_t &xlower);
    void setXrhorizon(const size_t nx4dyn);
    void setUvartype(const state_var_t &uvartype);
    void setUupper(const state_t &uupper);
    void setUlower(const state_t &ulower);
    void setUrhorizon(const size_t nu4dyn);
    void setObjective(f_t* objective);
    void setGradient(std::vector<f_t*> gradient);
    void setConstraints(std::vector<f_t*> constraints);

 protected:
    SCIP* scip_;                            /**< SCIP model */
    SCIP_VAR** x_;                          /**< State variables */
    SCIP_VAR** u_;                          /**< Control variables */
    SCIP_VAR** p_;                          /**< Path binary variables */
    vector_t params_;                       /**< Path constraint configs */
    std::vector<std::string> pnames_;       /**< Path constraint names */
    std::atomic<size_t> nConstr_;           /**< Number of path constraints */
    bool x0_changed_;                       /**< X0 change flag */
    bool xf_changed_;                       /**< Xf change flag */
    bool reset_;                            /**< Reset flag */
    bool configured_;                       /**< SCIP interface ended */
    /** Initial state constraint equation names */
    std::vector<std::string> _x0_constraint_names;
    /** Goal state constraint equation names */
    std::vector<std::string> _xf_upper_constraint_names;
    std::vector<std::string> _xf_lower_constraint_names;

    /**
     * @brief Converts the objective function to a SCIP expression
     * @param tIdx a time index
     * @return a SCIP linear expression
     *
     * This function is called for each time index and added to objective
     * function in the SCIP model
     */
    scip_expr_t objfunc(const size_t& tIdx);

    /**
     * @brief Converts the gradient function to a SCIP expression
     * @param tIdx a time index
     * @param sIdx a state index
     * @return a SCIP linear expression
     *
     * This function is called for each time index and added to as a nex
     * constraint to the SCIP model
     */
    scip_expr_t dynConstr(const size_t& tIdx, const size_t& sIdx);

    /**
     * @brief Converts the constraint functions to a SCIP expression
     * @param tIdx a time index
     * @param kIdx a path constraint index
     * @return parameters for a SCIP constraint
     *
     * This function is called for each time index and path constraint. The
     * result is added as a new constraint to the SCIP model.
     */
    fout_scip_t pathConstr(const size_t& tIdx, const size_t& kIdx);
    vector_t getStates(const size_t &tIdx);
    vector_t getControls(const size_t &tIdx);
    SCIP_VAR* getVarByName(const std::string& name);
    std::shared_ptr<std::function<SCIP_Var*(const std::string&)>> getVar_;

 private:
    /**
     * @brief Add all variables to the SCIP model
     */
    void createVars();

    /**
     * @brief Set the initial state constraint to the SCIP model
     */
    void configX0();

    /**
     * @brief Set the goal state constraint to the SCIP model
     */
    void configXf();

    /**
     * @brief Add the path constraints to the SCIP model
     */
    void addConstr();

    /**
     * @brief Add the state dynamics to the SCIP model
     */
    void addDyn();

    /**
     * @brief Add the objective function to the SCIP model
     */
    void addObj();

    /**
     * @brief Get the trajectory results from the SCIP model
     */
    void getTraj();

    void releaseVars(SCIP_VAR** var, const size_t &len);
};

}  // namespace ETOL

#endif  // INCLUDE_ETOL_ESCIP_HPP
