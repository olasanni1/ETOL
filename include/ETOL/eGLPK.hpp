/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Thanakorn Khamvilai
 * @author Thanakorn Khamvilai <thanakornkhamvilai@gmail.com>
 * @date 6 May 2020
 * @version 1.0.0
 * @brief The header file for the GLPK implementation for ETOL
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EGLPK_HPP
#define INCLUDE_ETOL_EGLPK_HPP

#include <ETOL/eGLPK_Types.hpp>
#include <ETOL/TrajectoryOptimizer.hpp>
#include <limits>

#define GLPK_UNBOUND std::numeric_limits<double>::infinity()

namespace ETOL
{

class eGLPK : public TrajectoryOptimizer{
public:
	/**
	 * @brief Constructor for a GLPK implementation of ETOL
	 */
	eGLPK();
	virtual ~eGLPK();

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

	// File manipulation functions
	/**
	 * @brief Get the model from an lp file
	 * @param lp_file path to lp file
	 */
	void read_lp(const char* lp_file);

	/**
	 * @brief Write the model to an lp file
	 */
	void write_lp();

	/**
	 * @brief Write the solution to a text file
	 */
	void write_sol();

	/**
	 * @brief Solve the problem from an lp file
	 * @param lp_file path to lp file
	 */
	void solve_lp(const char* lp_file);

	// static functions
	/**
	 * @brief Converts the variable type to a GLPK compliant type
	 * @param varType a ETOL variable type
	 * @return a Gurobi variable type
	 */
	static int getGLPKType(var_t varType);
	/**
	 * @brief Provides a consistent state variable name
	 * @param tIdx a time index
	 * @param sIdx a state index
	 * @return a state variable name
	 */
	static std::string getStateName (const size_t& tIdx, const size_t& sIdx);

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

	// Oters
	/**
	 * @brief Provides a consistent variable index
	 * @param name a base name for the variable
	 * @return idx a variable index
	 */
	int getVarByName(const std::string& name);

	 /**
	  * @brief Add obstacles
	  * @param t TrajectoryOptimizer class pointer
	  * @return a collection of obstacles
	  */
	static std::vector<closure_t> createObstacles(TrajectoryOptimizer* t);

	/**
	 * @brief Add tracks
	 * @param t TrajectoryOptimizer class pointer
	 * @param NSIDES number of sides for a polygon approximation of a circle
	 * @return a collection of tracks
	 */
	static std::vector<track_t> createTracks(TrajectoryOptimizer* t,int NSIDES);

	/**
	 * @brief Adds the side of an obstacle to the constraint set
	 * @param seg a segment
	 * @param name a base name for the constraint
	 * @param ind_i an index
	 * @param ind_j an index
	 * @param tIdx a time index
	 * @param pnames a set of parameter names
	 * @param sign the sign of the edge's slope
	 * @param BigM an arbitrary large scalar
	 * @return parameters for a constraint
	 */
	static std::vector<std::pair<std::vector<glpk_t>, glpk_bnd_t>>
		addObstacleSides (seg_t seg, int sign, const std::string& name,
			int ind_i, int ind_j, int tIdx, std::vector<std::string> pnames,
				int BigM);

	/**
	 * @brief Adds the sum of binary variables for an obstacle
	 * @param name a base name for the constraint
	 * @param tIdx a time index
	 * @param ind_i and index
	 * @param ind_j an index
	 * @param pnames a list of parameter names
	 * @param NSIDES number of sides for a polygon approximation of a circle
	 * @return parameters for a constraint
	 */
	static std::vector<std::pair<std::vector<glpk_t>, glpk_bnd_t>>
		addObstacleSum(const std::string& name,	int ind_i, int ind_j, int tIdx,
							std::vector<std::string> pnames, int NSIDES);

protected:
	// protected data
	glp_prob *_model;                       /**< Pointer to a GLPK model */
	glp_iocp _params;                       /**< GLPK parameters */
	int _status_relax;                      /**< status of integer-relaxed problem */
	int _status_mip;                        /**< status of integer problem */
	int _var_cnt;                           /**< counter for the number of variables */
	int _const_cnt;                         /**< counter for the number of constraints */
	int _ind_cnt;                           /**< counter for the number of non-zero element in constraint matrix */
	std::vector<int> _ia{0};                /**< The first index of non-zero element in constraint matrix */
    std::vector<int> _ja{0};                /**< The second index of non-zero element in constraint matrix */
	std::vector<double> _ar{0};             /**< The value of non-zero element in constraint matrix */
	std::map<std::string, int> _var_map;    /**< map between name and value of non-zero element in constraint matrix */

	vector_t x;                             /**< State Trajectory */
	vector_t u;                             /**< Control Trajectory */
	vector_t params;                        /**< Path constraint configs */
	std::vector<std::string> pnames;        /**< Path constraint names */

private:
	// private functions
    /**
     * @brief Add all variables to the GLPK model
     */
	void createVars();

    /**
     * @brief Add the initial state constraint to the GLPK model
     */
	void addX0();

    /**
     * @brief Add the goal state constraint to the GLPK model
     */
	void addXf();

    /**
     * @brief Add the path constraints to the GLPK model
     */
	void addConstr();

    /**
     * @brief Add the state dynamics to the GLPK model
     */
	void addDyn();

    /**
     * @brief Add the objective function to the GLPK model
     */
	void addObj();

    /**
     * @brief Get the trajectory results from the GLPK model
     */
	void getTraj();
};

} /* namespace ETOL */

#endif  // INCLUDE_ETOL_EGLPK_HPP
