/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 20 April 2020
 * @version 1.0.0
 * @brief The header file for the TrajectoryOptimizer class
 ******************************************************************************/

#ifndef INCLUDE_ETOL_TRAJECTORYOPTIMIZER_HPP_
#define INCLUDE_ETOL_TRAJECTORYOPTIMIZER_HPP_

#include <list>
#include <vector>
#include <cfloat>                                    /* DBL_MIN, DBL_MAX */
#include <string>
#include <ETOL/ETOL_Types.hpp>

namespace ETOL {
///
/// @brief An abstract class for defining a trajectory optimization problem
///
/// Provides a common API and utility functions for trajectory optimization
class TrajectoryOptimizer {
 public:
    //!
    //! @brief Initializes a TrajectoryOptimizer object
    TrajectoryOptimizer();

    // Virtual Members
    virtual ~TrajectoryOptimizer() {}

    /**
     * @brief Constructs a trajectory optimization problem
     */
    virtual void setup() = 0;

    /**
     * @brief Solves a trajectory optimization problem
     */
    virtual void solve() = 0;

    /**
     * @brief Generates debug information
     */
    virtual void debug() = 0;

    /**
     * @brief Releases all dynamically allocated resources
     */
    virtual void close() = 0;

    // API

    const double getScore() const;

    /**
     * @brief Sets all configurations to default values
     */
    void resetConfigs();

    /**
     * @brief Prints configurations onto the console
     */
    void printConfigs();

    /**
     * @brief Loads configurations from an XML file
     * @param filepath an absolute or relative path to an XML file
     */
    void loadConfigs(const char* filepath);

    /**
     * @brief Saves configurations to an XML file
     * @param filepath an absolute or relative path to an XML file
     */
    void saveConfigs(const char* filepath);

    /**
     * @brief Adds a non-state or non-control variable to the problem
     * @param params name and settings for the new variable
     */
    void addParams(const std::list<param_t> &params);

    /**
     * @brief Adds a non-state or non-control variable to the problem
     * @param params name and settings for the new variable
     */
    void addParams(const std::vector<std::vector<ETOL::param_t>> &params);


    /**
     * @brief Add an exclusion zone to the problem
     * @param border a list of corners for the exclusion zone
     */
    void addExclZone(border_t* border);

    /**
     * @brief Add the track of an adjacent or nearby object
     * @param track a trajectory with its minimum separation distance
     */
    void addAdjTrack(track_t* track);

    /**
     * @brief Plot the time history of a state variable
     * @param idx the index of the state variable
     */
    void plotX(const size_t idx);

    /**
     * @brief Plot the time history of a control variable
     * @param idx the index of the control variable
     */
    void plotU(const size_t idx);

    // Static Members
    /**
     * @brief Create convex partitions
     * @param border a list of corners
     * @return lower and upper segments for convex partitions
     */
    static region_t genRegion(border_t* border);

    /**
     * @brief Computes slopes for lower and upper segments for convex partitions
     * @param region lower and upper segments for convex partitions
     * @param lower updated to include corners for lower segments
     * @param upper updated to include corners for lower segments
     */
    static void calcSlopes(const region_t& region,
            std::vector<seg_t>* lower, std::vector<seg_t>* upper);

    /**
     * @brief Plots a trajectory
     * @param traj a pointer to a list of trajectories
     * @param title a title for the plot
     * @param tlab a label for the time axis
     * @param xlab a label for the data axis
     * @param tmin a lower bound for the time axis
     * @param tmax an upper bound for the time axis
     * @param xmin a lower bound for the data axis
     * @param xmax an upper bound for the data axis
     */
    static void plot(traj_t* traj,
            const std::string title = "Time History",
            const std::string tlab = "t",
            const std::string xlab = "",
            double tmin = DBL_MAX, double tmax = DBL_MIN,
            double xmin = DBL_MAX , double xmax = DBL_MIN);

    /**
     * @brief Creates a XY-plot
     * @param traj a pointer to a list of trajectories
     * @param xIdx an index to the data for the x-axis
     * @param yIdx an index to the data for the y-axis
     * @param title a title for the plot
     * @param xlab a label for the x-axis
     * @param ylab a label for the y-axis
     * @param xmin a lower bound for the x-axis
     * @param xmax an upper bound for the x-axis
     * @param ymin a lower bound for the y-axis
     * @param ymax an upper bound for the y-axis
     */
    static void plotXY(traj_t* traj,
                        size_t xIdx = 0, size_t yIdx = 1,
                        const std::string title = "XY Plot",
                        const std::string xlab = "x",
                        const std::string ylab = "y",
                        double xmin = DBL_MAX, double xmax = DBL_MIN,
                        double ymin = DBL_MAX, double ymax = DBL_MIN);

    /**
     * @brief Creates a XY-plot wth exclusion zones
     * @param traj a pointer to a list of trajectories
     * @param zones a pointer to a list of exclusion zones
     * @param xIdx an index to the data for the x-axis
     * @param yIdx an index to the data for the y-axis
     * @param title a title for the plot
     * @param xlab a label for the x-axis
     * @param ylab a label for the y-axis
     * @param xmin a lower bound for the x-axis
     * @param xmax an upper bound for the x-axis
     * @param ymin a lower bound for the y-axis
     * @param ymax an upper bound for the y-axis
     */
    static void plotXY_wExclZones(traj_t* traj,
            std::list<region_t>* zones = NULL,
            size_t xIdx = 0, size_t yIdx = 1,
            const std::string title = "XY Plot",
            const std::string xlab = "x",
            const std::string ylab = "y",
            double xmin = DBL_MAX, double xmax = DBL_MIN,
            double ymin = DBL_MAX, double ymax = DBL_MIN);

    /**
     * @brief Creates a 2D animation with static and moving exclusion zones
     * @param traj a pointer to a list of trajectories
     * @param framerate
     * @param toFile if false a animation is displayed, if true it is saved
     * @param outFile a .mp4 filename or filepath
     * @param obstacles a pointer to the static obstacles
     * @param tracks a point to the moving obstacles
     * @param xIdx an index to the data for the x-axis
     * @param yIdx an index to the data for the y-axis
     * @param title a title for the plot
     * @param xlab a label for the x-axis
     * @param ylab a label for the y-axis
     * @param xmin a lower bound for the x-axis
     * @param xmax an upper bound for the x-axis
     * @param ymin a lower bound for the y-axis
     * @param ymax an upper bound for the y-axis
     * @return the .mp4 filepath used
     */
    static std::string animate2D(traj_t* traj, const int framerate = 2,
            bool toFile = false, std::string outFile = "animation.mp4",
            std::list<region_t>* obstacles = NULL,
            std::list<track_t>* tracks = NULL,
            size_t xIdx = 0, size_t yIdx = 1,
            const std::string title = "notitle",
            const std::string xlab = "x",
            const std::string ylab = "y",
            double xmin = DBL_MAX, double xmax = DBL_MIN,
            double ymin = DBL_MAX, double ymax = DBL_MIN);

    /**
     * @brief Saves trajectories to a file
     * @param traj a pointer to a list of trajectories
     * @param fp an absolute or relative path for a file
     * @return name of file created or modified
     */
    static std::string save(traj_t* traj, std::string fp = "traj.csv");

    // Template functions
    /**
	 * @brief A linear interplation
	 *
	 * @tparam T a numeric datatype
	 * @param tval independent variable's value
	 * @param tvec independent variable's interpolation points
	 * @param ref dependent variable's interpolation points
	 * @return a linear interpolation value
	 */
	template<class T>
	static T linear_interpolation(const T& tval, const state_t &tvec,
			const state_t &ref) {
		size_t j = 0;  // if t < tvec[0]
		if (tval > tvec.back()) {
			j = tvec.size() - 2.;
		} else if (tval >= tvec.front()) {
			auto curr = tvec.cbegin();
			auto next = std::next(curr, 1);
			while (next != tvec.cend()) {
				if (tval >= *curr && tval <= *next)
					j = std::distance(tvec.cbegin(), curr);
				curr = next;
				std::advance(next, 1);
			}
		}
		return (tval - tvec.at(j)) * (ref.at(j+1) - ref.at(j)) /
				(tvec.at(j+1) - tvec.at(j)) + ref.at(j);
	}

	/**
	 * @brief Extracts a m-dimensional trajectory
	 *
	 * @tparam T an integer datatype
	 * @param traj an n-dimensional trajectory
	 * @param x_idxs
	 * @return a m-dimensionional trajectory
	 */
	template<typename T>
	static traj_t extractTraj(const traj_t &traj,
			const std::vector<T> &idxs) {
		ETOL::traj_t out(traj.size());
		std::transform(traj.cbegin(), traj.cend(), out.begin(),
				[&idxs](const ETOL::traj_elem_t &elem) -> ETOL::traj_elem_t {
			ETOL::state_t state;
			std::transform(idxs.cbegin(), idxs.cend(),
					std::back_inserter(state),
					[&elem](const size_t &i) -> double {
				return i == 0 ? elem.first : elem.second.at(i - 1);
			});
			return ETOL::traj_elem_t(elem.first, state);
		});
		return out;
	}

	/**
	 * @brief Scale the state values at each time step
	 *
	 * @tparam T a numeric datatype
	 * @param traj a pointer to an n-dimensional trajectory
	 * @param scalers a vector of values that will scale the trajectory
	 */
	template<typename T>
	static void scaleTraj(traj_t* traj, const std::vector<T> &scalers) {
		auto init = scalers.cbegin();
		auto stop = scalers.cend();
		std::for_each(traj->begin(), traj->end(),
				[&init, &stop](ETOL::traj_elem_t &elem) {
			auto curr = init;
			std::for_each(elem.second.begin(), elem.second.end(),
					[&curr, &stop](state_t::value_type &state) {
				state *= (curr == stop) ? 1. : *(curr++);
			});
		});
	}

	/**
	 * @brief Offset the state values at each time step
	 *
	 * @tparam T a numeric datatype
	 * @param traj a pointer to a state vector trajectory
	 * @param offsets a vector of values that will be added to the trajectory
	 */
	template<typename T>
	static void offsetTraj(traj_t* traj, const std::vector<T> &offsets) {
		auto init = offsets.cbegin();
		auto stop = offsets.cend();
		std::for_each(traj->begin(), traj->end(),
				[&init, &stop](ETOL::traj_elem_t &elem) {
			auto curr = init;
			std::for_each(elem.second.begin(), elem.second.end(),
					[&curr, &stop](state_t::value_type &state) {
				state += (curr == stop) ? 0. : *(curr++);
			});
		});
	}

    // Getters and Setters
    /**
     * @brief Get the initial state
     * @return the inital state vector
     */
    state_t& getX0();

    /**
     * @brief Set an initial state
     * @param x0 the initial state vector
     */
    virtual void setX0(const state_t &x0);

    /**
     * @brief Get the goal state
     * @return the final (or goal) state vector
     */
    state_t& getXf();

    /**
     * @brief Set the goal state
     * @param xf a final (or goal) state vector
     */
    virtual void setXf(const state_t &xf);

    /**
     * @brief Get the number of control variables
     * @return the dimension of the control vector
     */
    const size_t getNControls() const;

    /**
     * @brief Get the number of state variables
     * @return the dimension of the state vector
     */
    const size_t getNStates() const;

    /**
     * @brief Get the states' lower bound
     * @return the lower bounds for the states
     */
    state_t& getXlower();

    /**
     * @brief Set the states' lower bound
     * @param xlower lower bounds for the states
     */
    virtual void setXlower(const state_t &xlower);

    /**
     * @brief Get the states' upper bound
     * @return the upper bounds for the states
     */
    state_t& getXupper();

    /**
     * @brief Set the states' upper bound
     * @param xupper upper bounds for the states
     */
    virtual void setXupper(const state_t &xupper);

    /**
     * @brief Get the states' variable type
     * @return the state variables' type
     */
    state_var_t& getXvartype();

    /**
     * @brief Set the states' variable type
     * @param xvartype state variables' type
     */
    virtual void setXvartype(const state_var_t &xvartype);

    /**
     * @brief Get the time step size
     * @return the time step size
     */
    const double getDt() const;

    /**
     * @brief Set the time step size
     * @param dt a time step size
     */
    virtual void setDt(double dt);

    /**
     * @brief Get the number of time steps
     * @return the number of time steps
     */
    const size_t getNSteps() const;

    /**
     * @brief Set the number of steps
     * @param nSteps number of time steps
     */
    virtual void setNSteps(const size_t nSteps);

    /**
     * @brief Get the states' error tolerance
     * @return the error tolerance for the states
     */
    state_t& getXtol();

    /**
     * @brief Set the states' error tolerance
     * @param xtol error tolerance for the states
     */
    virtual void setXtol(const state_t &xtol);

    /**
     * @brief Get the controls' lower bound
     * @return the lower bound on the controls
     */
    state_t& getUlower();

    /**
     * @brief Set the control's lower bound
     * @param ulower lower bound on the controls
     */
    virtual void setUlower(const state_t &ulower);

    /**
     * @brief Get the controls' upper bound
     * @return the upper bound on the controls
     */
    state_t& getUupper();

    /**
     * @brief Set the controls' upper bound
     * @param uupper upper bound on the controls
     */
    virtual void setUupper(const state_t &uupper);

    /**
     * @brief Get the control variables' type
     * @return the control variables' type
     */
    state_var_t& getUvartype();

    /**
     * @brief Set the control variables' type
     * @param uvartype control variables' type
     */
    virtual void setUvartype(const state_var_t &uvartype);

    /**
     * @brief Get the controls' reverse time horizon
     * @return the reverse time horizon for the control variables
     *
     * The reverse time horizon is the number of time steps needed that is
     * relative to the current time index. For example, if the current time
     * index is 10, and \a nu4dyn is 2, then time index 8, 9, and 10 are used
     * when computing the state at time index 10.
     */
    const size_t getUrhorizon() const;

    /**
     * @brief Sets the reverse time horizon for control variables
     * @param nu4dyn an integer number of prior time indices
     *
     * The reverse time horizon is the number of time steps needed that is
     * relative to the current time index. For example, if the current time
     * index is 10, and \a nu4dyn is 2, then time index 8, 9, and 10 are used
     * when computing the state at time index 10.
     */
    virtual void setUrhorizon(const size_t nu4dyn);

    /**
     * @brief Get the states' reverse time horizon
     * @return the reverse time horizon for the state variables
     *
     * The reverse time horizon is the number of time steps needed that is
     * relative to the current time index. For example, if the current time
     * index is 10, and \a nx4dyn is 2, then state at time index 8, 9, and 10
     * are used when computing the state at time index 10.
     */
    const size_t getXrhorizon() const;

    /**
     * @brief Sets the reverse time horizon for state variables
     * @param nx4dyn an integer number of prior time indices
     *
     * The reverse time horizon is the number of time steps needed that is
     * relative to the current time index. For example, if the current time
     * index is 10, and \a nx4dyn is 2, then state at time index 8, 9, and 10
     * are used when computing the state at time index 10.
     */
    virtual void setXrhorizon(const size_t nx4dyn);

    /**
     * @brief Get the maximium possible reverse time horizon
     * @return the maximum possible reverse time horizon
     */
    size_t getRhorizon() const;

    /**
     * @brief Get the number of control variables
     * @param nControls number of control variables
     */
    virtual void setNControls(const size_t nControls);

    /**
     * @brief Get the number of state variables
     * @param nStates number of state variables
     */
    virtual void setNStates(const size_t nStates);

    /**
     * @brief Set the equality constraints
     * @param constraints a vector equality constraint functions
     */
    virtual void setEqConstraints(std::vector<f_t*> constraints);

    /**
     * @brief Set the inequality constraints
     * @param constraints a vector of inequality constraint functions
     */
    virtual void setLessEqConstraints(std::vector<f_t*> constraints);

    /**
     * @brief Set sense-agnostic constraint functions
     * @param constraints a vector of constraint functions
     */
    virtual void setConstraints(std::vector<f_t*> constraints);

    /**
     * @brief Set the states' time derivative function
     * @param gradient a vector of time derivative functions for the states
     */
    virtual void setGradient(std::vector<f_t*> gradient);

    /**
     * @brief Set the objective function
     * @param objective an objective function
     */
    virtual void setObjective(f_t* objective);

    /**
     * @brief Get the pointer to the computed control trajectory
     * @return the pointer to the computed control trajectory
     */
    traj_t* getUtraj();

    /**
     * @brief Get the pointer to the computed state trajectory
     * @return the pointer to the computed state trajectory
     */
    traj_t* getXtraj();

    /**
     * @brief Set the objective function
     * @return the pointer to the objective function
     */
    const f_t* getObjective() const;

    /**
     * @brief Get the time derivative functions for the states
     * @return the pointer to the time derivative functions for the states
     */
    std::vector<f_t*>* getGradient();

    /**
     * @brief Get the equality constraint functions
     * @return the pointer to the inequality constraint functions
     */
    std::vector<f_t*>* getEqConstraints();

    /**
     * @brief Get the inequality constraint functions
     * @return the pointer to the inequality constraint functions
     */
    std::vector<f_t*>* getLessEqConstraints();

    /**
     * @brief Get the sense-agnostic constraint functions
     * @return the pointer to the constraint functions
     */
    std::vector<f_t*>* getConstraints();

    /**
     * @brief Get the pointer to the unprocessed exclusion zones
     * @return the pointer to unprocessed exclusion zones
     */
    std::vector<border_t>* getObstacles_Raw();

    /**
     * @brief Get the pointer to the polygon partitioned exclusion zones
     * @return the pointer to polygon partitioned exclusion zones
     */
    std::list<region_t>* getObstacles();

    /**
     * @brief Get the pointer to the moving exclusion zones
     * @return the pointer to moving exclusion zones
     */
    std::list<track_t>* getTracks();

    /**
     * @brief Get the optimization direction
     * @return the optimization direction.
     *
     * True indicates maximization. False indicates minimization
     */
    bool isMaximized() const;

    /**
     * @brief Set the optimization direction
     * @param maximize a optimization direction
     *
     * True indicates maximization. False indicates minimization
     */
    virtual void setMaximize(const bool maximize);

    /**
     * @brief Get the number of exclusion zones
     * @return the number of exclusion zones
     */
    size_t getNExclZones();

    /**
     * @brief Get the number of moving exclusion zones
     * @return the number of moving exclusion zones
     */
    size_t getNTracks();

 protected:
    /**
     * @brief An error handler that is called in try-catch blocks.
     */
    void errorHandler();

    /**
     * @brief Stores the eSovler's objective function result.
     * @param score an eSolver's objective function result
     */
    void setScore(const double score);

    // Protected Data Members
    bool _maximize;                         /**< maximize objective if TRUE */
    double _score;                          /**< optimization result */
    double _dt;                             /**< time step */
    size_t _nSteps;                         /**< number of time steps */
    size_t _nStates;                        /**< number of state variables */
    size_t _nControls;                      /**< number of control variables */
    state_t _x0;                            /**< initial state */
    state_t _xlower;                        /**< state lower bound */
    state_t _xupper;                        /**< state upper bound */
    state_var_t _xvartype;                  /**< state variables' type */
    state_t _xtol;                          /**< states' error tolerance */
    state_t _xf;                            /**< goal state */
    state_t _ulower;                        /**< control lower bound */
    state_t _uupper;                        /**< control upper bound */
    state_var_t _uvartype;                  /**< control variables' type */
    size_t _xrhorizon;                      /**< state's reverse time horizon */
    size_t _urhorizon;                    /**< control's reverse time horizon */
    size_t _rhorizon;                       /**< Maximum reverse time horizon */
    paramset_t _parameters;                 /**< parameter key/value map */
    std::vector<border_t> _obstacles_raw;   /**< raw exclusion zones*/
    std::list<region_t> _obstacles;         /**< partitioned exclusion zones */
    std::list<track_t> _tracks;             /**< moving exclusion zones */
    std::vector<f_t*> _constraints;         /**< sense-agnostic constraints */
    std::vector<f_t*> _eq;                  /**< equality constraints */
    std::vector<f_t*> _lesseq;              /**< inequality constraints */
    std::vector<f_t*> _gradient;            /**< state time derivatives */
    f_t* _objective;                        /**< objective function */
    traj_t _xtraj;                          /**< state trajectory */
    traj_t _utraj;                          /**< control trajectory */
    std::bad_any_cast* _eAny;               /**< bad cast exception */
};

} /* namespace ETOL */

#endif  // INCLUDE_ETOL_TRAJECTORYOPTIMIZER_HPP_
