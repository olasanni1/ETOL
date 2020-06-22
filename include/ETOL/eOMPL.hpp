/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Thanakorn Khamvilai
 * @author Thanakorn Khamvilai <thanakornkhamvilai@gmail.com>
 * @date 6 May 2020
 * @version 1.0.0
 * @brief The header file for the OMPL implementation for ETOL
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EOMPL_HPP
#define INCLUDE_ETOL_EOMPL_HPP

#include <ETOL/eOMPL_Types.hpp>
#include <ETOL/TrajectoryOptimizer.hpp>

#define OMPL_DYN_ARGS const ompl::control::ODESolver::StateType& x, const ompl::control::Control* control, ompl::control::ODESolver::StateType& xdot
#define OMPL_DYNPOST_ARGS const ompl::base::State* /*state*/, const ompl::control::Control* /*control*/, const double /*duration*/, ompl::base::State *result
#define INF 10000

namespace ETOL
{
    class ValidityChecker : public ompl::base::StateValidityChecker {
    public:
        /**
         * @brief Constructor for a OMPL Collision checker of ETOL
         * @param si OMPL space information pointer
         * @param obstacles
         */
        ValidityChecker(const ompl::base::SpaceInformationPtr &si, std::vector<border_t>* obstacles) :
                ompl::base::StateValidityChecker(si), obstacles_(obstacles)
        {}

        /**
         * @brief (x,y) co-ordinate for polyhedron obstacles
         */
        struct Point {
            double x;
            double y;
        };

        /**
         * @brief Check if a point lies on a line segment
         * @param p first colinear point
         * @param q second colinear point
         * @param r third colinear point
         * @return true if q lies on line segment 'pr'
         */
        bool onSegment(Point p, Point q, Point r) const;

        /**
         * @brief Find orientation of ordered triplet (p, q, r).
         * @param p first colinear point
         * @param q second colinear point
         * @param r third colinear point
         * @return 0 --> p, q and r are colinear, 1 --> Clockwise, 2 --> Counterclockwise
         */
        int orientation(Point p, Point q, Point r) const;

        /**
         * @brief Check if two lines intersect
         * @param p1 first point of the first line
         * @param q1 second point of the first line
         * @param p2 first point of the second line
         * @param q2 second point of the second line
         * @return true if there is an intersection
         */
        bool doIntersect(Point p1, Point q1, Point p2, Point q2) const;

        /**
         * @brief Check if the point lies inside the polygon
         * @param polygon set of points that forms a polygon
         * @param n the number of vertices of the polygon
         * @param p the point to be check
         * @return true if the point lies inside the polygon
         */
        bool isInside(Point polygon[], int n, Point p) const;

        /**
         * @brief Check if the state collides with obstacles
         * @param state generated point from sampling-based path planning algorithm
         * @return true if the state does NOT collide with obstacles
         */
        bool isValid(const ompl::base::State *state) const;
        std::vector<border_t>* obstacles_;
    };

    class eOMPL : public TrajectoryOptimizer {
    public:
        /**
         * @brief Constructor for a OMPL implementation of ETOL
         */
        explicit eOMPL();

        virtual ~eOMPL() = default;

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

        // Others
        /**
         * @brief Set the system dynamics x_dot = f(x,u)
         * @param ode Solver to solve the differential equation
         * @param postEvent Function to execute after the integration
         */
        void setGradient(const ompl::control::ODESolver::ODE &ode,
                         const ompl::control::ODESolver::PostPropagationEvent &postEvent = nullptr);

        /**
         * @brief Set the sampling-based path planning algorithm to be used
         * @param planner_name Name of sampling-based path planning algorithm to be used
         */
        void setPlanner(std::string planner_name);

        ompl::control::SpaceInformationPtr si_;
        /**< pointer to OMPL space information */
        ompl::base::ProblemDefinitionPtr pdef_;
        /**< pointer to OMPL problem definition */

    protected:
        // protected data
        vector_t x;                         /**< State Trajectory */
        vector_t u;                         /**< Control Trajectory */
        vector_t params;                    /**< Path constraint configs */
        std::vector<std::string> pnames;    /**< Path constraint names */

        std::shared_ptr<ompl::base::RealVectorStateSpace> space_;
        /**< pointer to OMPL state space */
        std::shared_ptr<ompl::control::RealVectorControlSpace> cspace_;
        /**< pointer to OMPL control space */
        ompl::base::PlannerPtr planner_;
        /**< pointer to OMPL planner */
        ompl::base::PlannerAllocator pa_;
        /**< OMPL planner allocator */
        ompl::base::PlannerStatus last_status_;   /**< OMPL status */
        bool configured_;                         /**< OMPL configure flag */
        double planTime_;                         /**< OMPL planning time */
        double solveTime_;                        /**< OMPL solving time */
        const double threshold_ = std::numeric_limits<double>::epsilon();
        /**< OMPL goal state tolerant */
        std::string planner_name_ = "SST";        /**< OMPL planner name */

    private:
        // private functions
        /**
         * @brief Add all variables to the OMPL model
         */
        void createVars();

        /**
         * @brief Add the initial and goal state constraint to the OMPL model
         */
        void addX0Xf();

        /**
         * @brief Get the trajectory results from the OMPL model
         */
        void getTraj();
    };
} /* namespace ETOL */

#endif  // INCLUDE_ETOL_EOMPL_HPP
