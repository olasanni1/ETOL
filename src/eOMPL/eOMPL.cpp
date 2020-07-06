/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Thanakorn Khamvilai
 * @author Thanakorn Khamvilai <thanakornkhamvilai@gmail.com>
 * @date 6 May 2020
 * @version 1.0.0
 * @brief A GLPK implementation for ETOL
 * @section DESCRIPTION
 * A <a href="https://www.gnu.org/software/glpk/">GLPK 4.32</a> implementation
 * for the pure virtual methods in the TrajectoryOptimization class. It casts
 * the trajectory optimization problem as a sampling-based problem
 ******************************************************************************/

#include <re2/re2.h>
#include <iostream>
#include <cassert>
#include <ETOL/eOMPL.hpp>

namespace ETOL {
    bool ValidityChecker::onSegment(Point p, Point q, Point r) const {
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
            return true;
        return false;
    }

    int ValidityChecker::orientation(Point p, Point q, Point r) const {
        int val = (q.y - p.y) * (r.x - q.x) -
                  (q.x - p.x) * (r.y - q.y);

        if (val == 0) return 0;     // colinear
        return (val > 0) ? 1: 2;    // clock or counterclock wise
    }

    bool ValidityChecker::doIntersect(Point p1, Point q1, Point p2,
            Point q2) const {
        // Find the four orientations needed for general and
        // special cases
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;

        // p1, q1 and p2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;

        // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false;  // Doesn't fall in any of the above cases
    }

    bool ValidityChecker::isInside(Point polygon[], int n, Point p) const {
        // There must be at least 3 vertices in polygon[]
        if (n < 3)  return false;

        // Create a point for line segment from p to infinite
        Point extreme = {INF, p.y};

        // Count intersections of the above line with sides of polygon
        int count = 0, i = 0;
        do {
            int next = (i+1)%n;

            // Check if the line segment from 'p' to 'extreme' intersects
            // with the line segment from 'polygon[i]' to 'polygon[next]'
            if (doIntersect(polygon[i], polygon[next], p, extreme)) {
                // If the point 'p' is colinear with line segment 'i-next',
                // then check if it lies on segment. If it lies, return true,
                // otherwise false
                if (orientation(polygon[i], p, polygon[next]) == 0)
                    return onSegment(polygon[i], p, polygon[next]);
                count++;
            }
            i = next;
        } while (i != 0);

        // Return true if count is odd, false otherwise
        return count&1;  // Same as (count%2 == 1)
    }

    bool ValidityChecker::isValid(const ompl::base::State *state) const {
        bool res = true;
        const auto *s = state->
                as<ompl::base::RealVectorStateSpace::StateType>();
        Point p = {s->values[0], s->values[1]};
        for (const auto& obs : *this->obstacles_) {
            int n = obs.size();
            std::vector<Point> polygons;
            int i = 0;
            for (const auto& corner : obs) {
                polygons.push_back({corner[0], corner[1]});
                i++;
            }
            res = res & !isInside(&polygons[0], n, p);
        }
        return res;
    }

    // Constructors
    eOMPL::eOMPL()
        : configured_(false),
          planTime_(0.),
          solveTime_(0.),
          last_status_(ompl::base::PlannerStatus::UNKNOWN) {}

    // API
    void eOMPL::setup() {
        if (!this->configured_ || !this->si_->isSetup() ||
                !this->planner_->isSetup()) {
            if (!this->si_->isSetup())
                this->si_->setup();
            if (!this->planner_) {
                if (this->pa_)
                    this->planner_ = this->pa_(this->si_);
                if (!this->planner_) {
                    std::string planner_usage =
                            "Using " + this->planner_name_ + ".";
                    OMPL_INFORM(planner_usage.c_str());
                    if (this->planner_name_ == "RRT")
                        this->planner_ = ompl::base::PlannerPtr(
                                new ompl::control::RRT(this->si_));
                    else if (this->planner_name_ == "SST")
                        this->planner_ = ompl::base::PlannerPtr(
                                new ompl::control::SST(this->si_));
                    else if (this->planner_name_ == "EST")
                        this->planner_ = ompl::base::PlannerPtr(
                                new ompl::control::EST(this->si_));
                    else if (this->planner_name_ == "KPIECE")
                        this->planner_ = ompl::base::PlannerPtr(
                                new ompl::control::KPIECE1(this->si_));
                    else if (this->planner_name_ == "PDST")
                        this->planner_ = ompl::base::PlannerPtr(
                                new ompl::control::PDST(this->si_));
                    else
                        this->planner_ = ompl::base::PlannerPtr(
                                new ompl::control::SST(this->si_));
                }
            }
            this->planner_->setProblemDefinition(this->pdef_);
            if (!this->planner_->isSetup())
                this->planner_->setup();

            this->configured_ = true;
        }
    }

    void eOMPL::solve() {
        this->last_status_ = ompl::base::PlannerStatus::UNKNOWN;
        ompl::time::point start = ompl::time::now();
        this->last_status_ = this->planner_->solve(this->solveTime_);
        this->planTime_ = ompl::time::seconds(ompl::time::now() - start);
        if (this->last_status_) {
            OMPL_INFORM("Solution found in %f seconds", this->planTime_);
            this->setScore(pdef_->getSolutionPath()->length());
            this->getTraj();
        } else {
            OMPL_INFORM("No solution found after %f seconds", this->planTime_);
        }
    }

    void eOMPL::debug() {}

    void eOMPL::close() {
        if (planner_)
            planner_->clear();
        if (pdef_)
            pdef_->clearSolutionPaths();
    }

// private members
    void eOMPL::createVars() {
        // create state-space and its bound
        this->space_ = std::shared_ptr<ompl::base::RealVectorStateSpace>(
                std::make_shared<ompl::base::RealVectorStateSpace>(
                        this->getNStates()));
        ompl::base::RealVectorBounds bounds(this->getNStates());
        state_t::iterator l_it, u_it;
        l_it = this->getXlower().begin();
        u_it = this->getXupper().begin();
        for (int i = 0; i < this->getNStates(); i++) {
            bounds.setLow(i, *(l_it++));
            bounds.setHigh(i, *(u_it++));
        }
        this->space_->setBounds(bounds);

        // create a control space and its bound
        this->cspace_ = std::shared_ptr<ompl::control::RealVectorControlSpace>(
                std::make_shared<ompl::control::RealVectorControlSpace>(
                        this->space_, this->getNControls()));
        ompl::base::RealVectorBounds cbounds(this->getNControls());
        l_it = this->getUlower().begin();
        u_it = this->getUupper().begin();
        for (int i = 0; i < this->getNControls(); i++) {
            cbounds.setLow(i, *(l_it++));
            cbounds.setHigh(i, *(u_it++));
        }
        this->cspace_->setBounds(cbounds);

        // Construct a space information instance for this state space
        this->si_ = std::make_shared<ompl::control::SpaceInformation>(
                this->cspace_->getStateSpace(), this->cspace_);
        this->pdef_ = std::make_shared<ompl::base::ProblemDefinition>(
                this->si_);
        this->si_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
                new ETOL::ValidityChecker(
                        this->si_, this->getObstacles_Raw())));
    }

    void eOMPL::addX0Xf() {
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(
                this->space_);
        for (int i = 0; i < this->getNStates(); i++) {
            start[i] = this->getX0().at(i);
        }

        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(
                this->space_);
        for (int i = 0; i < this->getNStates(); i++) {
            goal[i] = this->getXf().at(i);
        }

        this->pdef_->setStartAndGoalStates(start, goal, this->threshold_);
    }

    void eOMPL::setGradient(const ompl::control::ODESolver::ODE &ode,
           const ompl::control::ODESolver::PostPropagationEvent &postEvent) {
        this->solveTime_ = this->getNSteps() * this->getDt();
        this->createVars();
        this->addX0Xf();

        ompl::control::ODESolverPtr odeSolver(
                new ompl::control::ODEBasicSolver<> (this->si_, ode));
        this->si_->setStatePropagator(
                ompl::control::ODESolver::getStatePropagator(
                        odeSolver, postEvent));
    }

    void eOMPL::setPlanner(std::string planner_name) {
        this->planner_name_ = planner_name;
    }

    void eOMPL::getTraj() {
        if (this->last_status_) {
            std::cout << this->planner_->getName()
                      << " found a solution of length "
                      << this->pdef_->getSolutionPath()->length() << std::endl;

            std::shared_ptr<ompl::control::PathControl> sol =
                    std::static_pointer_cast<ompl::control::PathControl>(
                            this->pdef_->getSolutionPath());
            sol->print(std::cout);

            std::ofstream myfile;
            myfile.open("ompl_path.txt");
            sol->printAsMatrix(myfile);
            myfile.close();

            std::ifstream myReadFile;
            myReadFile.open("ompl_path.txt");
            std::vector<std::vector<double> > data;
            std::vector<double> dat;
            double d;

            traj_t* xtraj = this->getXtraj();
            xtraj->clear();
            traj_t* utraj = this->getUtraj();
            utraj->clear();

            for (int i = 0; i < sol->getStateCount(); i++) {
                state_t x_sol;
                state_t u_sol;
                double t;
                size_t nvars = this->getNStates() + this->getNControls() + 1;
                for (int j = 0; j < nvars; j++) {
                    myReadFile >> d;
                    if (j < this->getNStates())
                        x_sol.push_back(d);
                    else if (j < this->getNControls())
                        u_sol.push_back(d);
                    else
                        t = d;
                }
                xtraj->push_back(traj_elem_t(t, x_sol));
                utraj->push_back(traj_elem_t(t, u_sol));
            }
        }
    }
} /* namespace ETOL */
