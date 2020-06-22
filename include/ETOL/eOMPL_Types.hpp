/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Thanakorn Khamvilai
 * @author Thanakorn Khamvilai <thanakornkhamvilai@gmail.com>
 * @date 6 May 2020
 * @version 1.0.0
 * @brief Typedefs that are shared to all eOMPL objects
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EOMPL_TYPES_HPP
#define INCLUDE_ETOL_EOMPL_TYPES_HPP

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/config/SelfConfig.h>

#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/ltl/LTLPlanner.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/syclop/Syclop.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>

#include <any>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

namespace ETOL {
    struct ompl_t
    {
        std::string _name;
        int _tIdx, _sIdx;
        double _obj_coeff;

        ompl_t(std::string name, int tIdx, int sIdx, double obj_coeff) : \
        _name(std::move(name)), _tIdx(tIdx), _sIdx(sIdx), _obj_coeff(obj_coeff) {}
    };

    struct ompl_bnd_t
    {
        double _lb, _ub;
        ompl_bnd_t(double lb, double ub) : _lb(lb), _ub(ub) {}
    };
}

#endif  // INCLUDE_ETOL_EOMPL_TYPES_HPP
