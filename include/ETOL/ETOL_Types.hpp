/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 20 April 2020
 * @version 1.0.0
 * @brief Typedefs that are shared to all ETOL objects
 ******************************************************************************/
#ifndef INCLUDE_ETOL_ETOL_TYPES_HPP_
#define INCLUDE_ETOL_ETOL_TYPES_HPP_

#include <any>
#include <map>
#include <list>
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <utility>
#include <functional>

#define PARAM_PAIR ETOL::param_name_t, ETOL::param_configs_t
#define F_ARGS const ETOL::vector_t &x, const ETOL::vector_t &u, \
    const ETOL::vector_t &params, const std::vector<std::string> &pnames, \
    const std::any &k, const std::any &dt

namespace ETOL {

typedef enum {
    CONTINUOUS = 0,                         /**< Continuous variable type */
    INTERGER = 1,                           /**< Integer variable type */
    BINARY = 2,                             /**< Binary variable type */
} var_t;                                    /*!< ETOL variable types */


typedef std::string param_name_t;           /*!< Custom variable's name */

typedef struct {
    var_t varType = var_t::CONTINUOUS;      /**< Variable type*/
    double lbnd = 0.;                       /**< Lower bound */
    double ubnd = 0.;                       /**< Upper bound */
    double tStart = 0.;                     /**< Time it becomes active */
    double tStop = 0.;                      /**< Time it stops being active */
} param_configs_t;                          /*!< Custom variable's configs */

typedef std::pair<PARAM_PAIR> param_t;      /*!< Custom variable settings */

/*! Map of variable name to it settings */
typedef std::map<PARAM_PAIR> paramset_t;

typedef std::pair<double, double> coord_t;  /*!< A 2D coordinate */

typedef std::vector<coord_t> line_t;        /*!< A 2D line */

typedef std::vector<line_t> lines_t;        /*!< A collection of 2D lines */

typedef std::array<double, 3> corner_t;     /*!< A 3D coordinate */

struct edge_prop_ {
    double slope = 0.;
    double length = 0.;
};

typedef edge_prop_ edge_prop_t;

/*! A 3D coord with slope & len */
typedef std::pair<corner_t, edge_prop_t> edge_t;

/*! A segment formed from edges */
typedef std::vector<edge_t> seg_t;

/*!
 * A closure formed from a pair of segments. Typically, the first is the
 *     bottom segment and the second is the top segment
 */
typedef std::pair<std::vector<seg_t>, std::vector<seg_t>> closure_t;

/*!
 * The borders of a polygon that is formed from connecting corners
 */
typedef std::list<corner_t> border_t;

typedef std::vector<double> state_t;        /*!< A vector of variables */

typedef std::vector<var_t> state_var_t;     /*!< A vector of variable types */

/*! A trajectory's waypoint */
typedef std::pair<double, state_t> traj_elem_t;

/*!
 * A trajectory
 */
typedef std::vector<traj_elem_t> traj_t;

typedef struct {
    border_t lower;                         /**< A lower segment */
    border_t upper;                         /**< An upper segement */
} boundary_t;                               /*!< A boundary over a space */

typedef struct {
    double radius = 0.0;                    /**< Minimum separation distance */
    traj_t trajectory = traj_t();           /**< A trajectory */
} track_t;
/*!< The space restricted to a moving object */

typedef std::list<boundary_t> region_t;     /*!< A list of boundaries */

/*! Scalar data type that a ETOL callback should use */
typedef std::any scalar_t;

/*! Vector data type that a ETOL callback should use */
typedef std::vector<scalar_t> vector_t;

/*! Lambda expression type for ETOL callback should use */
typedef std::function< scalar_t (F_ARGS) > f_t;

} /* namespace ETOL */

#endif  // INCLUDE_ETOL_ETOL_TYPES_HPP_
