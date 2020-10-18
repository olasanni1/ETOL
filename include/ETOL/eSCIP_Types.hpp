/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 16 October 2020
 * @version 1.0.0
 * @brief Typedefs that are shared to all eSCIP objects
 ******************************************************************************/

#ifndef INCLUDE_ETOL_ESCIP_TYPES_HPP_
#define INCLUDE_ETOL_ESCIP_TYPES_HPP_

#include <scip/scip.h>
#include <vector>
#include <utility>
#include <numeric>

#define ESCIP_INF std::numeric_limits<SCIP_Real>::infinity()

namespace ETOL {

typedef std::pair<SCIP_VAR*, SCIP_Real> scip_var_t;
typedef std::vector<scip_var_t> scip_expr_t;

typedef struct {
    scip_expr_t expr = {};
    SCIP_Real lhs = 0.;
    SCIP_Real rhs = 0.;
} scip_t;

typedef std::vector<scip_t> fout_scip_t;

}  // namespace ETOL

#endif  // INCLUDE_ETOL_EGUROBI_TYPES_HPP_
