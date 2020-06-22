/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 26 April 2020
 * @version 1.0.0
 * @brief Typedefs that are shared to all eGurobi objects
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EGUROBI_TYPES_HPP_
#define INCLUDE_ETOL_EGUROBI_TYPES_HPP_

#include <gurobi_c++.h>
#include <vector>

namespace ETOL {

typedef struct {
    GRBLinExpr expr = GRBLinExpr();
    char sense = GRB_EQUAL;
    double rhs = 0.;
} grb_t;

typedef std::vector<grb_t> fout_grb_t;

} /* namespace ETOL */

#endif  // INCLUDE_ETOL_EGUROBI_TYPES_HPP_
