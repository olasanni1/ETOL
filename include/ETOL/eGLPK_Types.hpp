/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Thanakorn Khamvilai
 * @author Thanakorn Khamvilai <thanakornkhamvilai@gmail.com>
 * @date 6 May 2020
 * @version 1.0.0
 * @brief Typedefs that are shared to all eGLPK objects
 ******************************************************************************/

#ifndef INCLUDE_ETOL_EGLPK_TYPES_HPP_
#define INCLUDE_ETOL_EGLPK_TYPES_HPP_

#include <glpk.h>
#include <any>
#include <string>
#include <utility>


namespace ETOL {

struct glpk_t {
    std::string _name;
    int _tIdx, _sIdx;
    double _obj_coeff;

    glpk_t(std::string name, int tIdx, int sIdx, double obj_coeff) : \
    _name(std::move(name)), _tIdx(tIdx), _sIdx(sIdx), _obj_coeff(obj_coeff) {}
};

struct glpk_bnd_t {
    double _lb, _ub;
    glpk_bnd_t(double lb, double ub) : _lb(lb), _ub(ub) {}
};

}  // namespace ETOL

#endif  // INCLUDE_ETOL_EGLPK_TYPES_HPP_
