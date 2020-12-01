#pragma once

#include <Eigen/Core>       // for more vector/matrix math
#include <Eigen/StdVector>  // for Eigen std::vector allocator
#include <Eigen/Geometry>   // for cross product

#ifndef INC_GENERAL_MATHTYPES_HPP
#define INC_GENERAL_MATHTYPES_HPP

// This definitions are used to wrap the Matrix types used by the simulation and easily swap them if necessary. 

namespace SPhys::math {

    template<typename T>
    using MatrixBase = Eigen::MatrixBase<T>

    template<typename T, int i, int j>
    using Matrix = Eigen::Matrix<prec, i, j>

    template<typename T, int i>
    using ColVector = Matrix<T,i,1>

    template<typename T, int i>
    using RowVector = Matrix<T,1,i>

    template<typename T>
    using Vector3D = ColVector<T,3,1>

}


#endif //INC_GENERAL_MATHTYPES_HPP

