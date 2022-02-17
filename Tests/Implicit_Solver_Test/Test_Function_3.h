#pragma once

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

class TestFunction3 : public ::testing::Test
{
public:
    using Precision = double;
    using Vec3D = Eigen::Matrix<Precision, 3, 1>;
    using JacobiMat = Eigen::Matrix<Precision, 3, 3>;
    template<class Derived>
    using MatBase = Eigen::MatrixBase<Derived>;

    template<class Derived>
    static Vec3D calcFunction(const MatBase<Derived> &x)
    {
        Vec3D res;

        res(0) = x(0)*x(0) - 3.0 *  x(2)*x(2) - 9;
        res(1) = 16.0 * x(1) + x(1)*x(1)*x(2) + 64;
        res(2) = x(0)*x(1)*x(1) - x(2)*x(2)   + 16;

        return res;
    }

    template<class Derived>
    static JacobiMat calcFunctionJacobi(const MatBase<Derived> &x)
    {
        JacobiMat res;

        res(0, 0) = 2.0*x(0);
        res(0, 1) = 0.0;
        res(0, 2) = -6.0*x(2);
        res(1, 0) = 0.0;
        res(1, 1) = 16.0 + 2.0 * x(1) * x(2);
        res(1, 2) = x(1) * x(1);
        res(2, 0) = x(1) * x(1);
        res(2, 1) = 2.0 * x(0) * x(1);
        res(2, 2) = -2.0 * x(2);

        return res;
    }
    template<class Derived>
    static bool isRoot(const MatBase<Derived> &x)
    {
        //Calculated with Mathematica!
        Vec3D root1, root2;
        root1 << 24.94264094051703985939357751254743582237, 2.748175016536519576254822119664834650189, -14.29609896775523355019361832067594174884;
        root2 << 15.18760735359791398897973420200247195760, -1.952310380253357906329267841085196680235, -8.59580163271701364049493070256259835979;

        std::cout << "Root1:\n" << root1 << "\n";
        std::cout << "Root2:\n" << root2 << "\n";
        std::cout << "Result:\n" << x << "\n";
        return (x.isApprox(root1) || x.isApprox(root2));
    }
};
