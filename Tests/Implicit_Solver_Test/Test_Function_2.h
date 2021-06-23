#pragma once

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

class TestFunction2 : public ::testing::Test
{
public:
	using Precision = double;
	using Vec2D = Eigen::Matrix<Precision, 2, 1>;
	using JacobiMat = Eigen::Matrix<Precision, 2, 2>;
	template<class Derived>
	using MatBase = Eigen::MatrixBase<Derived>;

	template<class Derived>
	static Vec2D calcFunction(const MatBase<Derived> &x)
	{
		Vec2D res;

		res(0) = 16.0*x(0) + 2.0*std::pow(x(1),3) - 9.0;
		res(1) = 3*x(0)*x(0) + std::pow(x(1),4) + 6 * x(0)*x(1) - 64;

		return res;
	}

	template<class Derived>
	static JacobiMat calcFunctionJacobi(const MatBase<Derived> &x)
	{
		JacobiMat res;

		res(0, 0) = 16.0;
		res(0, 1) = 6.0*std::pow(x(1),2);
		res(1, 0) = 6.0*x(0)+6.0*x(1);
		res(1, 1) = 6.0*x(0)+4.0*std::pow(x(1),3);

		return res;
	};
	template<class Derived>
	static bool isRoot(const MatBase<Derived> &x)
	{
		//Calculated with Mathematica!
		Vec2D root1, root2;
		root1 << 4.171693661908896399197322740218308796100, -3.067844841059378634654464094551742447729;
		root2 << -3.177905973754653339504659865520039419155, 3.104580387803790513959254734612565911544;

		std::cout << "Root1:\n" << root1 << "\n";
		std::cout << "Root2:\n" << root2 << "\n";
		std::cout << "Result:\n" << x << "\n";
		return (x.isApprox(root1) || x.isApprox(root2));
	};
};
