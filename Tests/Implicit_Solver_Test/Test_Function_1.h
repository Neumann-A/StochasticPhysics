#pragma once

#include <gtest/gtest.h>

#include <Eigen/Core>

class TestFunction1 : public ::testing::Test
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

		res(0) = x(0) + 2.0*x(1) - 9.0;
		res(1) = 3.0*x(0) + x(1) + 6 * x(0)*x(1) - 64;

		return res;
	}

	template<class Derived>
	static JacobiMat calcFunctionJacobi(const MatBase<Derived> &x)
	{
		JacobiMat res;

		res(0, 0) = 1;
		res(0, 1) = 2;
		res(1, 0) = 3 + 6 * x(1);
		res(1, 1) = 1 + 6 * x(0);

		return res;
	};
	template<class Derived>
	static bool isRoot(const MatBase<Derived> &x)
	{
		//Zero crossings at x=7, y=1 and x = 17/6 , y = 37/12; Calculated with Mathematica!
		Vec2D root1,root2;
		root1 << 7.0, 1.0;
		root2 << 17.0 / 6.0, 37.0 / 12.0;

		std::cout << "Root1:\n" << root1 << "\n";
		std::cout << "Root2:\n" << root2 << "\n";
		std::cout << "Result:\n" << x << "\n";
		return (x.isApprox(root1) || x.isApprox(root2));
	};
};