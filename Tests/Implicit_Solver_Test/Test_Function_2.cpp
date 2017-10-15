
#include "Test_Function_2.h"
#include "../../Basic_Library/math/Implicit_Solver.h"
#include "../../Basic_Library/math/GSL_Implicit_Solver.h"
#include "../../Basic_Library/math/GSL_Implicit_Solver_Derivative_Free.h"

TEST_F(TestFunction2, FunctionTest1)
{
	Vec2D InitGuess, ExpectedResult;
	InitGuess << 0.0, -1.0;
	ExpectedResult << -11, -63;
	auto funcx = TestFunction2::calcFunction(InitGuess);

	EXPECT_TRUE(funcx.isApprox(ExpectedResult));
}

TEST_F(TestFunction2, FunctionTest2)
{
	Vec2D InitGuess, ExpectedResult;
	InitGuess << 2.0, -1.0;
	ExpectedResult << 21, -63;
	auto funcx = TestFunction2::calcFunction(InitGuess);

	EXPECT_TRUE(funcx.isApprox(ExpectedResult));
}

TEST_F(TestFunction2, SolverTest1)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	Implicit_Solver<Precision> Solver(err, err, 1000);

	Vec2D InitGuess;
	InitGuess << 0.0, -1.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	static_assert(std::is_same<std::result_of_t<decltype(funcx)(Vec2D)>, Vec2D>::value);
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction2::calcFunctionJacobi(x); };
	static_assert(std::is_same<std::result_of_t<decltype(funcjacobix)(Vec2D)>, JacobiMat>::value);
	auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction2::calcFunction(x), TestFunction2::calcFunctionJacobi(x)); };
	const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}

TEST_F(TestFunction2, SolverTest2)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	Implicit_Solver<Precision> Solver(err, err, 1000);

	Vec2D InitGuess;
	InitGuess << 10.0, 16.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction2::calcFunctionJacobi(x); };
	auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction2::calcFunction(x), TestFunction2::calcFunctionJacobi(x)); };
	const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}

TEST_F(TestFunction2, SolverTest3)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	Implicit_Solver<Precision> Solver(err, err, 1000);

	Vec2D InitGuess;
	InitGuess << 3.0, 5.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction2::calcFunctionJacobi(x); };
	auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction2::calcFunction(x), TestFunction2::calcFunctionJacobi(x)); };
	const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}


TEST_F(TestFunction2, SolverTest1_GSL)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver<Precision> Solver(err, err, 1000, Vec2D::RowsAtCompileTime, gsl_solver_type::newton);

	Vec2D InitGuess;
	InitGuess << 0.0, -1.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	static_assert(std::is_same<std::result_of_t<decltype(funcx)(Vec2D)>, Vec2D>::value);
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction2::calcFunctionJacobi(x); };
	static_assert(std::is_same<std::result_of_t<decltype(funcjacobix)(Vec2D)>, JacobiMat>::value);
	auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction2::calcFunction(x), TestFunction2::calcFunctionJacobi(x)); };
	const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}

TEST_F(TestFunction2, SolverTest2_GSL)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver<Precision> Solver(err, err, 1000, Vec2D::RowsAtCompileTime, gsl_solver_type::newton);

	Vec2D InitGuess;
	InitGuess << 10.0, 16.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction2::calcFunctionJacobi(x); };
	auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction2::calcFunction(x), TestFunction2::calcFunctionJacobi(x)); };
	const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}

TEST_F(TestFunction2, SolverTest3_GSL)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver<Precision> Solver(err, err, 1000, Vec2D::RowsAtCompileTime, gsl_solver_type::newton);

	Vec2D InitGuess;
	InitGuess << 3.0, 5.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction2::calcFunctionJacobi(x); };
	auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction2::calcFunction(x), TestFunction2::calcFunctionJacobi(x)); };
	const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}

TEST_F(TestFunction2, SolverTest1_GSL2)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver_Derivative_Free<Precision> Solver(err, err, 1000, Vec2D::RowsAtCompileTime, gsl_solver_type_derivative_free::dnewton);

	Vec2D InitGuess;
	InitGuess << 0.0, -1.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	const auto res = Solver.getResult(funcx, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}

TEST_F(TestFunction2, SolverTest2_GSL2)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver_Derivative_Free<Precision> Solver(err, err, 1000, Vec2D::RowsAtCompileTime, gsl_solver_type_derivative_free::dnewton);

	Vec2D InitGuess;
	InitGuess << 10.0, 16.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	const auto res = Solver.getResult(funcx, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}

TEST_F(TestFunction2, SolverTest3_GSL2)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver_Derivative_Free<Precision> Solver(err, err, 1000, Vec2D::RowsAtCompileTime, gsl_solver_type_derivative_free::dnewton);

	Vec2D InitGuess;
	InitGuess << 3.0, 5.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction2::calcFunction(x); };
	const auto res = Solver.getResult(funcx, InitGuess);

	EXPECT_TRUE(TestFunction2::isRoot(res));
}