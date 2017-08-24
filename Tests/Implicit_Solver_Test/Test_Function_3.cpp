
#include "Test_Function_3.h"
#include "Implicit_Solver.h"
#include "GSL_Implicit_Solver.h"


TEST_F(TestFunction3, FunctionTest1)
{
	Vec3D InitGuess, ExpectedResult;
	InitGuess << 0.0, -1.0, 2.0  ;
	ExpectedResult << -21.0, 50.0, 12.0;
	auto funcx = TestFunction3::calcFunction(InitGuess);

	EXPECT_TRUE(funcx.isApprox(ExpectedResult));
}

TEST_F(TestFunction3, FunctionTest2)
{
	Vec3D InitGuess, ExpectedResult;
	InitGuess << 3.0, 1.0, -2.0;
	ExpectedResult << -12.0, 78.0, 15.0;
	auto funcx = TestFunction3::calcFunction(InitGuess);

	EXPECT_TRUE(funcx.isApprox(ExpectedResult));
}

TEST_F(TestFunction3, SolverTest1)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	Implicit_Solver<Precision> Solver(err, err, 1000);

	Vec3D InitGuess;
	InitGuess << 0.0, -1.0, 5.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
	static_assert(std::is_same<std::result_of_t<decltype(funcx)(Vec3D)>, Vec3D>::value);
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
	static_assert(std::is_same<std::result_of_t<decltype(funcjacobix)(Vec3D)>, JacobiMat>::value);
	const auto res = Solver.getResult(funcx, funcjacobix, InitGuess);

	EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest2)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	Implicit_Solver<Precision> Solver(err, err, 1000);

	Vec3D InitGuess;
	InitGuess << 10.0, 16.0, -10.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
	const auto res = Solver.getResult(funcx, funcjacobix, InitGuess);

	EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest3)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	Implicit_Solver<Precision> Solver(err, err, 1000);

	Vec3D InitGuess;
	InitGuess << 3.0, 5.0, 0.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
	const auto res = Solver.getResult(funcx, funcjacobix, InitGuess);

	EXPECT_TRUE(TestFunction3::isRoot(res));
}


TEST_F(TestFunction3, SolverTest1_GSL)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver<Precision> Solver(err, err, 1000,Vec3D::RowsAtCompileTime);

	Vec3D InitGuess;
	InitGuess << 0.0, -1.0, 5.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
	static_assert(std::is_same<std::result_of_t<decltype(funcx)(Vec3D)>, Vec3D>::value);
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
	static_assert(std::is_same<std::result_of_t<decltype(funcjacobix)(Vec3D)>, JacobiMat>::value);
	const auto res = Solver.getResult(funcx, funcjacobix, InitGuess);

	EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest2_GSL)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver<Precision> Solver(err, err, 1000, Vec3D::RowsAtCompileTime);

	Vec3D InitGuess;
	InitGuess << 10.0, 16.0, -10.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
	const auto res = Solver.getResult(funcx, funcjacobix, InitGuess);

	EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest3_GSL)
{
	const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
	const auto iter = 1000;
	GSL_Implicit_Solver<Precision> Solver(err, err, 1000, Vec3D::RowsAtCompileTime);

	Vec3D InitGuess;
	InitGuess << 0.0, -1.0, 5.0;

	auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
	auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
	const auto res = Solver.getResult(funcx, funcjacobix, InitGuess);

	EXPECT_TRUE(TestFunction3::isRoot(res));
}
