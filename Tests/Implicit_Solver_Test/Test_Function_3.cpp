
#include "Test_Function_3.h"
#include "MyCEL/math/Implicit_Solver.h"
#ifdef WITH_GSL_SOLVERS
#include "MyCEL/math/GSL_Implicit_Solver.h"
#include "MyCEL/math/GSL_Implicit_Solver_Derivative_Free.h"
#endif
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

// Test fails due to initial guess
TEST_F(TestFunction3, SolverTest1)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    //const auto iter = 1000;
    Implicit_Solver<Precision> Solver(err, err, 1000);

    Vec3D InitGuess;
    InitGuess << 0.0, -1.0, 5.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    static_assert(std::is_same<std::result_of_t<decltype(funcx)(Vec3D)>, Vec3D>::value);
    auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
    static_assert(std::is_same<std::result_of_t<decltype(funcjacobix)(Vec3D)>, JacobiMat>::value);
    auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction3::calcFunction(x), TestFunction3::calcFunctionJacobi(x)); };
    const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}

// Test fails due to initial guess
TEST_F(TestFunction3, SolverTest2)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    // const auto iter = 1000;
    Implicit_Solver<Precision> Solver(err, err, 1000);

    Vec3D InitGuess;
    InitGuess << 10.0, 16.0, -10.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
    auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction3::calcFunction(x), TestFunction3::calcFunctionJacobi(x)); };
    const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest3)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    // const auto iter = 1000;
    Implicit_Solver<Precision> Solver(err, err, 1000);

    Vec3D InitGuess;
    InitGuess << 3.0, 5.0, 0.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
    auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction3::calcFunction(x), TestFunction3::calcFunctionJacobi(x)); };
    const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}

#ifdef WITH_GSL_SOLVERS
TEST_F(TestFunction3, SolverTest1_GSL)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    const auto iter = 1000;
    GSL_Implicit_Solver<Precision> Solver(err, err, 1000,Vec3D::RowsAtCompileTime, gsl_solver_type::hybridsj);

    Vec3D InitGuess;
    InitGuess << 0.0, -1.0, 5.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    static_assert(std::is_same<std::result_of_t<decltype(funcx)(Vec3D)>, Vec3D>::value);
    auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
    static_assert(std::is_same<std::result_of_t<decltype(funcjacobix)(Vec3D)>, JacobiMat>::value);
    auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction3::calcFunction(x), TestFunction3::calcFunctionJacobi(x)); };
    const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest2_GSL)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    const auto iter = 1000;
    GSL_Implicit_Solver<Precision> Solver(err, err, 1000, Vec3D::RowsAtCompileTime, gsl_solver_type::hybridj);

    Vec3D InitGuess;
    InitGuess << 10.0, 16.0, -10.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
    auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction3::calcFunction(x), TestFunction3::calcFunctionJacobi(x)); };
    const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest3_GSL)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    const auto iter = 1000;
    GSL_Implicit_Solver<Precision> Solver(err, err, 1000, Vec3D::RowsAtCompileTime, gsl_solver_type::hybridsj);

    Vec3D InitGuess;
    InitGuess << 0.0, -1.0, 5.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    auto funcjacobix = [](const auto& x) -> auto { return TestFunction3::calcFunctionJacobi(x); };
    auto fdf = [](const auto& x) -> auto { return std::make_tuple(TestFunction3::calcFunction(x), TestFunction3::calcFunctionJacobi(x)); };
    const auto res = Solver.getResult(funcx, funcjacobix, fdf, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}



TEST_F(TestFunction3, SolverTest1_GSL2)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    const auto iter = 1000;
    GSL_Implicit_Solver_Derivative_Free<Precision> Solver(err, err, 1000, Vec3D::RowsAtCompileTime, gsl_solver_type_derivative_free::hybrids);

    Vec3D InitGuess;
    InitGuess << 0.0, -1.0, 5.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    const auto res = Solver.getResult(funcx, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest2_GSL2)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    const auto iter = 1000;
    GSL_Implicit_Solver_Derivative_Free<Precision> Solver(err, err, 1000, Vec3D::RowsAtCompileTime, gsl_solver_type_derivative_free::hybrid);

    Vec3D InitGuess;
    InitGuess << 10.0, 16.0, -10.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    const auto res = Solver.getResult(funcx, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}

TEST_F(TestFunction3, SolverTest3_GSL2)
{
    const auto err = std::numeric_limits<Precision>::epsilon() * 1000;
    const auto iter = 1000;
    GSL_Implicit_Solver_Derivative_Free<Precision> Solver(err, err, 1000, Vec3D::RowsAtCompileTime, gsl_solver_type_derivative_free::hybrids);

    Vec3D InitGuess;
    InitGuess << 0.0, -1.0, 5.0;

    auto funcx = [](const auto& x) -> auto { return TestFunction3::calcFunction(x); };
    const auto res = Solver.getResult(funcx, InitGuess);

    EXPECT_TRUE(TestFunction3::isRoot(res));
}
#endif
