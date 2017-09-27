///-------------------------------------------------------------------------------------------------
// file:	Test_Neel_Spherical_Problem.cpp
//
// summary:	Implements the test neel spherical problem class
///-------------------------------------------------------------------------------------------------

#include "Test_Neel_Spherical_Problem.h"

#include <Eigen/Geometry>

using namespace Problems;

TEST_F(NeelSphericalProblemTest, CheckRotationFunctions)
{
	Matrix3x3 Rotation90DYAxis, BackRotation90DYAxis;
	Rotation90DYAxis <<	 0, 0, 1,
						 0, 1, 0,
						-1, 0, 0;
	
	BackRotation90DYAxis = Rotation90DYAxis.transpose();

	Vec2D testvec1, testvec2;
	testvec1 << 0.0, math::constants::pi<Precision>;
	testvec2 << math::constants::pi<Precision>, math::constants::pi<Precision>/2.0;
	
	const Vec3D expectedvec1 = math::coordinates::calcPointOnSphere(testvec1);
	const Vec3D expectedvec2 = math::coordinates::calcPointOnSphere(testvec2);

	const Vec2D rotatedvec1 = Rotate2DSphericalCoordinate90DegreeAroundYAxis(testvec1);
	const Vec2D rotatedvec2 = Rotate2DSphericalCoordinate90DegreeAroundYAxis(testvec2);

	Vec3D resultvec1 = BackRotation90DYAxis*math::coordinates::calcPointOnSphere(rotatedvec1);
	Vec3D resultvec2 = BackRotation90DYAxis*math::coordinates::calcPointOnSphere(rotatedvec2);
	
	EXPECT_TRUE(resultvec1.isApprox(expectedvec1));
	EXPECT_TRUE(resultvec2.isApprox(expectedvec2));

	resultvec1 = math::coordinates::calcPointOnSphere(inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(rotatedvec1));
	resultvec2 = math::coordinates::calcPointOnSphere(inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(rotatedvec2));

	EXPECT_TRUE(resultvec1.isApprox(expectedvec1));
	EXPECT_TRUE(resultvec2.isApprox(expectedvec2));
}

TEST_F(NeelSphericalProblemTest, CheckRotationFunctionsRandomInput)
{
	Matrix3x3 Rotation90DYAxis, BackRotation90DYAxis;
	Rotation90DYAxis << 0, 0, 1,
		0, 1, 0,
		-1, 0, 0;

	BackRotation90DYAxis = Rotation90DYAxis.transpose();

	Vec2D RandomVec, RotatedVec;
	Vec3D ExpectedVec, ResultVec;

	for (auto i = 1'000'000; --i;)
	{
		RandomVec = getRandomCoords();
		ExpectedVec = math::coordinates::calcPointOnSphere(RandomVec);
		RotatedVec = Rotate2DSphericalCoordinate90DegreeAroundYAxis(RandomVec);
		ResultVec = BackRotation90DYAxis*math::coordinates::calcPointOnSphere(RotatedVec);

		//NOTE: If test fail reduce the precission goal in the check. 
		EXPECT_TRUE(ResultVec.isApprox(ExpectedVec, 1.0E-6));

		if (!ResultVec.isApprox(ExpectedVec, 1.0E-6))
		{
			std::cout <<"Result:\t"<< ResultVec.transpose() << "\n";
			std::cout <<"Expected:\t" << ExpectedVec.transpose() << "\n";
		}

		ResultVec = math::coordinates::calcPointOnSphere(inverseRotate2DSphericalCoordinate90DegreeAroundYAxis(RotatedVec));

		EXPECT_TRUE(ResultVec.isApprox(ExpectedVec,1.0E-6));

		if (!ResultVec.isApprox(ExpectedVec, 1.0E-6))
		{
			std::cout << "Result:\t" << ResultVec.transpose() << "\n";
			std::cout << "Expected:\t" << ExpectedVec.transpose() << "\n";
		}

		
	}
}

TEST_F(NeelSphericalProblemTest, DeterministicVectorTestWithoutField)
{
	constexpr auto pi = math::constants::pi<Precision>;

	////Testcalculations have been performed in Mathematic testres1 is the result obtained by it!

	Vec2D testy1, testres1;
	testy1 << pi / 2.5, pi / 3.0;
	testy1 = math::coordinates::Wrap2DSphericalCoordinates(testy1);
	testres1 << 2.9523604146293106E9, -8.799697720531372E8;
	prepareCalculations(testy1);
	const auto resvec1 = getDeterministicVector(testy1, Vec3D::Zero());
	EXPECT_FALSE(isRotated);
	EXPECT_TRUE(resvec1.isApprox(testres1));
	if (!resvec1.isApprox(testres1))
	{
		std::cout << "Result:\t" << resvec1.transpose() << "\n";
		std::cout << "Expected:\t" << testres1.transpose() << "\n";
	}

	Vec2D testy2, testres2;
	testy2 << pi / 7.0, pi / 12.0;
	testy2 = math::coordinates::Wrap2DSphericalCoordinates(testy2);
	testres2 << -2.7775066442414486E8, -3.0591291325311937E9;
	prepareCalculations(testy2);
	const auto resvec2 = getDeterministicVector(testy2, Vec3D::Zero());
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(resvec2.isApprox(testres2));
	if (!resvec2.isApprox(testres2))
	{
		std::cout << "Result:\t" << resvec2.transpose() << "\n";
		std::cout << "Expected:\t" << testres2.transpose() << "\n";
	}

	Vec2D testy3, testres3;
	testy3 << 0.0, 2.0*pi / 3.0;
	testy3 = math::coordinates::Wrap2DSphericalCoordinates(testy3);
	testres3 << 0.0, 0.0;
	prepareCalculations(testy3);
	const auto resvec3 = getDeterministicVector(testy3, Vec3D::Zero());
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE((resvec3- testres3).norm() < 1E-6 );
	if (!((resvec3 - testres3).norm() < 1E-6))
	{
		std::cout << "Result:\t" << resvec3.transpose() << "\n";
		std::cout << "Expected:\t" << testres3.transpose() << "\n";
		std::cout << "Norm diff:\t" << (resvec3 - testres3).norm() << "\n";
	}

	Vec2D testy4, testres4;
	testy4 << pi, 2.0*pi / 1.2;
	testy4 = math::coordinates::Wrap2DSphericalCoordinates(testy4);
	testres4 << 0.0, 0.0;
	prepareCalculations(testy4);
	const auto resvec4 = getDeterministicVector(testy4, Vec3D::Zero());
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE((resvec4 - testres4).norm() < 2E-6);
	//EXPECT_TRUE(resvec4.isApprox(testres4,1E-6));
	if (!((resvec4 - testres4).norm() < 2E-6))
	{
		std::cout << "Result:\t" << resvec4.transpose() << "\n";
		std::cout << "Expected:\t" << testres4.transpose() << "\n";
		std::cout << "Norm diff:\t" << (resvec4 - testres4).norm() << "\n";
	}

	Vec2D testy5, testres5;
	testy5 << pi *15.0/16.0, 2.0*pi / 7.0;
	testy5 = math::coordinates::Wrap2DSphericalCoordinates(testy5);
	testres5 << -8.812698800146618E7, -8.878625493152951E8;
	prepareCalculations(testy5);
	const auto resvec5 = getDeterministicVector(testy5, Vec3D::Zero());
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(resvec5.isApprox(testres5));
	if (!resvec5.isApprox(testres5))
	{
		std::cout << "Result:\t" << resvec5.transpose() << "\n";
		std::cout << "Expected:\t" << testres5.transpose() << "\n";
	}

	Vec2D testy6, testres6;
	testy6 << pi *3.0 / 7.0, 2.0*pi / 7.0;
	testy6 = math::coordinates::Wrap2DSphericalCoordinates(testy6);
	testres6 << 3.4073733237535477E9, -9.872223429902736E8;
	prepareCalculations(testy6);
	const auto resvec6 = getDeterministicVector(testy6, Vec3D::Zero());
	EXPECT_FALSE(isRotated);
	EXPECT_TRUE(resvec6.isApprox(testres6));
	if (!resvec6.isApprox(testres6))
	{
		std::cout << "Result:\t" << resvec6.transpose() << "\n";
		std::cout << "Expected:\t" << testres6.transpose() << "\n";
	}

	Vec2D testy7, testres7;
	testy7 << pi *9.0 / 12.0, 2.0*pi / 3.0;
	testy7 = math::coordinates::Wrap2DSphericalCoordinates(testy7);
	testres7 << -2.1437010663944268E9, 1.606413037557967E9;
	prepareCalculations(testy7);
	const auto resvec7 = getDeterministicVector(testy7, Vec3D::Zero());
	EXPECT_FALSE(isRotated);
	EXPECT_TRUE(resvec7.isApprox(testres7));
	if (!resvec7.isApprox(testres7))
	{
		std::cout << "Result:\t" << resvec7.transpose() << "\n";
		std::cout << "Expected:\t" << testres7.transpose() << "\n";
	}

};

TEST_F(NeelSphericalProblemTest, DeterministicVectorTestWithField)
{
	constexpr auto pi = math::constants::pi<Precision>;

	////Testcalculations have been performed in Mathematic testres1 is the result obtained by it!
	Vec3D Testfield;
	Testfield << 0.015, -0.007, 0.003;

	Vec2D testy1, testres1;
	testy1 << pi / 2.5, pi / 3.0;
	testy1 = math::coordinates::Wrap2DSphericalCoordinates(testy1);
	testres1 << 3.68129216700612E7, -1.019187859953005E9;
	prepareCalculations(testy1);
	const auto resvec1 = getDeterministicVector(testy1, Testfield);
	EXPECT_FALSE(isRotated);
	EXPECT_TRUE(resvec1.isApprox(testres1));
	if (!resvec1.isApprox(testres1))
	{
		std::cout << "Result:\t" << resvec1.transpose() << "\n";
		std::cout << "Expected:\t" << testres1.transpose() << "\n";
	}

	Vec2D testy2, testres2;
	testy2 << pi / 7.0, pi / 12.0;
	testy2 = math::coordinates::Wrap2DSphericalCoordinates(testy2);
	testres2 << -1.3309402097673228E9, -4.746447228306928E8;
	prepareCalculations(testy2);
	const auto resvec2 = getDeterministicVector(testy2, Testfield);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(resvec2.isApprox(testres2));
	if (!resvec2.isApprox(testres2))
	{
		std::cout << "Result:\t" << resvec2.transpose() << "\n";
		std::cout << "Expected:\t" << testres2.transpose() << "\n";
	}

	Vec2D testy3, testres3;
	testy3 << 0.0, 2.0*pi / 3.0;
	testy3 = math::coordinates::Wrap2DSphericalCoordinates(testy3);
	testres3 << -9.584158415841583E8, 2.735841584158416E9;
	prepareCalculations(testy3);
	const auto resvec3 = getDeterministicVector(testy3, Testfield);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(resvec3.isApprox(testres3));
	if (!resvec3.isApprox(testres3))
	{
		std::cout << "Result:\t" << resvec3.transpose() << "\n";
		std::cout << "Expected:\t" << testres3.transpose() << "\n";
	}

	Vec2D testy4, testres4;
	testy4 << pi, 2.0*pi / 1.2;
	testy4 = math::coordinates::Wrap2DSphericalCoordinates(testy4);
	testres4 << 1.481188118811881E9, 2.4918811881188116E9;
	prepareCalculations(testy4);
	const auto resvec4 = getDeterministicVector(testy4, Testfield);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(resvec4.isApprox(testres4));
	if (!resvec4.isApprox(testres4))
	{
		std::cout << "Result:\t" << resvec4.transpose() << "\n";
		std::cout << "Expected:\t" << testres4.transpose() << "\n";
	}

	Vec2D testy5, testres5;
	testy5 << pi *15.0 / 16.0, 2.0*pi / 7.0;
	testy5 = math::coordinates::Wrap2DSphericalCoordinates(testy5);
	testres5 << 1.3048614971882508E9, 1.6989332374057689E9;
	prepareCalculations(testy5);
	const auto resvec5 = getDeterministicVector(testy5, Testfield);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(resvec5.isApprox(testres5));
	if (!resvec5.isApprox(testres5))
	{
		std::cout << "Result:\t" << resvec5.transpose() << "\n";
		std::cout << "Expected:\t" << testres5.transpose() << "\n";
	}

	Vec2D testy6, testres6;
	testy6 << pi *3.0 / 7.0, 2.0*pi / 7.0;
	testy6 = math::coordinates::Wrap2DSphericalCoordinates(testy6);
	testres6 << 5.673168013363386E8, -1.0680690672779332E9;
	prepareCalculations(testy6);
	const auto resvec6 = getDeterministicVector(testy6, Testfield);
	EXPECT_FALSE(isRotated);
	EXPECT_TRUE(resvec6.isApprox(testres6));
	if (!resvec6.isApprox(testres6))
	{
		std::cout << "Result:\t" << resvec6.transpose() << "\n";
		std::cout << "Expected:\t" << testres6.transpose() << "\n";
	}

	Vec2D testy7, testres7;
	testy7 << pi *9.0 / 12.0, 2.0*pi / 3.0;
	testy7 = math::coordinates::Wrap2DSphericalCoordinates(testy7);
	testres7 << -3.667324754358012E9, 3.680829263367496E9;
	prepareCalculations(testy7);
	const auto resvec7 = getDeterministicVector(testy7, Testfield);
	EXPECT_FALSE(isRotated);
	EXPECT_TRUE(resvec7.isApprox(testres7));
	if (!resvec7.isApprox(testres7))
	{
		std::cout << "Result:\t" << resvec7.transpose() << "\n";
		std::cout << "Expected:\t" << testres7.transpose() << "\n";
	}
};

TEST_F(NeelSphericalProblemTest, StochasticMatrixTest)
{
	constexpr auto pi = math::constants::pi<Precision>;


	Vec2D TestInput1,TestInput2;
	TestInput1 << pi *9.0 / 12.0, 2.0*pi / 3.0; //Not Rotated
	TestInput2 << pi *15.0 / 16.0, 2.0*pi / 7.0; // Rotated

	Matrix2x3 Expected1, Expected2;
	Expected1 << -6963.321900495292, - 4704.726648377463, - 592.7518451088552,
				   5218.06480438183, -6666.9459779408635, -8382.776984746191;
	Expected2 << 832.0532128800853, -8298.878718999289, -1187.4169166838587,
				 8382.776984746191, 676.6563125993357, 1144.8614477421022;

	prepareCalculations(TestInput1);
	const auto calcVal1 = getStochasticMatrix(TestInput1);
	EXPECT_TRUE(!isRotated);
	EXPECT_TRUE(calcVal1.isApprox(Expected1));
	if (!calcVal1.isApprox(Expected1))
	{
		std::cout << "Result:\n" << calcVal1 << "\n";
		std::cout << "Expected:\n" << Expected1 << "\n";
	}

	prepareCalculations(TestInput2);
	const auto calcVal2 = getStochasticMatrix(TestInput2);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(calcVal2.isApprox(Expected2));
	if (!calcVal2.isApprox(Expected2))
	{
		std::cout << "Result:\n" << calcVal2 << "\n";
		std::cout << "Expected:\n" << Expected2 << "\n";
	}
};

TEST_F(NeelSphericalProblemTest, DriftTest)
{
	constexpr auto pi = math::constants::pi<Precision>;

	////Testcalculations have been performed in Mathematic testres1 is the result obtained by it!

	Vec2D testy1, testres1;
	testy1 << pi / 2.5, pi / 3.0;
	testy1 = math::coordinates::Wrap2DSphericalCoordinates(testy1);
	testres1 << -1.1530369938699268E7, 0;
	prepareCalculations(testy1);
	const auto resvec1 = getDrift(testy1);
	EXPECT_TRUE(resvec1.isApprox(testres1));
	if (!resvec1.isApprox(testres1))
	{
		std::cout << "Result:\t" << resvec1.transpose() << "\n";
		std::cout << "Expected:\t" << testres1.transpose() << "\n";
	}
	
	Vec2D testy2, testres2;
	testy2 << pi / 7.0, pi / 12.0;
	testy2 = math::coordinates::Wrap2DSphericalCoordinates(testy2);
	testres2 << 1.6380496393433755E7, 0;
	prepareCalculations(testy2);
	const auto resvec2 = getDrift(testy2);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(resvec2.isApprox(testres2));
	if (!resvec2.isApprox(testres2))
	{
		std::cout << "Result:\t" << resvec2.transpose() << "\n";
		std::cout << "Expected:\t" << testres2.transpose() << "\n";
	}

	Vec2D testy3, testres3;
	testy3 << 0.0, 2.0*pi / 3.0;
	testy3 = math::coordinates::Wrap2DSphericalCoordinates(testy3);
	testres3 << 0.0, 0.0;
	prepareCalculations(testy3);
	const auto resvec3 = getDrift(testy3);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE((resvec3 - testres3).norm() < 1E-6);
	if (!((resvec3 - testres3).norm() < 1E-6))
	{
		std::cout << "Result:\t" << resvec3.transpose() << "\n";
		std::cout << "Expected:\t" << testres3.transpose() << "\n";
		std::cout << "Norm diff:\t" << (resvec3 - testres3).norm() << "\n";
	}

	Vec2D testy4, testres4;
	testy4 << pi, 2.0*pi / 1.2;
	testy4 = math::coordinates::Wrap2DSphericalCoordinates(testy4);
	testres4 << -2.1729416225187967E-9, 0;
	prepareCalculations(testy4);
	const auto resvec4 = getDrift(testy4);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE((resvec4 - testres4).norm() < 2E-6);
	//EXPECT_TRUE(resvec4.isApprox(testres4,1E-6));
	if (!((resvec4 - testres4).norm() < 2E-6))
	{
		std::cout << "Result:\t" << resvec4.transpose() << "\n";
		std::cout << "Expected:\t" << testres4.transpose() << "\n";
		std::cout << "Norm diff:\t" << (resvec4 - testres4).norm() << "\n";
	}

	Vec2D testy5, testres5;
	testy5 << pi *15.0 / 16.0, 2.0*pi / 7.0;
	testy5 = math::coordinates::Wrap2DSphericalCoordinates(testy5);
	testres5 << 4.348796576254137E6, 0.0;
	prepareCalculations(testy5);
	const auto resvec5 = getDrift(testy5);
	EXPECT_TRUE(isRotated);
	EXPECT_TRUE(resvec5.isApprox(testres5));
	if (!resvec5.isApprox(testres5))
	{
		std::cout << "Result:\t" << resvec5.transpose() << "\n";
		std::cout << "Expected:\t" << testres5.transpose() << "\n";
	}

	Vec2D testy6, testres6;
	testy6 << pi *3.0 / 7.0, 2.0*pi / 7.0;
	testy6 = math::coordinates::Wrap2DSphericalCoordinates(testy6);
	testres6 << -8.099637314464323E6, 0.0;
	prepareCalculations(testy6);
	const auto resvec6 = getDrift(testy6);
	EXPECT_FALSE(isRotated);
	EXPECT_TRUE(resvec6.isApprox(testres6));
	if (!resvec6.isApprox(testres6))
	{
		std::cout << "Result:\t" << resvec6.transpose() << "\n";
		std::cout << "Expected:\t" << testres6.transpose() << "\n";
	}

	Vec2D testy7, testres7;
	testy7 << pi *9.0 / 12.0, 2.0*pi / 3.0;
	testy7 = math::coordinates::Wrap2DSphericalCoordinates(testy7);
	testres7 << 3.548682973787517E7, 0.0;
	prepareCalculations(testy7);
	const auto resvec7 = getDrift(testy7);
	EXPECT_FALSE(isRotated);
	EXPECT_TRUE(resvec7.isApprox(testres7));
	if (!resvec7.isApprox(testres7))
	{
		std::cout << "Result:\t" << resvec7.transpose() << "\n";
		std::cout << "Expected:\t" << testres7.transpose() << "\n";
	}

};

TEST_F(NeelSphericalProblemTest, JacobiTestWithOutField)
{
	constexpr auto pi = math::constants::pi<Precision>;

	{
		Vec2D TestInput(pi *9.0 / 12.0, 2.0*pi / 3.0); //Not Rotated
		JacobiMatrixType TestOutput;
		TestOutput << 2.234942199622573E9, -2.2646204041680737E9, 1.290344480575347E9, 4.83484893215773E9;
		prepareCalculations(TestInput);
		prepareJacobiCalculations(TestInput);
		const auto ResultTest = getJacobiDeterministic(TestInput, Vec3D::Zero(), 0.0);
		EXPECT_TRUE(TestOutput.isApprox(ResultTest));
		if (!TestOutput.isApprox(ResultTest))
		{
			std::cout << "Expected:\n" << TestOutput << "\n";
			std::cout << "Result:\n" << ResultTest << "\n";
		}
	}
	{
		Vec2D TestInput(pi *15.0 / 16.0, 2.0*pi / 7.0); //Rotated
		JacobiMatrixType TestOutput;
		TestOutput << 7.083297093017924E8, 0.0, 7.245091042020364E9, 0.0;
		prepareCalculations(TestInput);
		prepareJacobiCalculations(TestInput);
		const auto ResultTest = getJacobiDeterministic(TestInput, Vec3D::Zero(), 0.0);
		EXPECT_TRUE(TestOutput.isApprox(ResultTest));
		if (!TestOutput.isApprox(ResultTest))
		{
			std::cout << "Expected:\n" << TestOutput << "\n";
			std::cout << "Result:\n" << ResultTest << "\n";
		}
	}
}

TEST_F(NeelSphericalProblemTest, JacobiTestWithField)
{
	constexpr auto pi = math::constants::pi<Precision>;

	Vec3D TestField(0.1, -0.3, 0.7);

	{
		Vec2D TestInput(pi *9.0 / 12.0, 2.0*pi / 3.0); //Not Rotated
		JacobiMatrixType TestOutput;
		TestOutput << 1.4677651082925764E10, 5.094048346148586E10, 1.0770055221188318E11, -1.3847441908663843E10;
		prepareCalculations(TestInput);
		prepareJacobiCalculations(TestInput);
		const auto ResultTest = getJacobiDeterministic(TestInput, TestField, 0.0);
		EXPECT_TRUE(TestOutput.isApprox(ResultTest));
		if (!TestOutput.isApprox(ResultTest))
		{
			std::cout << "Expected:\n" << TestOutput << "\n";
			std::cout << "Result:\n" << ResultTest << "\n";
		}
	}
	{
		Vec2D TestInput(pi *15.0 / 16.0, 2.0*pi / 7.0); //Rotated
		JacobiMatrixType TestOutput;
		TestOutput << -1.02459745386672E10, -1.1335432239931761E11, -1.0781155396106686E11, 2.706559941754919E9;
		prepareCalculations(TestInput);
		prepareJacobiCalculations(TestInput);
		const auto ResultTest = getJacobiDeterministic(TestInput, TestField, 0.0);
		EXPECT_TRUE(TestOutput.isApprox(ResultTest));
		if (!TestOutput.isApprox(ResultTest))
		{
			std::cout << "Expected:\n" << TestOutput << "\n";
			std::cout << "Result:\n" << ResultTest << "\n";
		}
	}
}

TEST_F(NeelSphericalProblemTest, StochasticJacobiTest)
{
	constexpr auto pi = math::constants::pi<Precision>;

	Vec3D TestField(0.5, -0.7, 0.3);

	{
		Vec2D TestInput(pi *9.0 / 12.0, 2.0*pi / 3.0); //Not Rotated
		JacobiMatrixType TestOutput;
		TestOutput <<	685.3502240129275, 7226.688654535435,
						14453.37730907087, - 319.1723740968496;
		prepareCalculations(TestInput);
		prepareJacobiCalculations(TestInput);
		const auto ResultTest = getJacobiStochastic(TestField);
		EXPECT_TRUE(TestOutput.isApprox(ResultTest));
		if (!TestOutput.isApprox(ResultTest))
		{
			std::cout << "Expected:\n" << TestOutput << "\n";
			std::cout << "Result:\n" << ResultTest << "\n";
		}
	}
	{
		Vec2D TestInput(pi *15.0 / 16.0, 2.0*pi / 7.0); //Rotated
		JacobiMatrixType TestOutput;
		TestOutput <<	-106.16597771249461, -1658.4717740210835,
						-1683.3782260104874, - 598.4061196396729;
		prepareCalculations(TestInput);
		prepareJacobiCalculations(TestInput);
		const auto ResultTest = getJacobiStochastic(TestField);
		EXPECT_TRUE(TestOutput.isApprox(ResultTest));
		if (!TestOutput.isApprox(ResultTest))
		{
			std::cout << "Expected:\n" << TestOutput << "\n";
			std::cout << "Result:\n" << ResultTest << "\n";
		}
	}
}

TEST_F(NeelSphericalProblemTest, finishCalculations)
{
	constexpr auto pi = math::constants::pi<Precision>;

	//Case1
	{
		Vec2D testy1, testres1;
		testy1 << pi / 2.5, pi / 3.0;
		testy1 = math::coordinates::Wrap2DSphericalCoordinates(testy1);
		testres1 = testy1;
		prepareCalculations(testy1);
		Vec2D resvec1 = testy1;
		finishCalculations(resvec1);
		EXPECT_FALSE(isRotated);
		EXPECT_TRUE(resvec1.isApprox(testres1));
		if (!resvec1.isApprox(testres1))
		{
			std::cout << "Result:\t" << resvec1.transpose() << "\n";
			std::cout << "Expected:\t" << testres1.transpose() << "\n";
		}
	}
	//Case2
	{
		Vec2D testy2, testres2;
		testy2 << pi / 7.0, pi / 12.0;
		testy2 = math::coordinates::Wrap2DSphericalCoordinates(testy2);
		testres2 = testy2;
		prepareCalculations(testy2);
		Vec2D resvec2 = testy2;
		finishCalculations(resvec2);
		resvec2 = math::coordinates::Wrap2DSphericalCoordinates(resvec2);
		EXPECT_TRUE(isRotated);
		EXPECT_TRUE(resvec2.isApprox(testres2));
		if (!resvec2.isApprox(testres2))
		{
			std::cout << "Result:\t" << resvec2.transpose() << "\n";
			std::cout << "Expected:\t" << testres2.transpose() << "\n";
		}
	}
	//Case3 (Special case will not conserve phi because at theta = 0 phi can be any value)
	{
		Vec2D testy3, testres3;
		testy3 << 0.0, 2.0*pi / 3.0;
		testy3 = math::coordinates::Wrap2DSphericalCoordinates(testy3);
		testres3 = testy3;
		prepareCalculations(testy3);
		Vec2D resvec3 = testy3;
		finishCalculations(resvec3);
		resvec3 = math::coordinates::Wrap2DSphericalCoordinates(resvec3);
		EXPECT_TRUE(isRotated);
		EXPECT_DOUBLE_EQ(resvec3(0), testy3(0));
		//EXPECT_TRUE(resvec3.isApprox(testres3));
		//if (!resvec3.isApprox(testres3))
		//{
		//	std::cout << "Result:\t" << resvec3.transpose() << "\n";
		//	std::cout << "Expected:\t" << testres3.transpose() << "\n";
		//}
	}
	//Case4 (Special case will not conserve phi because at theta = pi phi can be any value)
	{
		Vec2D testy4, testres4;
		testy4 << pi, 2.0*pi / 1.2;
		testy4 = math::coordinates::Wrap2DSphericalCoordinates(testy4);
		testres4 = testy4;
		prepareCalculations(testy4);
		Vec2D resvec4 = testy4;
		finishCalculations(resvec4);
		resvec4 = math::coordinates::Wrap2DSphericalCoordinates(resvec4);
		EXPECT_TRUE(isRotated);
		EXPECT_DOUBLE_EQ(resvec4(0), testy4(0));
		//EXPECT_TRUE(resvec4.isApprox(testres4));
		//if (!resvec4.isApprox(testres4))
		//{
		//	std::cout << "Result:\t" << resvec4.transpose() << "\n";
		//	std::cout << "Expected:\t" << testres4.transpose() << "\n";
		//}
	}
	//Case5
	{
		Vec2D testy5, testres5;
		testy5 << pi *15.0 / 16.0, 2.0*pi / 7.0;
		testy5 = math::coordinates::Wrap2DSphericalCoordinates(testy5);
		testres5 = testy5;
		prepareCalculations(testy5);
		Vec2D resvec5 = testy5;
		finishCalculations(resvec5);
		resvec5 = math::coordinates::Wrap2DSphericalCoordinates(resvec5);
		EXPECT_TRUE(isRotated);
		EXPECT_TRUE(resvec5.isApprox(testres5));
		if (!resvec5.isApprox(testres5))
		{
			std::cout << "Result:\t" << resvec5.transpose() << "\n";
			std::cout << "Expected:\t" << testres5.transpose() << "\n";
		}
	}
	//Case6
	{
		Vec2D testy6, testres6;
		testy6 << pi *3.0 / 7.0, 2.0*pi / 7.0;
		testy6 = math::coordinates::Wrap2DSphericalCoordinates(testy6);
		testres6 = testy6;
		prepareCalculations(testy6);
		Vec2D resvec6 = testy6;
		finishCalculations(resvec6);
		EXPECT_FALSE(isRotated);
		EXPECT_TRUE(resvec6.isApprox(testres6));
		if (!resvec6.isApprox(testres6))
		{
			std::cout << "Result:\t" << resvec6.transpose() << "\n";
			std::cout << "Expected:\t" << testres6.transpose() << "\n";
		}
	}
	//Case7
	{
		Vec2D testy7, testres7;
		testy7 << pi *9.0 / 12.0, 2.0*pi / 3.0;
		testy7 = math::coordinates::Wrap2DSphericalCoordinates(testy7);
		testres7 = testy7;
		prepareCalculations(testy7);
		auto resvec7 = testy7;
		finishCalculations(resvec7);
		EXPECT_FALSE(isRotated);
		EXPECT_TRUE(resvec7.isApprox(testres7));
		if (!resvec7.isApprox(testres7))
		{
			std::cout << "Result:\t" << resvec7.transpose() << "\n";
			std::cout << "Expected:\t" << testres7.transpose() << "\n";
		}
	}
};