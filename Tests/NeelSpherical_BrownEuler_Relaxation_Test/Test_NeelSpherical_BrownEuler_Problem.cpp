///-------------------------------------------------------------------------------------------------
// file:	Test_NeelSpherical_BrownEuler_Problem.cpp
//
// summary:	Implements the tests to run
///-------------------------------------------------------------------------------------------------

#include "Test_NeelSpherical_BrownEuler_Problem.h"

#include <limits>

#include "math/ApproxJacobian.h"

#include <Eigen/Geometry>

using namespace Problems;

TEST_F(NeelSpherical_BrownEuler_ProblemTest, EmptyTest)
{

}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicNotRotatedWithoutField)
{
	Vec5D state;
	state << 2.827433388230814, 0.6283185307179586, 0.2617993877991494, 2.199114857512855, 3.365992128846207;
	Vec5D expectedresult;
	expectedresult << -5.1031048783581274E-11, 5.045902371131142E-13, 2.673307047832739E-11, 2.897753312309516E9, -2.7768353291032495E9;
	
	prepareCalculations(state);
	const auto result = getDeterministicVector(state, Vec3D::Zero());
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}

	//EXPECT_TRUE((result- expectedresult).norm()/ expectedresult.norm() < 1E-6);
	//if (!((result - expectedresult).norm() / expectedresult.norm() < 1E-6))
	//{
	//	std::cout << "Relative Error: " << (result - expectedresult).norm() / expectedresult.norm() <<'\n';
	//	std::cout << "Absolute Error: " << (result - expectedresult).norm() << '\n';
	//	std::cout << "Norm Expected " << (expectedresult).norm() << '\n';
	//	std::cout << "Result:\t" << result.transpose() << "\n";
	//	std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	//}

}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicNotRotated)
{
	Vec5D state;
	state << 2.827433388230814, 0.6283185307179586, 0.2617993877991494, 2.199114857512855, 3.365992128846207;
	Vec5D expectedresult;
	expectedresult << 803.6932281674599, - 82032.63942133621, 98869.89438491964, 4.984821941957457E9, -1.6712500687549996E9;

	Vec3D Testfield(0.003, -0.012, 0.005);
	prepareCalculations(state);
	const auto result = getDeterministicVector(state, Testfield);
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, StochasticMatrixNotRotated)
{
	Vec5D state;
	state << 2.827433388230814, 0.6283185307179586, 0.2617993877991494, 2.199114857512855, 3.365992128846207;

	Matrix5x6 expectedresult;
	expectedresult << 88.62988684685467, 330.7712407815423, 0.0 , -0.4622104829936629, 0.12384892565135111, 0.5822961578444831,
		194.4224572138732, -52.09534040093183, 0.0, 0.07279657201013119, 0.271680505358656, -0.1808923837050222,
		-71.70308466863406, -267.5995550427554, 201.28093889032147, 0.46008003610648557, -0.4776169733816674, -0.4710874874554237,
		44.78922250947815, -196.23440551433833, 0., 2345.9911434075575, -8062.898544847359, -678.5680312457905,
		-142.57264106401647, -32.541274949430864,	201.28093889032144, -5707.0430997584335, -2366.0172499853024, 8382.77698474619;

	prepareCalculations(state);
	const auto result = getStochasticMatrix(state);
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\n" << result << "\n";
		std::cout << "Expected:\n" << expectedresult << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DriftNotRotated)
{
	Vec5D state;
	state << 2.827433388230814, 0.6283185307179586, 0.2617993877991494, 2.199114857512855, 3.365992128846207;

	Vec5D expectedresult;
	expectedresult << -2579.7575258504394, 25012.408369345783, -270.7396458915823,-2.5797700086952366E7, 0.0;

	prepareCalculations(state);
	const auto result = getDrift(state);
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

	///< State 1 END.

TEST_F(NeelSpherical_BrownEuler_ProblemTest, NeelRotation)
{
	Vec5D state;
	state << 2.0943951023931957, 1.8479956785822313, 0.3141592653589793, 0.4487989505128276, 4.45058959258554;
	Vec5D expectedresult(state);
	expectedresult(3) = 1.4582615774534704;
	expectedresult(4) = -0.43539362725516695;
	Vec5D copy(state);

	prepareCalculations(state);
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);

	EXPECT_TRUE(state.isApprox(expectedresult));
	if (!state.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << state.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}

	finishCalculations(state);
	for (int i = 3; i < 5; i++)
	{
		if (state(i) < 0)
		{
			state(i) = math::constants::two_pi<Precision> +state(i);
		}
	}
	EXPECT_TRUE(state.isApprox(copy));
	if (!state.isApprox(copy))
	{
		std::cout << "Result:\t" << state.transpose() << "\n";
		std::cout << "Expected:\t" << copy.transpose() << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicNeelRotatedWithoutField)
{
	Vec5D state;
	state << 2.0943951023931957, 1.8479956785822313, 0.3141592653589793, 0.4487989505128276, 4.45058959258554;
	Vec5D expectedresult;
	expectedresult << 5.755593605036077E-11, -1.7987156425075806E-11, 1.5750929555939314E-11, -1.5748633960324807E9, 2.964206707956691E9;

	prepareCalculations(state);
	const auto result = getDeterministicVector(state, Vec3D::Zero());
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicNeelRotated)
{
	Vec5D state;
	state << 2.0943951023931957, 1.8479956785822313, 0.3141592653589793, 0.4487989505128276, 4.45058959258554;
	Vec5D expectedresult;
	expectedresult << 59963.27526820711, 72423.11318547415, 42320.775678309335, -3.0326194823699446E9, 2.0985706842830565E9;

	Vec3D Testfield(0.003, -0.012, 0.005);
	prepareCalculations(state);
	const auto result = getDeterministicVector(state, Testfield);
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, StochasticMatrixNeelRotated)
{
	Vec5D state;
	state << 2.0943951023931957, 1.8479956785822313, 0.3141592653589793, 0.4487989505128276, 4.45058959258554;

	Matrix5x6 expectedresult;
	expectedresult << 64.66788570501728, 199.02728721826261,0.,  0.42630043085301933, - 0.13851340649672014, -0.0112972619598378,
			 191.42954853764684, -62.19923076085461, 0.,	-0.13322574629176598, - 0.4100266860900552, -0.2073355597062891,
			 17.697206963674738, 54.466402526084984, 201.28093889032147, 0.3172082631968774, - 0.09164202793426318, -0.0030916424875568177,
			 0., 182.50225122082819, 84.89372568040024, 833.4507829307851,	7560.97188975485, 3620.9840979002092,
			-201.28093889032144,9.59402818847148, - 20.624984103817862, - 8382.77698474619,	1164.9086788828743, - 502.9598818592171;

	prepareCalculations(state);
	const auto result = getStochasticMatrix(state);
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\n" << result << "\n";
		std::cout << "Expected:\n" << expectedresult << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DriftNeelRotated)
{
	Vec5D state;
	state << 2.0943951023931957, 1.8479956785822313, 0.3141592653589793, 0.4487989505128276, 4.45058959258554;

	Vec5D expectedresult;
	expectedresult << -1807.0317335269065, -5672.599682873375, 3119.529701731519, 4.0127799544901657E6, 0.;

	prepareCalculations(state);
	const auto result = getDrift(state);
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	const auto tol = std::numeric_limits<Vec5D::Scalar>::epsilon()*expectedresult.norm();
	EXPECT_TRUE(result.isApprox(expectedresult,tol));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
		std::cout << "Difference:\t" << (result - expectedresult).norm() << "\n";
	}
}

///< State 2 END.

TEST_F(NeelSpherical_BrownEuler_ProblemTest, BrownRot)
{
	Vec5D state;
	state << 3.4033920413889427, 0.3141592653589793, 3.6249146002959156, 0.9424777960769379, 0.7853981633974483;
	Vec5D expectedresult;
	expectedresult << -0.8460271871395861, 1.4266907269075624, 1.8509328124765516, 0.9424777960769379, 0.7853981633974483;
	Vec5D copy(state);

	prepareCalculations(state);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);

	EXPECT_TRUE(state.isApprox(expectedresult));
	if (!state.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << state.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}

	finishCalculations(state);
	for (int i = 0; i < 3; i++)
	{
		if (state(i) < 0)
		{
			state(i) = math::constants::two_pi<Precision> +state(i);
		}
	}
	EXPECT_TRUE(state.isApprox(copy));
	if (!state.isApprox(copy))
	{
		std::cout << "Result:\t" << state.transpose() << "\n";
		std::cout << "Expected:\t" << copy.transpose() << "\n";
	}

}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicBrownRotWithoutField)
{
	Vec5D state;
	state << 3.4033920413889427, 0.3141592653589793, 3.6249146002959156, 0.9424777960769379, 0.7853981633974483;
	Vec5D expectedresult;
	expectedresult << -8.131106423660079E-12, -2.7969296520179497E-11, 3.027151716656241E-13, 9.973508193811858E8, 4.353150097023238E9;

	prepareCalculations(state);
	const auto result = getDeterministicVector(state, Vec3D::Zero());
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicBrownRot)
{
	Vec5D state;
	state << 3.4033920413889427, 0.3141592653589793, 3.6249146002959156, 0.9424777960769379, 0.7853981633974483;
	Vec5D expectedresult;
	expectedresult << -79839.13669056905, 34085.967216155375, -87148.03753964901, -9.866778193999017E8, 5.801558978068349E9;

	Vec3D Testfield(0.003, -0.012, 0.005);
	prepareCalculations(state);
	const auto result = getDeterministicVector(state, Testfield);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, StochasticMatrixBrownRot)
{
	Vec5D state;
	state << 3.4033920413889427, 0.3141592653589793, 3.6249146002959156, 0.9424777960769379, 0.7853981633974483;

	Matrix5x6 expectedresult;
	expectedresult << 0., -56.23441002394785, 195.46054568411864, -0.3444049603631463,	0.2658245681965882, 0.07647828726808065,
			 0., -193.43454711408955, - 55.651525974912865, -0.19461445715725811, - 0.07568557025145159, 0.2630692285286265,
			 -201.28093889032147, 8.075675043306669, - 28.069572527926113, 0.04945908638520869, 0.24308999199155892, -0.2847230948999048,
			-142.32711681294137, 142.3271168129414, 0., - 5578.908774331065, 6276.128127846038, - 678.5680312457906,	
			 -103.40670325298875, - 103.40670325298872,	201.28093889032144, - 5039.694052481117, - 3573.4944280234777,	8382.77698474619;
	prepareCalculations(state);
	const auto result = getStochasticMatrix(state);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\n" << result << "\n";
		std::cout << "Expected:\n" << expectedresult << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DriftBrownRot)
{
	Vec5D state;
	state << 3.4033920413889427, 0.3141592653589793, 3.6249146002959156, 0.9424777960769379, 0.7853981633974483;

	Vec5D expectedresult;
	expectedresult << 1648.439817263778, 82.36681705984392, -2531.398389611761, 2.5797700086952373E7, 0.;

	prepareCalculations(state);
	const auto result = getDrift(state);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

///< State 3 END.

TEST_F(NeelSpherical_BrownEuler_ProblemTest, BothRotated)
{
	Vec5D state;
	state << 2.8797932657906435, 0.39269908169872414, 4.2839899821679, 2.9919930034188504, 1.308996938995747;
	Vec5D expectedresult;
	expectedresult << -0.7208862155822325, 1.2152515793441003, 1.7411981677912913, 1.6093808769980391, 2.9970185345243556;
	Vec5D copy(state);

	prepareCalculations(state);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);

	EXPECT_TRUE(state.isApprox(expectedresult));
	if (!state.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << state.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}

	finishCalculations(state);
	for (int i = 0; i < 3; i++)
	{
		if (state(i) < 0)
		{
			state(i) = math::constants::two_pi<Precision> +state(i);
		}
	}
	EXPECT_TRUE(state.isApprox(copy));
	if (!state.isApprox(copy))
	{
		std::cout << "Result:\t" << state.transpose() << "\n";
		std::cout << "Expected:\t" << copy.transpose() << "\n";
	}

}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicBothRotatedWithoutField)
{
	Vec5D state;
	state << 2.8797932657906435, 0.39269908169872414, 4.2839899821679, 2.9919930034188504, 1.308996938995747;
	Vec5D expectedresult;
	expectedresult << -2.9516894829254684E-13, 4.761332677310178E-14, -2.9001081822867654E-11, 3.0364396530135117E9, 1.1391767841567204E9;

	prepareCalculations(state);
	const auto result = getDeterministicVector(state, Vec3D::Zero());
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicBothRotated)
{
	Vec5D state;
	state << 2.8797932657906435, 0.39269908169872414, 4.2839899821679, 2.9919930034188504, 1.308996938995747;
	Vec5D expectedresult;
	expectedresult << -3672.090574480425, 32480.94612974337, 112150.8808749778, 5.0369447083583965E9, 7.66120981669770E8;

	Vec3D Testfield(0.003, -0.012, 0.005);
	prepareCalculations(state);
	const auto result = getDeterministicVector(state, Testfield);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
		std::cout << "Difference:\t" << (result - expectedresult).norm() << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, StochasticMatrixBothRotated)
{
	Vec5D state;
	state << 2.8797932657906435, 0.39269908169872414, 4.2839899821679, 2.9919930034188504, 1.308996938995747;

	Matrix5x6 expectedresult;
	expectedresult << 0., -36.410092699823466, 211.59981428686652, 0.013172224108651405,0.019405030137040695, 0.0033390338668949056,
			 0., -198.36572725853583, - 34.132896299046855, 0.4779998740955621, - 0.0031302006742290394, 0.018191381351542135,
			 -201.28093889032147, 12.674393107689653, - 73.65812687971747, -0.004585265624892078, - 0.47992579279109926, -0.07005121917830118,
			 0., - 199.18104584786028, - 28.998747137107603,	838.1319349323676, - 8299.983846724665, - 1175.6976508581272,
			 -201.28093889032147, 1.1194592080531343, - 7.689127216076023, - 8382.776984746191, - 784.001709588679, - 441.1606747387089;

	prepareCalculations(state);
	const auto result = getStochasticMatrix(state);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\n" << result << "\n";
		std::cout << "Expected:\n" << expectedresult << "\n";
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DriftBothRotated)
{
	Vec5D state;
	state << 2.8797932657906435, 0.39269908169872414, 4.2839899821679, 2.9919930034188504, 1.308996938995747;

	Vec5D expectedresult;
	expectedresult << -4274.295693461015, 7625.44916005849, 1333.1545877079254, -1.3707207486121655E6, 0.;

	prepareCalculations(state);
	const auto result = getDrift(state);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult));
	if (!result.isApprox(expectedresult))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

///< State 4 END.

///-------------------------------------------------------------------------------------------------
// End of Test_NeelSpherical_BrownEuler_Problem.cpp
///-------------------------------------------------------------------------------------------------
