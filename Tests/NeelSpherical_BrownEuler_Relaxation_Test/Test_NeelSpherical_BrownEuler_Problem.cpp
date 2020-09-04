///-------------------------------------------------------------------------------------------------
// file:	Test_NeelSpherical_BrownEuler_Problem.cpp
//
// summary:	Implements the tests to run
///-------------------------------------------------------------------------------------------------

#include "Test_NeelSpherical_BrownEuler_Problem.h"

#include <limits>

#include <MyCEL/math/ApproxJacobian.h>

#include <Eigen/Geometry>

using namespace Problems;

TEST_F(NeelSpherical_BrownEuler_ProblemTest, EmptyTest)
{

}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, IsStratonovich)
{
	EXPECT_FALSE(Traits::IsIto::value);
	if (Traits::IsIto::value)
	{
		std::cout << "Warning: currently the implementation is set to intepret the noise integral the Ito way!";
	};
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, DeterministicNotRotatedWithoutField)
{
	Vec5D state;
	state << 2.827433388230814, 0.6283185307179586, 0.2617993877991494, 2.199114857512855, 3.365992128846207;
	Vec5D expectedresult;
	expectedresult << -2.391361587625311E-11, 3.766312803818739E-12, 4.8450352097577024E-11, -1.883995659756739E9, - 3.1170454111541033E9;
	
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
	expectedresult << -70477.14978745366, - 70806.15016492385, 156537.30775792996, 2.0307296989120203E8, -2.0114601508058534E9;

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
	expectedresult << 88.62988684685467, - 330.7712407815423, 1.810217639970441E-14,0.4622104829936629, 0.12384892565135108, - 0.6581595176428002,
			 194.4224572138732,	52.09534040093183, - 2.1826249350538855E-16, -0.07279657201013119, 0.271680505358656, 0.014475147384107134,
			 -71.70308466863406, 267.5995550427554,	201.28093889032147, - 0.2877922353337664, - 0.4776169733816674,	0.5324622347826434,
			 44.78922250947815, - 196.23440551433833, 0., 2345.9911434075575, -8062.898544847359, -678.5680312457905,
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
	expectedresult << -206.51867134433164, 24638.60788003464, - 2190.70205422572,-2.5797700086952366E7, 0.0;

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
	expectedresult << 7.206046902503714E-13, -1.5525989717363832E-11, - 1.621786569351973E-12, -6.898671440282415E8, - 3.0793140695979565E8;

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
	expectedresult << -4252.520084319385, 92491.58283829824, 24747.289112279974, -2.147623230365705E9, -1.1735674306334295E9;

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
	expectedresult << 64.66788570501728, - 199.02728721826261,0., - 0.42630043085301933, - 0.13851340649672014, - 0.11756601567679807,
			 191.42954853764684, 62.19923076085461, 0.,	0.13322574629176598, - 0.4100266860900552, - 0.17412486368820504,
			 17.697206963674738, - 54.466402526084984, 201.28093889032147, 0.08388296204436849, - 0.09164202793426318, - 0.03217346738097393,
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
	expectedresult << 1517.6138085762407, - 6711.51420770384, 4029.370860106001, 4.0127799544901657E6, 0.;

	prepareCalculations(state);
	const auto result = getDrift(state);
	EXPECT_FALSE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult,1E-6));
	if (!result.isApprox(expectedresult,1E-6))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
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
	expectedresult << 2.131091902800412E-12, 5.996200575286362E-11, - 3.060404774782776E-13, 1.0332495835853837E9, 4.256827487198723E9;

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
	expectedresult << -85936.13695929295, 13113.568195113929, - 86272.4634402995, - 9.50779055195704E8, 5.705236368243834E9;

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
	expectedresult << 0., 56.23441002394785, 195.46054568411864, - 0.18724417603002994,	0.2658245681965882, - 0.07647828726808065,
			 0., 193.43454711408955, - 55.651525974912865,	0.3459855976601613, - 0.07568557025145159, - 0.2630692285286265,
			 -201.28093889032147, - 8.075675043306669, - 28.069572527926113,	0.02688964139085454, 0.24308999199155892, - 0.2627574053663022,
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
	expectedresult << 2930.7325639745077, 4492.84151015584, - 2715.6102848102564, 2.5797700086952373E7, 0.;

	prepareCalculations(state);
	const auto result = getDrift(state);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_FALSE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult,1E-6));
	if (!result.isApprox(expectedresult,1E-6))
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
	expectedresult << 2.632323683235198E-12, 1.4341155517226945E-11, - 3.0020145211081096E-11, 2.1234192736575503E9, - 1.8783517069013772E9;

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
	expectedresult << -15041.781344105477, - 29462.23530665741, 116108.68267227443, 4.123924329002434E9, -2.251407509388327E9;

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
	}
}

TEST_F(NeelSpherical_BrownEuler_ProblemTest, StochasticMatrixBothRotated)
{
	Vec5D state;
	state << 2.8797932657906435, 0.39269908169872414, 4.2839899821679, 2.9919930034188504, 1.308996938995747;

	Matrix5x6 expectedresult;
	expectedresult << 0., 36.410092699823466, 211.59981428686652, - 0.15801334089633365,0.019405030137040695, - 0.0033390338668949056,
			 0., 198.36572725853583, - 34.132896299046855, - 0.4546357381873439, - 0.0031302006742290394, - 0.018191381351542135,
			 -201.28093889032147, - 12.674393107689653, - 73.65812687971747,0.05500461685968784, - 0.47992579279109926, - 0.06772657649321841,
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
	expectedresult << -4065.395681533756, 8763.685888090651, 1260.4669864762586, -1.3707207486121655E6, 0.;

	prepareCalculations(state);
	const auto result = getDrift(state);
	EXPECT_TRUE(BrownCache.isRotated);
	EXPECT_TRUE(NeelCache.isRotated);
	EXPECT_TRUE(result.isApprox(expectedresult,1E-6));
	if (!result.isApprox(expectedresult,1E-6))
	{
		std::cout << "Result:\t" << result.transpose() << "\n";
		std::cout << "Expected:\t" << expectedresult.transpose() << "\n";
	}
}

///< State 4 END.

///-------------------------------------------------------------------------------------------------
// End of Test_NeelSpherical_BrownEuler_Problem.cpp
///-------------------------------------------------------------------------------------------------
