///-------------------------------------------------------------------------------------------------
// file:	Test_BrownAndNeel_Relaxation.cpp
//
// summary:	Implements the test brown and neel relaxation class
///-------------------------------------------------------------------------------------------------

#include "Test_BrownAndNeel_Relaxation.h"

#include <MyCEL/math/ApproxJacobian.h>

using namespace Problems;

TEST_F(BrownAndNeelRelaxation_Test, IsStratonovich)
{
	EXPECT_FALSE(Traits::IsIto::value);
	if (Traits::IsIto::value)
	{
		std::cout << "Warning: currently the implementation is set to intepret the noise integral the Ito way!";
	};
}
TEST_F(BrownAndNeelRelaxation_Test, DeterministicVectorTestWithoutField)
{
	{ //Subtest
		Vec6D Input;
		Input << 0.7883796498202967, -0.5463542861976525, 0.28276230601457447,
			-0.20473715419227356, - 0.979628408666828, - 0.17882105872489718;
		Vec6D Expected;
		Expected << 5.143416383627018E-13, -2.057366553450807E-12, -5.409304857695873E-12,
			1.091983124889862E9, 1.3752088895393547E8, -2.0036168534911115E9;
		const auto Output = getDeterministicVector(Input, Vec3D::Zero());

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{ //Subtest
		Vec6D Input;
		Input << -0.9075760046194221, - 0.3069213437022088, 0.286539848221984, 
			- 0.20138519309283084, - 0.9635899112543046, - 0.17589339649880165;
		Vec6D Expected;
		Expected << -2.0848517904377075E-12, 0., - 6.6034845412608565E-12, 
			7.748536840588413E8, -6.461809647979987E8, 2.6527965746810875E9;
		const auto Output = getDeterministicVector(Input, Vec3D::Zero());

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{ //Subtest
		Vec6D Input;
		Input << 0.9742841236190559, - 0.08799985632688247, - 0.20742823277050867, 
			0.9742841236190559, - 0.08799985632688247, - 0.20742823277050867;
		Vec6D Expected;
		Expected << 0., - 2.731998744136045E-13, 1.159029770239534E-13, 
			0., 1.9738607975926274E-7, -9.869303987963137E-8;
		const auto Output = getDeterministicVector(Input, Vec3D::Zero());

		EXPECT_TRUE(((Output-Expected).norm() < 1E-6));
		if (!((Output - Expected).norm() < 1E-6))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{ //Subtest
		Vec6D Input;
		Input << -0.20138519309283084, - 0.9635899112543046, - 0.17589339649880165, 
			- 0.9075760046194221, - 0.3069213437022088, 0.286539848221984;
		Vec6D Expected;
		Expected << -2.8044157406964357E-11, 5.861080516270328E-12, 0., 
			-9.730369483916581E8, 4.191269719276371E8, -2.633022866606737E9;
		const auto Output = getDeterministicVector(Input, Vec3D::Zero());

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{ //Subtest
		Vec6D Input;
		Input << -0.20473715419227356, - 0.979628408666828, - 0.17882105872489718, 
			0.7883796498202967, - 0.5463542861976525, 0.28276230601457447;
		Vec6D Expected;
		Expected << 4.27664086746588E-11, -1.1540142023320628E-11, 1.4255469558219601E-11,
			-9.925439053485414E8, -3.855150438676625E8, 2.0224535157444327E9;
		const auto Output = getDeterministicVector(Input, Vec3D::Zero());

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
};
TEST_F(BrownAndNeelRelaxation_Test, DeterministicVectorTestWithField)
{
	Vec3D Testfield(-0.123, 0.523, -0.872);
	{ //Subtest
		Vec6D Input;
		Input << 0.7883796498202967, -0.5463542861976525, 0.28276230601457447,
			-0.20473715419227356, -0.979628408666828, -0.17882105872489718;
		Vec6D Expected;
		Expected << -1.677065130434849E6, -4.45039843480503E6, -3.923189959512555E6,
			-1.674611095382276E11, 3.118240777297087E10, 2.090547123676795E10;
		const auto Output = getDeterministicVector(Input, Testfield);

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{ //Subtest
		Vec6D Input;
		Input << -0.9075760046194221, -0.3069213437022088, 0.286539848221984,
			-0.20138519309283084, -0.9635899112543046, -0.17589339649880165;
		Vec6D Expected;
		Expected << -1.1222644097219862E6, -636294.3406569965, - 4.236173679362907E6, 
			- 1.649639469844982E11, 2.9829768879558537E10, 2.5456509786988834E10;
		const auto Output = getDeterministicVector(Input, Testfield);

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{ //Subtest
		Vec6D Input;
		Input << 0.9742841236190559, -0.08799985632688247, -0.20742823277050867,
			0.9742841236190559, -0.08799985632688247, -0.20742823277050867;
		Vec6D Expected;
		Expected << -1.3690323605623604E6, 5.215512889556172E6, -8.642945343665222E6,
			-3.467578751297353E10, - 1.4334892489621368E11, -1.0205642775648203E11;
		const auto Output = getDeterministicVector(Input, Testfield);

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{ //Subtest
		Vec6D Input;
		Input << -0.20138519309283084, -0.9635899112543046, -0.17589339649880165,
			-0.9075760046194221, -0.3069213437022088, 0.286539848221984;
		Vec6D Expected;
		Expected << -3.465136854399259E6, 1.2325371496993422E6, -2.7848294360697074E6,
			-2.8368303369304657E10, 1.5198928128911234E11, 7.294749104372025E10;
		const auto Output = getDeterministicVector(Input, Testfield);

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{ //Subtest
		Vec6D Input;
		Input << -0.20473715419227356, -0.979628408666828, -0.17882105872489718,
			0.7883796498202967, -0.5463542861976525, 0.28276230601457447;
		Vec6D Expected;
		Expected << 2.202064800599363E6, -118469.62997804009, - 1.872197088632616E6, 
			-5.173693082116708E10, -1.1099684380261235E11, -7.021889954942075E10;
		const auto Output = getDeterministicVector(Input, Testfield);

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
};
TEST_F(BrownAndNeelRelaxation_Test, StochasticMatrixTest)
{
	Vec6D Input;
	Input << 0.7883796498202967, -0.5463542861976525, 0.28276230601457447,
		-0.20473715419227356, -0.979628408666828, -0.17882105872489718;
	{
		StochasticMatrixType Expected(StochasticMatrixType::Zero());
		Expected << 0., 56.914662437405944, 109.97070369261489, 0.23191747939825336, -0.0535262760939153, 0.027702195522289492,
			-56.914662437405944, 0., 158.6857961178522, 0.3695666637570228, -0.10143306584019601, 0.13254974566361302,
			-109.97070369261489, -158.6857961178522, 0., 0.06746058147945655, -0.04675079810734375, 0.17887559054636046,
			0., -35.99327059350861, 197.18052586009065, 803.5978161950169, -1667.2433132134333, 8181.298510091952,
			35.99327059350861, 0., -41.20968662155334, 1330.7907977205973, 33.825512355897445, -1863.197698825965,
			-197.18052586009065, 41.20968662155334, 0., -8242.714445459693, 1569.3341093448807, 811.9353308110741;

		const auto Output = getStochasticMatrix(Input);

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\n" << Output << '\n';
			std::cout << "Expected:\n" << Expected << '\n';
		}
	}

	//Simplified Version
	{
		StochasticMatrixType Expected(StochasticMatrixType::Zero());
		Expected << 0., 56.914662437405944, 109.97070369261489, 0, 0, 0,
			-56.914662437405944, 0., 158.6857961178522, 0, 0, 0,
			-109.97070369261489, -158.6857961178522, 0., 0, 0, 0,
			0., 0., 0., 803.1393587706513, - 1667.147339113849, 8181.316029173622,
			0., 0., 0., 1330.8867718201814, 33.80621469609797, - 1863.1138733465814,
			0., 0., 0., -8242.696926378021, 1569.4179348242642, 811.4721167840903;

		const auto Output = getStochasticMatrixSimplified(Input);

		EXPECT_TRUE(Output.isApprox(Expected));
		if (!Output.isApprox(Expected))
		{
			std::cout << "Result:\n" << Output << '\n';
			std::cout << "Expected:\n" << Expected << '\n';
		}
	}
};

TEST_F(BrownAndNeelRelaxation_Test, DriftTest)
{
	Vec6D Input;
	Input << 0.7883796498202967, -0.5463542861976525, 0.28276230601457447,
		-0.20473715419227356, -0.979628408666828, -0.17882105872489718;
	{
		Vec6D Expected;
		Expected << -33443.36466540489, 21801.69451008975, -7909.103942511765,
			1.45394040869399E7, 6.95682879096618E7, 1.26989731898589E7;

		auto Output = getDrift(Input);

		EXPECT_TRUE(Output.isApprox(Expected, 1E-6));
		if (!Output.isApprox(Expected, 1E-6))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	{
		Vec6D Expected;
		Expected << -31940.42603115941, 22135.006489678864, - 11455.836692026778, 
			1.4530945063676618E7, 6.952781308949065E7, 1.269158492903401E7;

		auto Output = getStratonovichtoItoSimplified(Input);

		EXPECT_TRUE(Output.isApprox(Expected, 1E-6));
		if (!Output.isApprox(Expected, 1E-6))
		{
			std::cout << "Result:\t" << Output.transpose() << "\n";
			std::cout << "Expected:\t" << Expected.transpose() << "\n";
		}
	}
	////Testing direction:
	//Expected.normalize();
	//Output.normalize();
	//EXPECT_TRUE(Output.isApprox(-Input,1E-6)); //Direction Test
	//EXPECT_TRUE(Expected.isApprox(-Input,1E-6)); //Direction Test
};
