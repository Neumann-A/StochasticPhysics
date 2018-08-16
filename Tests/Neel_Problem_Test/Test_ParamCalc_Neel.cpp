///-------------------------------------------------------------------------------------------------
// file:	Test_ParamCalc_Neel.cpp
//
// summary:	Implements the test parameter calculate neel class
///-------------------------------------------------------------------------------------------------

#include "Test_ParamCalc_Neel.h"

#include <cmath>
#include <limits>

TEST_F(ParamCalcNeelTest, Prefactor1)
{
	const Precision Testvalue = - mProp.getGyromagneticRatio() / (static_cast<Precision>(1.0) + std::pow(mProp.getDampingConstant(), 2));

	const TestParams Params = NeelCalculator<Precision>::calcNeelParams(mProp, 295);

	EXPECT_DOUBLE_EQ(Params.NeelFactor1, Testvalue);
}

TEST_F(ParamCalcNeelTest, NegaticGyromagneticRatioIsElectron)
{
	const TestParams Params = NeelCalculator<Precision>::calcNeelParams(mProp, 295);


	EXPECT_TRUE(Params.NeelFactor1 > 0.0);
}

TEST_F(ParamCalcNeelTest, PositivGyromagneticRatioIsElectron)
{
	const TestParams Params = NeelCalculator<Precision>::calcNeelParams(mProp, 295);


	EXPECT_TRUE(Params.NeelFactor1 < 0.0);
}

TEST_F(ParamCalcNeelTest, Prefactor2)
{
	const Precision Testvalue = std::abs(mProp.getGyromagneticRatio()) * mProp.getDampingConstant() / (static_cast<Precision>(1.0) + std::pow(mProp.getDampingConstant(), 2));

	const TestParams Params = NeelCalculator<Precision>::calcNeelParams(mProp, 295);

	EXPECT_DOUBLE_EQ(Params.NeelFactor2, Testvalue);
}

TEST_F(ParamCalcNeelTest, DriftPrefactor)
{
	const Precision Temp = 295;
	const auto& MsVm = mProp.getSaturationMoment();
	const auto& alpha = mProp.getDampingConstant();
	const auto& gamma = mProp.getGyromagneticRatio();
	
	const Precision Testvalue = std::sqrt(static_cast<Precision>(2.0)*kB*Temp*alpha / (gamma*MsVm));

	const TestParams Params = NeelCalculator<Precision>::calcNeelParams(mProp, Temp);

	EXPECT_DOUBLE_EQ(Params.NoisePrefactor, Testvalue);
}

TEST_F(ParamCalcNeelTest, NoisePrefactor1)
{
	const Precision Temp = 295;
	const auto& MsVm = mProp.getSaturationMoment();
	const auto& alpha = mProp.getDampingConstant();
	const auto& gamma = mProp.getGyromagneticRatio();

	const Precision drift = std::sqrt(static_cast<Precision>(2.0)*kB*Temp*alpha / (gamma*MsVm));
	const Precision prefactor1 = -mProp.getGyromagneticRatio() / (static_cast<Precision>(1.0) + std::pow(mProp.getDampingConstant(), 2));

	const TestParams Params = NeelCalculator<Precision>::calcNeelParams(mProp, Temp);
	
	EXPECT_DOUBLE_EQ(Params.NeelNoise_H_Pre1, prefactor1*drift);
}

TEST_F(ParamCalcNeelTest, NoisePrefactor2)
{
	const Precision Temp = 295;
	const auto& MsVm = mProp.getSaturationMoment();
	const auto& alpha = mProp.getDampingConstant();
	const auto& gamma = mProp.getGyromagneticRatio();

	const Precision drift = std::sqrt(static_cast<Precision>(2.0)*kB*Temp*alpha / (gamma*MsVm));
	const Precision prefactor2 = std::abs(mProp.getGyromagneticRatio()) * mProp.getDampingConstant() / (static_cast<Precision>(1.0) + std::pow(mProp.getDampingConstant(), 2));

	const TestParams Params = NeelCalculator<Precision>::calcNeelParams(mProp, Temp);

	EXPECT_DOUBLE_EQ(Params.NeelNoise_H_Pre2, prefactor2*drift);
}

TEST_F(ParamCalcNeelTest, InfDampingTest)
{
	//General Parameters
	const Precision T = 295;
	//Magnetic Parameters
	const Precision damping = std::numeric_limits<Precision>::infinity();
	const Precision gyro = 1.76E+11;
	const Precision Ms = 477464;
	const Precision rmag = 10E-9;
	const Precision KUni = 10E4;

	const auto Prop = TestProperties{ rmag,Ms,damping,gyro,ParamCalcNeelTest::sAni,std::vector<Precision>{ {KUni,0.0} } };
	const auto Params = NeelCalculator<Precision>::calcNeelParams(Prop, T);

	EXPECT_DOUBLE_EQ(Params.NeelFactor1, 0.0);
	EXPECT_DOUBLE_EQ(Params.NeelFactor2, 0.0);
	EXPECT_DOUBLE_EQ(Params.NoisePrefactor, 0.0);
	EXPECT_DOUBLE_EQ(Params.NeelNoise_H_Pre1, 0.0);
	EXPECT_DOUBLE_EQ(Params.NeelNoise_H_Pre2, 0.0);
}


TEST_F(ParamCalcNeelTest, ItoStratonovichDriftPrefactor)
{
	const Precision Temp = 295;
	const auto& MsVm = mProp.getSaturationMoment();
	const auto& alpha = mProp.getDampingConstant();
	const auto& gamma = mProp.getGyromagneticRatio();

	const Precision drift = std::sqrt(static_cast<Precision>(2.0)*kB*Temp*alpha / (gamma*MsVm));
	const Precision prefactor1 = -mProp.getGyromagneticRatio() / (static_cast<Precision>(1.0) + std::pow(mProp.getDampingConstant(), 2));
	const Precision prefactor2 = std::abs(mProp.getGyromagneticRatio()) * mProp.getDampingConstant() / (static_cast<Precision>(1.0) + std::pow(mProp.getDampingConstant(), 2));

	const TestParams Params = NeelCalculator<Precision>::calcNeelParams(mProp, Temp);

	EXPECT_DOUBLE_EQ(Params.DriftPrefactor, -(std::pow(prefactor1, 2) + std::pow(prefactor2, 2))* std::pow(drift, 2));
}

//TODO: Test behavior for NaN values
//TEST_F(ParamCalcNeelTest, NaNTest)
//{}