///-------------------------------------------------------------------------------------------------
// file:	Test_Neel_Spherical_Problem.cpp
//
// summary:	Implements the test neel spherical problem class
///-------------------------------------------------------------------------------------------------

#include "Test_Neel_Spherical_Problem.h"

TEST_F(NeelSphericalProblemTest, DeterministicVectorTestWithoutField)
{
	//Some Test Vectors
	Vec3D testy1, testy2, testy3, testy4, testy5;
	Vec3D testres1, testres2, testres3, testres4, testres5;

	//Testcalculations have been performed in Mathematic testres1 is the result obtained by it!

	testy1 << 0.658, -0.456, 0.236;
	testy1.normalize();
	testres1 << -2.177876311321179E8, 1.3793163601388674E9, 3.2723411928315988E9;
	const auto resvec1 = mProblem.getDeterministicVector(testy1, Vec3D::Zero());

	EXPECT_TRUE(resvec1.isApprox(testres1));

	testy2 << -0.757, -0.256, 0.239;
	testy2.normalize();
	testres2 << 1.167966138967112E8, -2.0827622519068696E9, -1.8609711287378592E9;
	const auto resvec2 = mProblem.getDeterministicVector(testy2, Vec3D::Zero());
	EXPECT_TRUE(resvec2.isApprox(testres2));

	testy3 << 0.620, -0.056, -0.132;
	testy3.normalize();
	testres3 << -3.610582440770419E7, -1.5361155759934711E9, 4.820974327489226E8;
	const auto resvec3 = mProblem.getDeterministicVector(testy3, Vec3D::Zero());
	EXPECT_TRUE(resvec3.isApprox(testres3));

	testy4 << -0.158, -0.756, -0.138;
	testy4.normalize();
	testres4 << 1.4103529993942258E8, 2.3003269806987792E8, -1.4216543270380902E9;
	const auto resvec4 = mProblem.getDeterministicVector(testy4, Vec3D::Zero());
	EXPECT_TRUE(resvec4.isApprox(testres4));

	testy5 << -0.458, 0.256, 0.832;
	testy5.normalize();
	testres5 << 2.6618598882513666E8, -2.8336136367421064E9, 1.0184113868844856E9;
	const auto resvec5 = mProblem.getDeterministicVector(testy5, Vec3D::Zero());
	EXPECT_TRUE(resvec5.isApprox(testres5, 1E-6));
};

//TEST_F(NeelProblemTest, DeterministicVectorTestWithField)
//{
//	//Some Test Vectors
//	Vec3D testy1, testy2, testy3, testy4, testy5;
//	Vec3D testres1, testres2, testres3, testres4, testres5;
//
//	//Testcalculations have been performed in Mathematic testres1 is the result obtained by it!
//
//	testy1 << 0.658, -0.456, 0.236;
//	testy1.normalize();
//	testres1 << -2.177876311321182E8, 1.3793163601388752E9, 3.2723411928316007E9;
//	const auto resvec1 = mProblem.getDeterministicVector(testy1, testy1);
//
//	EXPECT_TRUE(resvec1.isApprox(testres1));
//
//	testy2 << -0.757, -0.256, 0.239;
//	testy2.normalize();
//	testres2 << -5.685022932267662E9, -9.818453167861041E10, -1.2317490572992004E11;
//	const auto resvec2 = mProblem.getDeterministicVector(testy2, testy1);
//	EXPECT_TRUE(resvec2.isApprox(testres2));
//
//	testy3 << 0.620, -0.056, -0.132;
//	testy3.normalize();
//	testres3 << 2.492542914922212E10, 6.660792427015209E10, 8.881607813173636E10;
//	const auto resvec3 = mProblem.getDeterministicVector(testy3, testy1);
//	EXPECT_TRUE(resvec3.isApprox(testres3));
//
//	testy4 << -0.158, -0.756, -0.138;
//	testy4.normalize();
//	testres4 << 7.922047948824426E10, 1.0289785795344185E10, -1.4707183927842606E11;
//	const auto resvec4 = mProblem.getDeterministicVector(testy4, testy1);
//	EXPECT_TRUE(resvec4.isApprox(testres4));
//
//	testy5 << -0.458, 0.256, 0.832;
//	testy5.normalize();
//	testres5 << -8.154135521381754E10, -1.5027557268901328E11, 1.3516898082439747E9;
//	const auto resvec5 = mProblem.getDeterministicVector(testy5, testy1);
//	EXPECT_TRUE(resvec5.isApprox(testres5, 1E-6));
//};
//
//TEST_F(NeelProblemTest, StochasticMatrixTest)
//{
//	Vec3D TestInput;
//	TestInput << 0.658, -0.456, 0.236;
//	TestInput.normalize();
//
//	Matrix3x3 Expected;
//	Expected << 317.25250533201563, 2731.4085608501623, 4393.093878132237,
//		-2009.2581411753074, 588.0492855781649, 6738.314962360152,
//		-4766.838393577995, -6479.306605151541, 771.2536060390577;
//	const auto calcVal = mProblem.getStochasticMatrix(TestInput);
//
//	EXPECT_TRUE(calcVal.isApprox(Expected));
//};
//
//TEST_F(NeelProblemTest, DriftTest)
//{
//	Vec3D TestInput;
//	TestInput << 0.658, -0.456, 0.236;
//	TestInput.normalize();
//
//	Vec3D Expected;
//	Expected << -5.5954188803957045E7, 3.877676306170884E7, -2.0068675619656324E7;
//
//	auto calcVal = mProblem.getDrift(TestInput);
//
//	EXPECT_TRUE(calcVal.isApprox(Expected));
//
//	Expected.normalize();
//	calcVal.normalize();
//
//	EXPECT_TRUE(calcVal.isApprox(-TestInput)); //Direction Test
//	EXPECT_TRUE(Expected.isApprox(-TestInput)); //Direction Test
//
//
//
//};
//
//TEST_F(NeelProblemTest, AllParts)
//{
//	Vec3D Input1, Input2, Input4;
//	Input1 << 0.658, -0.456, 0.236;
//	Input1.normalize();
//	Input2 << Input1;
//	Input4 << Input1;
//
//	Precision Input3 = 1E-6; //dt
//
//	Vec3D ExpectedDet;
//	Matrix3x3 ExpectedSto, ExpectedJacDet, ExpectedJacSto;
//	ExpectedDet << -2.177876311321182E8, 1.3793163601388752E9, 3.2723411928316007E9;
//	ExpectedSto << 317.25250533201563, 2731.4085608501623, 4393.093878132237,
//		-2009.2581411753074, 588.0492855781649, 6738.314962360152,
//		-4766.838393577995, -6479.306605151541, 771.2536060390577;
//
//	ExpectedJacDet << 0, (-4.927337499067257E10), (-9.520618218536737E10),
//		5.133733924947046E10, 0, (-1.3162623839649808E11),
//		9.919418092270566E10, 1.3162623839649808E11, 0;
//
//	ExpectedJacSto << 0, -2370.333351012735, -4579.966135855116,
//		2370.333351012735, 0, -6608.810783755846,
//		4579.966135855116, 6608.810783755846, 0;
//
//
//	const auto calcValues = mProblem.getAllProblemParts(Input1, Input2, Input3, Input4);
//	const auto& calcDet = std::get<0>(calcValues);
//	const auto& calcDetJac = std::get<1>(calcValues);
//	const auto& calcStoMat = std::get<2>(calcValues);
//	const auto& calcStoJac = std::get<3>(calcValues);
//
//	EXPECT_TRUE(calcDet.isApprox(ExpectedDet));
//	EXPECT_TRUE(calcStoMat.isApprox(ExpectedSto));
//	EXPECT_TRUE(calcDetJac.isApprox(ExpectedJacDet));
//	EXPECT_TRUE(calcStoJac.isApprox(ExpectedJacSto, std::numeric_limits<Precision>::epsilon() * 100));
//
//	//const Matrix3x3 tmp = (calcStoJac - ExpectedJacSto);
//	//EXPECT_TRUE(tmp.isMuchSmallerThan(std::numeric_limits<Precision>::epsilon() * 100));
//	//std::cout << "CalcStoJac:\n" << calcStoJac << "\nExpectedStoJac:\n" << ExpectedJacSto << std::endl;
//	//std::cout << "Diff:\n" << (calcStoJac - ExpectedJacSto) << std::endl;
//};
//
//TEST_F(NeelProblemTest, afterStepCheck)
//{
//	Vec3D TestVector{ Vec3D::Random() };
//	mProblem.afterStepCheck(TestVector);
//	EXPECT_EQ(TestVector.norm(), 1.0);
//};