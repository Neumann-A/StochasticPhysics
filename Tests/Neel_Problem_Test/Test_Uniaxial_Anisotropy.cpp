///-------------------------------------------------------------------------------------------------
// file:	Test_Uniaxial_Anisotropy.cpp
//
// summary:	Implements the test uniaxial anisotropy class
///-------------------------------------------------------------------------------------------------
#include "Test_Uniaxial_Anisotropy.h"

#include <iostream>

TEST_F(UniaxialAnisotropyTest, AnisotropyField)
{
    const auto& K = (Properties.Anisotropy.getEmumVariantType<UniaxialAnisotropyTest::aniso>()).K_uniaxial;
    const auto& MS = Properties.getSaturationMagnetisation();
    const auto prefactor{ 2 * K / MS };
    
    Vec3D TestVector;
    TestVector << 0.75, 0.13, -0.5;
    TestVector.normalize();
    Vec3D EasyAxis;
    EasyAxis << -0.25, 0.43, -0.16;
    EasyAxis.normalize();

    auto calcexpected = [](auto prefac, auto testvec, auto ea) { return (-prefac*(ea.dot(testvec))*ea).eval(); };

    const auto& expectedval = calcexpected(prefactor,TestVector,EasyAxis);
    const Anisotropy testanisotropy{ Properties };
    const auto testval = testanisotropy.getAnisotropyField(TestVector, Vec3D::Zero(), Vec3D::Zero(),EasyAxis);

    //std::cout << "Expected: " << expectedval << "\n" << std::endl;
    //std::cout << "Calculated: " << testval << "\n" << std::endl;
    //system("pause");

    //TODO: More Tests!
    EXPECT_TRUE(testval.isApprox(expectedval));
}

TEST_F(UniaxialAnisotropyTest, AnisotropyFieldJacobi)
{
    Vec3D TestVector;
    TestVector << 0.75, 0.13, -0.5;
    TestVector.normalize();
    Vec3D EasyAxis;
    EasyAxis << -0.25, 0.43, -0.16;
    EasyAxis.normalize();
        
    const Anisotropy testanisotropy{ Properties };
    const auto testval = testanisotropy.getJacobiAnisotropyField(TestVector, Vec3D::Zero(), Vec3D::Zero(), EasyAxis);
    std::decay_t<decltype(testval)> expectedval;

    expectedval << -0.00958973782055732, 0.01649434905135859, -0.006137432205156684, 
    0.01649434905135859, -0.028370280368336775, 0.010556383392869496, 
    -0.006137432205156684, 0.010556383392869496, -0.003927956611300278;

    //std::cout << "Expected: " << expectedval << "\n" << std::endl;
    //std::cout << "Calculated: " << testval << "\n" << std::endl;
    //system("pause");

    //TODO: More Tests!
    EXPECT_TRUE(testval.isApprox(expectedval));
}


TEST_F(UniaxialAnisotropyTest, PositiveKmeansUniaxial)
{
    Vec3D EasyAxis(1, 0, 0);
    Vec3D AxisOne(1, 0, 0);
    Vec3D AxisTwo(0, 1, 0);
    Vec3D AxisThree(0, 0, 1);

    Anisotropy anisotropy{ Properties };

    const Precision val1 = -anisotropy.getAnisotropyField(AxisOne,Vec3D::Zero(), Vec3D::Zero(), EasyAxis).dot(AxisOne);
    const Precision val2 = -anisotropy.getAnisotropyField(AxisTwo, Vec3D::Zero(), Vec3D::Zero(), EasyAxis).dot(AxisTwo);
    const Precision val3 = -anisotropy.getAnisotropyField(AxisThree, Vec3D::Zero(), Vec3D::Zero(), EasyAxis).dot(AxisThree);

    //std::cout << "Axis: " << val1 << "Perp Axis:" << val2 << ',' << val3 << '\n';

    EXPECT_TRUE((val1 < val2 && val1 < val3));
    EXPECT_DOUBLE_EQ(val2, val3);
}

TEST_F(UniaxialAnisotropyTest, NegativeKmeansUniaxial)
{
    Vec3D EasyAxis(1, 0, 0);
    Vec3D AxisOne(1, 0, 0);
    Vec3D AxisTwo(0, 1, 0);
    Vec3D AxisThree(0, 0, 1);

    Anisotropy anisotropy{ Properties };

    const Precision val1 = -anisotropy.getAnisotropyField(AxisOne, Vec3D::Zero(), Vec3D::Zero(), EasyAxis).dot(AxisOne);
    const Precision val2 = -anisotropy.getAnisotropyField(AxisTwo, Vec3D::Zero(), Vec3D::Zero(), EasyAxis).dot(AxisTwo);
    const Precision val3 = -anisotropy.getAnisotropyField(AxisThree, Vec3D::Zero(), Vec3D::Zero(), EasyAxis).dot(AxisThree);
    
    //std::cout << "Axis: " << val1 << "Perp Axis:" << val2 << ',' << val3 << '\n';

    EXPECT_TRUE((val1 > val2 && val1 > val3));
    EXPECT_DOUBLE_EQ(val2, val3);
}
