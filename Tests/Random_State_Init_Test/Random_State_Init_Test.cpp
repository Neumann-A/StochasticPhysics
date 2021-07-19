///-------------------------------------------------------------------------------------------------
// file:	Random_State_init_Test.cpp
//
// summary:	Implements the tests to run
///-------------------------------------------------------------------------------------------------

#include "Random_State_Init_Test.h"

#include <locale>
#include <Eigen/Geometry>

using namespace Problems;
namespace {
    const auto number = 1'000'000;
    const auto error = 4.0 / std::sqrt(number);
}

TEST_F(BrownAndNeelEulerSphericalFixture, Test_Problem_Random_Orientation_100000)
{
    //std::locale mylocale("");   // get global locale
    //std::cout.imbue(mylocale);  // imbue global locale
    auto StartState = BrownAndNeelEulerSphericalFixture::getStart(this->mInitSet);
    auto Axes = StateInitializer::ConvertEulerAnglesToAxisDirections(StartState.template head<3>());
    double e1 = 0;
    double e2 = 0;
    double e3 = 0;
    EXPECT_TRUE(Axes.xAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.xAxis.norm() < 1.01);
    EXPECT_TRUE(Axes.yAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.yAxis.norm() < 1.01);
    EXPECT_TRUE(Axes.zAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.zAxis.norm() < 1.01);
    for (auto i = number; --i;)
    {
        const auto k = (number-i)+2;
        const auto NextState = BrownAndNeelEulerSphericalFixture::getStart(this->mInitSet);
        //std::cout << (NextState[0] - 1.0*math::constants::pi<Precision>) << '\t' << (NextState[1] - 0.5*math::constants::pi<Precision>) << '\t'<< (NextState[2] - 1.0*math::constants::pi<Precision>) << '\n';
        e1 += (NextState[0] - 1.0*math::constants::pi<Precision> - e1)/k;
        e2 += (NextState[1] - 0.5*math::constants::pi<Precision> - e2)/k;
        e3 += (NextState[2] - 1.0*math::constants::pi<Precision> - e3)/k;
        //std::cout << e1 << ',' << e2 << ','<< e3 << '\n';
        const auto NextAxis = StateInitializer::ConvertEulerAnglesToAxisDirections(NextState.template head<3>());
        // Recursive Mean calculation
        Axes.xAxis += (NextAxis.xAxis - Axes.xAxis)/k;
        Axes.yAxis += (NextAxis.yAxis - Axes.yAxis)/k;
        Axes.zAxis += (NextAxis.zAxis - Axes.zAxis)/k;
    }
    //Axes.xAxis /= number;
    //Axes.yAxis /= number;
    //Axes.zAxis /= number;
    //MagDir /= number;

    EXPECT_TRUE(e1 < error);
    EXPECT_TRUE(e2 < error);
    EXPECT_TRUE(e3 < error);
    EXPECT_TRUE(Axes.xAxis.norm() < error);
    EXPECT_TRUE(Axes.yAxis.norm() < error);
    EXPECT_TRUE(Axes.zAxis.norm() < error);
    //EXPECT_TRUE(MagDir.norm() < error);

    std::cout << "Error:" << error << '\n';
    //std::cout << MagDir << '\n';
    //std::cout << "Norm of MagDir:\n";
    //std::cout << MagDir.norm() << '\n';
    std::cout << "Mean x Axis:\n";
    std::cout << Axes.xAxis << '\n';
    std::cout << "Mean y Axis:\n";
    std::cout << Axes.yAxis << '\n';
    std::cout << "Mean z Axis:\n";
    std::cout << Axes.zAxis << '\n';
    std::cout << "Norm of Axes:\n";
    std::cout << Axes.xAxis.norm() << '\n';
    std::cout << Axes.yAxis.norm() << '\n';
    std::cout << Axes.zAxis.norm() << '\n';
    std::cout << "Mean angles:\n";
    std::cout << e1 << '\n';
    std::cout << e2 << '\n';
    std::cout << e3 << '\n';
}

TEST_F(NeelSphericalFixture, Test_Problem_Random_Orientation_100000)
{
    auto StartState = NeelSphericalFixture::getStart(this->mInitSet);
    auto Axes = NeelSphericalFixture::calculateParticleAxes(this->mInitSet);
    EXPECT_TRUE(Axes.xAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.xAxis.norm() < 1.01);
    EXPECT_TRUE(Axes.yAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.yAxis.norm() < 1.01);
    EXPECT_TRUE(Axes.zAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.zAxis.norm() < 1.01);
    for (auto i = number; --i;)
    {
        const auto k = (number-i)+2;
        const auto NextState = NeelSphericalFixture::getStart(this->mInitSet);
        const auto NextAxis = NeelSphericalFixture::calculateParticleAxes(this->mInitSet);
        // Recursive Mean calculation
        Axes.xAxis += (NextAxis.xAxis - Axes.xAxis)/k;
        Axes.yAxis += (NextAxis.yAxis - Axes.yAxis)/k;
        Axes.zAxis += (NextAxis.zAxis - Axes.zAxis)/k;
        EXPECT_TRUE(NextAxis.xAxis.cross(NextAxis.yAxis).isApprox(NextAxis.zAxis));
    }
    //MagDir /= number;


    EXPECT_TRUE(Axes.xAxis.norm() < error);
    EXPECT_TRUE(Axes.yAxis.norm() < error);
    EXPECT_TRUE(Axes.zAxis.norm() < error);
    
    //EXPECT_TRUE(MagDir.norm() < error);

    std::cout << "Error:" << error << '\n';
    //std::cout << MagDir << '\n';
    //std::cout << "Norm of MagDir:\n";
    //std::cout << MagDir.norm() << '\n';
    std::cout << "Mean x Axis:\n";
    std::cout << Axes.xAxis << '\n';
    std::cout << "Mean y Axis:\n";
    std::cout << Axes.yAxis << '\n';
    std::cout << "Mean z Axis:\n";
    std::cout << Axes.zAxis << '\n';
    std::cout << "Norm of Axes:\n";
    std::cout << Axes.xAxis.norm() << '\n';
    std::cout << Axes.yAxis.norm() << '\n';
    std::cout << Axes.zAxis.norm() << '\n';
}

const typename New_Random_Init_Test::InitSettings New_Random_Init_Test::InitSet = New_Random_Init_Test::createInitializationSettings();

TEST_F(New_Random_Init_Test, Test_New_Random_Orientation_100000)
{
    auto MagDir = StateInitializer::getInitialMagnetisationDirection(New_Random_Init_Test::InitSet);
    auto Orientation = StateInitializer::getInitialParticleOrientation(New_Random_Init_Test::InitSet);
    auto Axes = StateInitializer::ConvertEulerAnglesToAxisDirections(Orientation);
    for (auto i = number; --i;)
    {
        const auto k = (number-i)+2;
        const auto NextOrientation = StateInitializer::getInitialParticleOrientation(New_Random_Init_Test::InitSet);
        const auto NextAxis = StateInitializer::ConvertEulerAnglesToAxisDirections(NextOrientation);
        // Recursive Mean calculation
        Axes.xAxis += (NextAxis.xAxis - Axes.xAxis)/k;
        Axes.yAxis += (NextAxis.yAxis - Axes.yAxis)/k;
        Axes.zAxis += (NextAxis.zAxis - Axes.zAxis)/k;
        const auto NextMagDir = StateInitializer::getInitialMagnetisationDirection(New_Random_Init_Test::InitSet);
        MagDir += (NextMagDir - MagDir)/k;
        EXPECT_TRUE(NextAxis.xAxis.cross(NextAxis.yAxis).isApprox(NextAxis.zAxis));
    }

    EXPECT_TRUE(Axes.xAxis.norm() < error);
    EXPECT_TRUE(Axes.yAxis.norm() < error);
    EXPECT_TRUE(Axes.zAxis.norm() < error);
    EXPECT_TRUE(MagDir.norm() < error);

    std::cout << "Error:" << error << '\n';
    std::cout << MagDir << '\n';
    std::cout << "Norm of MagDir:\n";
    std::cout << MagDir.norm() << '\n';
    std::cout << "Mean x Axis:\n";
    std::cout << Axes.xAxis << '\n';
    std::cout << "Mean y Axis:\n";
    std::cout << Axes.yAxis << '\n';
    std::cout << "Mean z Axis:\n";
    std::cout << Axes.zAxis << '\n';
    std::cout << "Norm of Axes:\n";
    std::cout << Axes.xAxis.norm() <<'\n';
    std::cout << Axes.yAxis.norm() <<'\n';
    std::cout << Axes.zAxis.norm() <<'\n';

}

///---------------------------------------------------------------------------------------------------
