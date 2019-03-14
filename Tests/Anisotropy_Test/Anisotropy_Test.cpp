///-------------------------------------------------------------------------------------------------
// file:	Random_State_init_Test.cpp
//
// summary:	Implements the tests to run
///-------------------------------------------------------------------------------------------------

#include "Random_State_Init_Test.h"

#include <Eigen/Geometry>

using namespace Problems;
namespace {
    const auto number = 1'000'000;
    const auto error = 2.0 / std::sqrt(number);
}

TEST_F(BrownAndNeelEulerSphericalFixture, Test_Problem_Random_Orientation_100000)
{
    auto StartState = BrownAndNeelEulerSphericalFixture::getStart(this->mInitSet);
    auto Axes = StateInitializer::ConvertEulerAnglesToAxisDirections(StartState.template head<3>());
    EXPECT_TRUE(Axes.xAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.xAxis.norm() < 1.01);
    EXPECT_TRUE(Axes.yAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.yAxis.norm() < 1.01);
    EXPECT_TRUE(Axes.zAxis.norm() > 0.99);
    EXPECT_TRUE(Axes.zAxis.norm() < 1.01);
    for (auto i = number; --i;)
    {
        const auto NextState = BrownAndNeelEulerSphericalFixture::getStart(this->mInitSet);
        const auto NextAxis = StateInitializer::ConvertEulerAnglesToAxisDirections(NextState.template head<3>());
        Axes.xAxis += NextAxis.xAxis;
        Axes.yAxis += NextAxis.yAxis;
        Axes.zAxis += NextAxis.zAxis;
    }
    Axes.xAxis /= number;
    Axes.yAxis /= number;
    Axes.zAxis /= number;
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
};

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
        const auto NextState = NeelSphericalFixture::getStart(this->mInitSet);
        const auto NextAxis = NeelSphericalFixture::calculateParticleAxes(this->mInitSet);
        Axes.xAxis += NextAxis.xAxis;
        Axes.yAxis += NextAxis.yAxis;
        Axes.zAxis += NextAxis.zAxis;
        EXPECT_TRUE(NextAxis.xAxis.cross(NextAxis.yAxis).isApprox(NextAxis.zAxis));
    }
    Axes.xAxis /= number;
    Axes.yAxis /= number;
    Axes.zAxis /= number;
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
};

const typename New_Random_Init_Test::InitSettings New_Random_Init_Test::InitSet = New_Random_Init_Test::createInitializationSettings();

TEST_F(New_Random_Init_Test, Test_New_Random_Orientation_100000)
{
    auto MagDir = StateInitializer::getInitialMagnetisationDirection(New_Random_Init_Test::InitSet);
    auto Orientation = StateInitializer::getInitialParticleOrientation(New_Random_Init_Test::InitSet);
    auto Axes = StateInitializer::ConvertEulerAnglesToAxisDirections(Orientation);
    for (auto i = number; --i;)
    {
        const auto NextOrientation = StateInitializer::getInitialParticleOrientation(New_Random_Init_Test::InitSet);
        const auto NextAxis = StateInitializer::ConvertEulerAnglesToAxisDirections(NextOrientation);
        Axes.xAxis += NextAxis.xAxis;
        Axes.yAxis += NextAxis.yAxis;
        Axes.zAxis += NextAxis.zAxis;
        MagDir += StateInitializer::getInitialMagnetisationDirection(New_Random_Init_Test::InitSet);
        EXPECT_TRUE(NextAxis.xAxis.cross(NextAxis.yAxis).isApprox(NextAxis.zAxis));
    }
    Axes.xAxis /= number;
    Axes.yAxis /= number;
    Axes.zAxis /= number;
    MagDir /= number;

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

};




///---------------------------------------------------------------------------------------------------