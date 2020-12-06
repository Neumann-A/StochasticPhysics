///---------------------------------------------------------------------------------------------------
// file:		Test_Neel_Spherical_Problem.h
//
// summary: 	Declares the test neel spherical problem class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 07.09.2017

#ifndef INC_Test_Neel_Spherical_Problem_H
#define INC_Test_Neel_Spherical_Problem_H
///---------------------------------------------------------------------------------------------------
#pragma once

//Eigen Core
#include <Eigen/Core>

//Google Test
#include <gtest/gtest.h>

//Properties Includes
#include "Properties/ParticleProperties.h"

//Problem Headers
#include "Selectors/ProblemSelector.h"
#include "Problems/Anisotropy/UniaxialAnisotropy.h"
#include "Problems/NeelRelaxationSpherical.h"

namespace Problems
{
    namespace {
        static constexpr const ::Properties::IAnisotropy sAni = ::Properties::IAnisotropy::Anisotropy_uniaxial;
        using Precision = double;
        using Anisotropy = typename ::Selectors::AnisotropyTypeSelector<sAni>::type<Precision>;
        using Problem = typename ::Problems::NeelRelaxationSpherical<Precision, Anisotropy>;

        using Vec3D = Eigen::Matrix<Precision, 3, 1>;
        using Vec2D = Eigen::Matrix<Precision, 2, 1>;
        using Matrix3x3 = Eigen::Matrix<Precision, 3, 3>;
        using Matrix2x3 = Eigen::Matrix<Precision, 2, 3>;
        using Matrix2x2 = Eigen::Matrix<Precision, 2, 2>;

        static std::random_device rd;
        static std::mt19937_64 prng;
        static std::uniform_real_distribution<Precision> dist{ -10 * math::coordinates::pi<Precision>,10 * math::coordinates::pi<Precision> };

        inline Vec2D getRandomCoords()
        {
            Vec2D res;
            res(0) = dist(prng);
            res(1) = dist(prng);
            return res;
        }

    }

    class NeelSphericalProblemTest : public ::testing::Test, public ::Problems::Problem
    {
    public:
        using InitSettings = typename Problem::InitSettings;
        using Properties = typename Problem::UsedProperties;
        using ProblemSettings = typename Problem::ProblemSettings;

    private:
        inline static ProblemSettings createProblemSettings()
        {
            ProblemSettings ProbSet{};
            ProbSet.mUseCoordinateTransformation = true;
            ProbSet.mMinAngleBeforeTransformation = math::constants::pi<Precision> / 6.0; //30�!
            return ProbSet;
        }
        inline static InitSettings createInitializationSettings()
        {
            //constexpr const Precision pi{ 3.1415926535897932384626433832795 };

            Vec3D Pos, Orientation, MagDir;
            Pos << 0, 0, 0; //Unimportant
            Orientation << 0, 0, 0; //Euler Angles!; Defines Easy Axis Direction
            MagDir << 1, 0, 0; //Starting Direction of Magnetisation; Mainly unimportant for test;
            return InitSettings(false, false, false, Pos, Orientation, MagDir);
        }
        inline static Properties createProperties()
        {
            //General Parameters
            const Precision T = 295;
            const Precision visc = 1E-3;

            //Magnetic Parameters
            const Precision damping = 0.1;
            const Precision gyro = 1.76E+11;
            const Precision Ms = 4.77464E5;
            const Precision rmag = 10E-9;
            const Precision KUni = 1E4;

            //Hydrodynamic parameters
            const Precision rhydro = 100E-9;

            const ::Properties::MagneticProperties<Precision> MagProps{ rmag,Ms,damping,gyro,sAni, ::Properties::Anisotropy::Uniaxial<Precision>{ {}, {KUni} } };
            const ::Properties::HydrodynamicProperties<Precision> HydroProps{ rhydro,visc };

            return ::Properties::ParticlesProperties<Precision>{T, MagProps, HydroProps};
        }

    public:
        ProblemSettings mSettings;
        InitSettings	mInitSet;
        Properties		mProperties;

        inline NeelSphericalProblemTest()
            : Problem(createProblemSettings(), createProperties(), createInitializationSettings())
        {
            std::array<std::random_device::result_type, std::mt19937_64::state_size> seed_data;
            std::generate(seed_data.begin(), seed_data.end(), [&]() {return rd(); });
            std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
            prng = std::mt19937_64{ seq };
            prng.discard(1'000'000);
        };

    };
}
#endif	// INC_Test_Neel_Spherical_Problem_H
// end of Test_Neel_Spherical_Problem.h
