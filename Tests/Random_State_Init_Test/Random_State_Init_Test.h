///---------------------------------------------------------------------------------------------------
// file:		Random_State_init_Test.h
//
// summary: 	Test class to check if the random initialization is uniform
//
// Copyright (c) 2019 Alexander Neumann.
//
// author: Alexander
// date: 01.03.2019

///---------------------------------------------------------------------------------------------------
#pragma once

#include <random>

//Eigen Core
#include <Eigen/Core>

//Google Test
#include <gtest/gtest.h>

#include <MyCEL/math/math_constants.h>
#include <MyCEL/math/Coordinates.h>
#include <MyCEL/math/random_helpers.h>

//Properties Includes
#include "Properties/ParticleProperties.h"

//Problem Headers
#include "Selectors/ProblemSelector.h"
#include "Problems/Anisotropy/UniaxialAnisotropy.h"
#include "Problems/BrownAndNeelRelaxationEulerSpherical.h"
#include "Problems/NeelRelaxationSpherical.h"

#include "Problems/Helpers/ParticleStateInitializer.h"

static std::mt19937_64 prng_global = math::random_helpers::create_seeded_PRNG<std::mt19937_64>(std::random_device{});

namespace Problems
{
    namespace {
        static constexpr const ::Properties::IAnisotropy sAni = ::Properties::IAnisotropy::Anisotropy_uniaxial;
        using Precision = double;
        using Anisotropy = typename ::Selectors::AnisotropyTypeSelector<sAni>::type<Precision>;
        using Problem = typename ::Problems::BrownAndNeelRelaxationEulerSpherical<Precision, Anisotropy>;
        using ProblemNeel = typename ::Problems::NeelRelaxationSpherical<Precision, Anisotropy>;

        using Vec3D = Eigen::Matrix<Precision, 3, 1>;
        using Vec5D = Eigen::Matrix<Precision, 5, 1>;
        using Matrix5x6 = Eigen::Matrix<Precision, 5, 6>;

        static std::random_device rd;
        static std::mt19937_64 prng;
        static std::uniform_real_distribution<Precision> dist{ -10.0 * math::coordinates::pi<Precision>,10.0 * math::coordinates::pi<Precision> };

        inline Vec5D getRandomCoords()
        {
            Vec5D res;
            res(0) = dist(prng);
            res(1) = dist(prng);
            return res;
        }

    }

    template<typename Problem>
    class Problem_Random_Init_Test : public Problem, public ::testing::Test
    {
    public:
        using InitSettings = typename Problem::InitSettings;
        using Properties = typename Problem::UsedProperties;
        using ProblemSettings = typename Problem::ProblemSettings;
        using StateInitializer = typename Problems::Helpers::template ParticleStateInitializer<Problem>;

    private:
        inline static ProblemSettings createProblemSettings()
        {
            ProblemSettings ProbSet{};
            //ProbSet.mUseCoordinateTransformation = false;
            //ProbSet.mUseEulerCoordinateTransformation = true;
            //ProbSet.mUseSphericalCoordinateTransformation = true;
            //ProbSet.mBrownMinAngleBeforeTransformation = math::constants::pi<Precision> / 6.0; //30�!
            //ProbSet.mNeelMinAngleBeforeTransformation = math::constants::pi<Precision> / 6.0; //30�!
            return ProbSet;
        }
        inline static InitSettings createInitializationSettings()
        {
            //constexpr const Precision pi{ 3.1415926535897932384626433832795 };

            Vec3D Pos, Orientation, MagDir;
            Pos << 0, 0, 0; //Unimportant
            Orientation << 0, 0, 0; //Euler Angles!; Defines Easy Axis Direction
            MagDir << 1, 0, 0; //Starting Direction of Magnetisation; Mainly unimportant for test;
            return InitSettings(true, true, true, Pos, Orientation, MagDir);
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
            const Precision KUni = -1E4;

            //Hydrodynamic parameters
            const Precision rhydro = 20E-9;

            const ::Properties::MagneticProperties<Precision> MagProps{ rmag,Ms,damping,gyro,{sAni,::Properties::Anisotropy::Uniaxial<Precision>{ {}, KUni }} };
            const ::Properties::HydrodynamicProperties<Precision> HydroProps{ rhydro,visc };

            return ::Properties::ParticlesProperties<Precision>{T, MagProps, HydroProps};
        }

    public:
        //ProblemSettings mSettings;
        InitSettings	mInitSet;
        //Properties		mProperties;

        Problem_Random_Init_Test()
            : Problem(createProblemSettings(), createProperties(), createInitializationSettings()), ::testing::Test()
        {
            prng = math::random_helpers::create_seeded_PRNG<std::mt19937_64>(rd);
            prng.discard(1'000'000);
        };

    };

    
    class New_Random_Init_Test : public ::testing::Test
    {
    public:
        using StateInitializer = typename Problems::Helpers::ParticleStateInitializer<Problem>;

        using InitSettings = typename Problem::InitSettings;

        inline static InitSettings createInitializationSettings()
        {
            //constexpr const Precision pi{ 3.1415926535897932384626433832795 };

            Vec3D Pos, Orientation, MagDir;
            Pos << 0, 0, 0; //Unimportant
            Orientation << 0, 0, 0; //Euler Angles!; Defines Easy Axis Direction
            MagDir << 1, 0, 0; //Starting Direction of Magnetisation; Mainly unimportant for test;
            return InitSettings(true, true, true, Pos, Orientation, MagDir);
        }

        static const InitSettings InitSet;
    };


    using BrownAndNeelEulerSphericalFixture = Problem_Random_Init_Test<Problem>;
    using NeelSphericalFixture = Problem_Random_Init_Test<ProblemNeel>;
}
