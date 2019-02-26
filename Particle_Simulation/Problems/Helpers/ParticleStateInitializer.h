///---------------------------------------------------------------------------------------------------
// file:		ParticleStateInitializer
//
// summary: 	Helper class to initialize the starting state of the particles
//
// Copyright (c) 2019 Alexander Neumann.
//
// author: Alexander Neumann
// date: 26.02.2019
///---------------------------------------------------------------------------------------------------
#pragma once

#include <cmath>
#include <random>
#include <type_traits>

#include <math/random_helpers.h>
#include <math/math_constants.h>

namespace Problems::Helpers
{
    template<typename Problem>
    struct ParticleStateInitializer
    {
        using DependentType = typename Problem::DependentType;
        using IndependentType = typename Problem::IndependentType;
        using Precision = typename Problem::Precision;
        using InitSettings = typename Problem::InitSettings;

        using RndDev = std::random_device;
        using Prng   = std::mt19937_64;

        static thread_local RndDev rnddev;
        static thread_local Prng prng{ math::random_helpers::create_seeded_PRNG<Prng>(rnddev) };
        static thread_local std::uniform_real_distribution<Precision> ud{ 0.0,1.0 };     
        static thread_local std::normal_distribution<Precision> nd{ 0.0,1.0 };
        
        using OrientationType = std::decay_t(std::invoke_result_t<InitSettings::getInitialParticleOrientation,void>);
        using MagnetisationType = std::decay_t(std::invoke_result_t<InitSettings::getInitialMagnetisationDirection, void>);

        static inline auto getInitialParticleOrientation(const typename InitSettings& init) noexcept
        {
            using ResultType = OrientationType;
            //using ResultType = std::decay_t<decltype(init.getInitialParticleOrientation())>;

            if (init.getUseRandomInitialParticleOrientation())
            {
                ResultType Result;
                Result(0) = ud(prng) * math::constants::two_pi<Precision>;
                Result(1) = ud(prng) * math::constants::pi<Precision>;
                Result(2) = ud(prng) * math::constants::two_pi<Precision>;
                return Result;
            }
            else
            {
                return init.getInitialParticleOrientation();
            }
        };
        
        static inline auto getInitialMagnetisationDirection(const typename InitSettings& init) noexcept
        {
            using ResultType = MagnetisationType;
            //using ResultType = std::decay_t<decltype(init.getInitialMagnetisationDirection())>;

            if (init.getUseRandomInitialMagnetisationDir())
            {
                ResultType Result;
                Result(0) = nd(prng);
                Result(1) = nd(prng);
                Result(2) = nd(prng);
                Result.normalize();
                return Result;
            }
            else
            {
                return init.getInitialMagnetisationDirection();
            }
        };

        template<typename ResultType>
        static inline auto ConvertMagnetisationDirectionToSphericalCoordinates(const MagnetisationType& tmp, ResultType& Result) noexcept
        {
            IndependentType z_axis;
            IndependentType y_axis;
            IndependentType x_axis;
            x_axis << 1.0, 0.0, 0.0;
            y_axis << 0.0, 1.0, 0.0;
            z_axis << 0.0, 0.0, 1.0;
            Result(0) = std::acos(tmp.dot(z_axis)); //Theta
            Result(1) = std::atan2(tmp.dot(y_axis), tmp.dot(x_axis)); //Phi
        }
    };
}

///---------------------------------------------------------------------------------------------------