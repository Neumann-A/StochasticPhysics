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

#include <MyCEL/math/random_helpers.h>
#include <MyCEL/math/math_constants.h> // with c++ 20 use maybe <numbers> instead?

namespace Problems::Helpers
{
    using RndDev = std::random_device;
    using Prng   = std::mt19937_64;

    template<typename Problem>
    struct ParticleStateInitializer
    {
        using DependentType = typename Problem::DependentType;
        using IndependentType = typename Problem::IndependentType;
        using Precision = typename Problem::Precision;
        using InitSettings = typename Problem::InitSettings;
        
        using OrientationType = std::decay_t<std::invoke_result_t<decltype(&InitSettings::getInitialParticleOrientation), InitSettings>>;
        using MagnetisationType = std::decay_t<std::invoke_result_t<decltype(&InitSettings::getInitialMagnetisationDirection), InitSettings>>;
        
        static inline Prng& getPRNG()
        {
            static thread_local Prng prng{ math::random_helpers::create_seeded_PRNG<Prng>(std::random_device{}) };
            return prng;
        }

        static inline auto getInitialParticleOrientation(const InitSettings& init) noexcept
        {
            using ResultType = OrientationType;
            //using ResultType = std::decay_t<decltype(init.getInitialParticleOrientation())>;


            if (init.getUseRandomInitialParticleOrientation())
            {
                static thread_local std::uniform_real_distribution<typename Problem::Precision> ud{ 0.0,1.0 };
                static auto prng = getPRNG();
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

        static inline auto ConvertEulerAnglesToAxisDirections(const OrientationType& eulerangles)
        {
            struct Result {
                OrientationType xAxis;
                OrientationType yAxis;
                OrientationType zAxis;
            };

            //const auto Sines = eulerangles.array().sin().eval();
            //const auto Cosines = eulerangles.array().cos().eval();
            const auto Sines = sin(eulerangles.array());
            const auto Cosines = cos(eulerangles.array());

            //Define some easy bindings
            const auto& cphi = Cosines(0);
            const auto& ctheta = Cosines(1);
            const auto& cpsi = Cosines(2);
            const auto& sphi = Sines(0);
            const auto& stheta = Sines(1);
            const auto& spsi = Sines(2);

            //Phi and Psi products (used twice)
            const auto cphicpsi = cphi * cpsi;
            const auto sphicpsi = sphi * cpsi;
            const auto cphispsi = cphi * spsi;
            const auto sphispsi = sphi * spsi;

            //const auto& a = eulerangles[0]; //!< Alpha
            //const auto& b = eulerangles[1];	//!< Beta
            //const auto& g = eulerangles[2]; //!< Gamma
            //using ::std::cos;
            //using ::std::sin;

            Result res{ {cphicpsi - ctheta * sphispsi, -sphicpsi - ctheta * cphispsi, stheta * spsi},
                        {ctheta * sphicpsi + cphispsi, ctheta * cphicpsi - sphispsi, -stheta * cpsi},
                        {stheta * sphi, stheta * cphi, ctheta }  };

            //Result res{ {cos(a) * cos(g) - sin(a) * cos(b) * sin(g), sin(a) * cos(g) + cos(a) * cos(b) * sin(g), sin(b) * sin(g)},
            //{-cos(a) * sin(g) - sin(a) * cos(b) * cos(g), -sin(a) * sin(g) + cos(a) * cos(b) * cos(g), sin(b) * cos(g)},{
            //    sin(a) * sin(b), -cos(a) * sin(b), cos(b)} };
            return res;
        }
        
        static inline auto getInitialMagnetisationDirection(const InitSettings& init) noexcept
        {
            using ResultType = MagnetisationType;
            //using ResultType = std::decay_t<decltype(init.getInitialMagnetisationDirection())>;


            if (init.getUseRandomInitialMagnetisationDir())
            {
                static thread_local std::uniform_real_distribution<typename Problem::Precision> ud{ 0.0,1.0 };
                static auto prng = getPRNG();
                const auto phi = ud(prng) * math::constants::two_pi<Precision>;
                const auto theta = ud(prng) * math::constants::pi<Precision>;
                ResultType Result;
                const auto stheta = sin(theta);
                Result(0) = cos(phi)*stheta;
                Result(1) = sin(phi)*stheta;
                Result(2) = cos(theta);
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
            return;
        }
    };   
}

///---------------------------------------------------------------------------------------------------
