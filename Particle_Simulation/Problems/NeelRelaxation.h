///---------------------------------------------------------------------------------------------------
// file:        NeelRelaxation.h
//
// summary:     Declares the neel relaxation class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 07.02.2017

#ifndef INC_NeelRelaxation_H
#define INC_NeelRelaxation_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <tuple>
#include <random>

#include "../SDEFramework/GeneralSDEProblem.h"
#include "Helpers/ParameterCalculatorNeel.h"

namespace Problems
{
    constexpr static struct NeelDimension : GeneralSDEDimension<3, 3, 3> //thats pretty handy
    { } NeelDimensionVar; //too get the memory space (else the compiler will optimize it away)

    template<typename precision, typename aniso>
    class NeelRelaxation :
        public GeneralSDEProblem <NeelRelaxation<precision, aniso>>
    {
    public:
        using ThisClass = NeelRelaxation<precision, aniso>;
        typedef SDEProblem_Traits<ThisClass>                Traits;
        typedef precision                                   Precision;

        typedef typename Traits::Dimension                  Dimension;
        typedef typename Traits::ProblemSettings            ProblemSettings;
        typedef typename Traits::UsedProperties             UsedProperties;
        typedef typename Traits::InitSettings               InitSettings;
        typedef typename Traits::NecessaryProvider          NecessaryProvider;
        typedef typename Traits::SimulationParameters       SimulationParameters;

        typedef aniso                                       Anisotropy;

        typedef typename Traits::StochasticMatrixType       StochasticMatrixType;
        typedef typename Traits::DeterministicType          DeterministicType;
        typedef typename Traits::DependentType              DependentType;
        typedef typename Traits::IndependentType            IndependentType;
        typedef typename Traits::NoiseType                  NoiseType;


        using OutputType = DependentType;

        template<typename T>
        using BaseMatrixType = typename Traits::template BaseMatrixType<T>;

        using JacobiMatrixType = typename Traits::JacobiMatrixType;
    private: // Important: Have often used Parameters at the top of the class defintion!
        
        //Particle Parameters
        Helpers::NeelParams<Precision>    mParams;
    
        
        //Helper Matrix
        struct CoordinateSystem
        {
            IndependentType xAxis;
            IndependentType yAxis;
            IndependentType zAxis;
        };
        const CoordinateSystem ParticleAxes;
    
        const Anisotropy                mAnisotropy;
        const ProblemSettings            mProblemSettings;

    public:
        const UsedProperties        mParParams;
        const InitSettings          mInit;

        ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit NeelRelaxation(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
            GeneralSDEProblem<NeelRelaxation<precision, aniso>>(NeelDimensionVar),
            mParams(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
            ParticleAxes(calculateParticleAxes(Init)),
            mAnisotropy(Properties.getMagneticProperties()),
            mProblemSettings(ProbSettings), mParParams(Properties), mInit(Init)
             {
        };

        template<typename Derived>
        BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const BaseMatrixType<Derived>& yi) const
        {
            const auto NeelNoise_H_Pre2 = mParams.NeelFactor2*mParams.NoisePrefactor;
            const auto NeelNoise_H_Pre1 = mParams.NeelFactor1*mParams.NoisePrefactor;

            StochasticMatrixType StochasticMatrix{ NeelNoise_H_Pre2*StochasticMatrixType::Identity() - (NeelNoise_H_Pre2*yi)*yi.transpose() };
            const auto yi2{ NeelNoise_H_Pre1*yi };

            //Crossproduct matrix
            StochasticMatrix(0,1) -= yi2(2);
            StochasticMatrix(0,2) += yi2(1);
            StochasticMatrix(1,0) += yi2(2);
            StochasticMatrix(1,2) -= yi2(0);
            StochasticMatrix(2,0) -= yi2(1);
            StochasticMatrix(2,1) += yi2(0);
            
            return StochasticMatrix;
        }

        template<typename Derived>
        BASIC_ALWAYS_INLINE DeterministicType getDrift(const BaseMatrixType<Derived>& yi) const
        {
            return (mParams.DriftPrefactor*yi).eval();
        }

        template<typename Derived, typename Derived2>
        BASIC_ALWAYS_INLINE DeterministicType getDeterministicVector(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi) const
        {
            const auto& xAxis = ParticleAxes.xAxis;
            const auto& yAxis = ParticleAxes.yAxis;
            const auto& zAxis = ParticleAxes.zAxis;

            mAnisotropy.prepareField(yi, xAxis, yAxis, zAxis);
            const auto Heff{ (mAnisotropy.getAnisotropyField(yi,xAxis,yAxis,zAxis) + xi) };            
    
            return (mParams.NeelFactor1*yi.cross(Heff) - mParams.NeelFactor2*yi.cross(yi.cross(Heff))).eval();
        }

        template<typename Derived>
        BASIC_ALWAYS_INLINE void prepareJacobiCalculations(BaseMatrixType<Derived>& /* yi */)
        {}

        template<typename Derived, typename Derived2>
        BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiDeterministic(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi,const Precision& dt) const
        {
            const auto& xAxis = ParticleAxes.xAxis;
            const auto& yAxis = ParticleAxes.yAxis;
            const auto& zAxis = ParticleAxes.zAxis;


            //Deterministc Jacobi Matrix
            const auto HeffJacobi{ mAnisotropy.getJacobiAnisotropyField(yi,xAxis,yAxis,zAxis) };

            JacobiMatrixType m_plus{ JacobiMatrixType::Zero() };
            {
                const auto& m{ yi };
                m_plus(0, 1) = -m(2);
                m_plus(0, 2) = +m(1);
                m_plus(1, 0) = +m(2);
                m_plus(1, 2) = -m(0);
                m_plus(2, 0) = -m(1);
                m_plus(2, 1) = +m(0);
            }

            const auto Heff{ (mAnisotropy.getAnisotropyField(yi,xAxis,yAxis,zAxis) + xi) };

            //JacobiMatrixType JacobiDet{ mParams.NeelFactor1*m_plus*HeffJacobi - static_cast<Precision>(2.0)*mParams.Damping/dt*m_plus };
            JacobiMatrixType JacobiDet{ m_plus*HeffJacobi };
            {
                JacobiDet(0, 1) += Heff(2);
                JacobiDet(0, 2) -= Heff(1);
                JacobiDet(1, 0) -= Heff(2);
                JacobiDet(1, 2) += Heff(0);
                JacobiDet(2, 0) += Heff(1);
                JacobiDet(2, 1) -= Heff(0);
            }
            JacobiDet = mParams.NeelFactor1*JacobiDet - static_cast<Precision>(2.0)*mParams.Damping / dt*m_plus;

            return JacobiDet;
        }

        BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiStochastic(const NoiseType& dW) const
        {
            JacobiMatrixType JacobiSto{ JacobiMatrixType::Zero() };
            //Crossproduct matrix (c * dW) (minus due to minus sign in NeelNoise_H_Pre1)
            {
                const auto NeelNoise_H_Pre1 = mParams.NeelFactor1*mParams.NoisePrefactor;
                const auto dw2{ -NeelNoise_H_Pre1*dW };

                JacobiSto(0, 1) -= dw2(2);
                JacobiSto(0, 2) += dw2(1);
                JacobiSto(1, 0) += dw2(2);
                JacobiSto(1, 2) -= dw2(0);
                JacobiSto(2, 0) -= dw2(1);
                JacobiSto(2, 1) += dw2(0);
                //std::cout << "Param\n" << mParams.NeelNoise_H_Pre1 << "\ndW\n" << dW << "\ndW2\n" << dw2 << "\nJacobiSto\n" << JacobiSto << std::endl;
            }
            return JacobiSto;
        }

        template<typename Derived>
        BASIC_ALWAYS_INLINE void finishJacobiCalculations(BaseMatrixType<Derived>& /* jacobi */) const noexcept {}
        
        template<typename Derived>
        BASIC_ALWAYS_INLINE void prepareCalculations(const BaseMatrixType<Derived>& /* yi */) const noexcept {}

        template<typename Derived>
        BASIC_ALWAYS_INLINE void prepareJacobiCalculations(const BaseMatrixType<Derived>& /* yi */) const noexcept {}

        template<typename Derived>
        BASIC_ALWAYS_INLINE void finishCalculations(BaseMatrixType<Derived>& /* yi */) const    noexcept{}

        template<typename Derived>
        BASIC_ALWAYS_INLINE void normalize(BaseMatrixType<Derived>& yi) const
        {
            yi.normalize();
        }

        template<typename Derived>
        BASIC_ALWAYS_INLINE OutputType calculateOutputResult(const BaseMatrixType<Derived>& yi) const noexcept
        {
            return static_cast<const Derived&>(yi);
        }

        ///-------------------------------------------------------------------------------------------------
        /// <summary>    Calculates the direction of the easy axis. </summary>
        ///
        /// <param name="init">    The initilization settings. </param>
        ///
        /// <returns>    The calculated easy axis direction. </returns>
        ///-------------------------------------------------------------------------------------------------
        static BASIC_ALWAYS_INLINE CoordinateSystem calculateParticleAxes(const InitSettings& init)
        {
            std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
            std::uniform_real_distribution<Precision> ud{ 0.0,1.0 };

            IndependentType EulerAngles;

            //Rotation of Coordinates (theta',phi') to (theta,phi) -90� around rotated y'-axis;
            if (init.getUseRandomInitialParticleOrientation()) {
                for (typename IndependentType::Index i = 0; i < 3; ++i) {
                    EulerAngles(i) = ud(rd)*math::constants::two_pi<Precision>;
                }
            }
            else {
                EulerAngles = init.getInitialParticleOrientation();
            }

            const auto Sines = EulerAngles.array().sin();
            const auto Cosines = EulerAngles.array().cos();

            //Define some easy bindings
            const auto& cphi = Cosines(0);
            const auto& ctheta = Cosines(1);
            const auto& cpsi = Cosines(2);
            const auto& sphi = Sines(0);
            const auto& stheta = Sines(1);
            const auto& spsi = Sines(2);

            //Phi and Psi products (used twice)
            const auto cphicpsi = cphi*cpsi;
            const auto sphicpsi = sphi*cpsi;
            const auto cphispsi = cphi*spsi;
            const auto sphispsi = sphi*spsi;

            IndependentType xAxis(cphicpsi - ctheta*sphispsi, -sphicpsi - ctheta*cphispsi, stheta*spsi);
            IndependentType yAxis(ctheta*sphicpsi + cphispsi, ctheta*cphicpsi - sphispsi, -stheta*cpsi);
            IndependentType zAxis(stheta*sphi, stheta*cphi, ctheta);
            return { xAxis, yAxis, zAxis };
        }

        inline decltype(auto) getStart(const InitSettings& Init) noexcept
        {
            DependentType Result;

            std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
            std::normal_distribution<precision> nd{ 0,1 };

            //Init Magnetisation Direction
            if (Init.getUseRandomInitialMagnetisationDir())
            {
                DependentType MagDir;
                for (unsigned int i = 0; i < 3; ++i)
                    MagDir(i) = nd(rd);
                Result = MagDir;
            }
            else
            {
                Result = Init.getInitialMagnetisationDirection();
            }

            Result.normalize(); //normalize if necessary
            return Result;
        }

        static auto getWeighting(const UsedProperties &Properties) noexcept
        {
            OutputType scale{ OutputType::Ones() };
            return (scale * Properties.getMagneticProperties().getSaturationMoment()).eval();
        }

        template<typename Derived, typename Derived2>
        BASIC_ALWAYS_INLINE void staticVectorChecks(const BaseMatrixType<Derived> &/* yi */, const Derived2 &/* tester */) const noexcept
        {
            using ToTest = Derived;
            using TestType = Derived2;
            static_assert(std::is_same_v<typename ToTest::Scalar, typename TestType::Scalar>, "Matrix scalar types do not agree!");
            #if defined(__clang__) || defined(GCC_VERSION)
                #pragma GCC diagnostic push
                #pragma GCC diagnostic ignored "-Wenum-compare"
            #endif
            static_assert(ToTest::RowsAtCompileTime == TestType::RowsAtCompileTime, "Number of rows do not agree!");
            static_assert(ToTest::ColsAtCompileTime == TestType::ColsAtCompileTime, "Number of cols do not agree!");
            #if defined(__clang__) || defined(GCC_VERSION)
                #pragma GCC diagnostic pop
            #endif
        }
    };


}

#include "Definitions/NeelRelaxation_Definitions.h"

#endif    // INC_NeelRelaxation_H
// end of NeelRelaxation.h
///---------------------------------------------------------------------------------------------------
