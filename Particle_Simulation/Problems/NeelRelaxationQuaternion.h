///---------------------------------------------------------------------------------------------------
// file:		NeelRelaxationQuaternion.h
//
// summary: 	Declares the neel relaxation quaternion class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 29.09.2017

#ifndef INC_NeelRelaxationQuaternion_H
#define INC_NeelRelaxationQuaternion_H
///---------------------------------------------------------------------------------------------------
#pragma once

// The Plan:
// The calculation vector will be the angle axis vector
// The solver will solve the time derivative of the change of the angle axis vector
// the problem has to supply the appriopiate determisitic vector / stoachastic matrix
// The problem will store the current magnetisation state (we need it for calculation)
// The problem will use quaternions for calculation by converting the angle axis vector into a unit quaternion
// Solving the quaternion rate is not directly possible due to the unit norm constraint 
// -> Thats why we solve the angle axis rate
// (It is the same reason why Neel cannot be directly solved using cartesian coordinates but spherical are ok) 
// The problem start vector will be zero angle axis

// Nice idea but the drift term is very complicated and needed for calculations!

#include <random>
#include <cmath>

#include <MyCEL/math/Coordinates.h>

#include "../SDEFramework/GeneralSDEProblem.h"
#include "Helpers/ParameterCalculatorNeel.h"



namespace Problems
{
    constexpr static struct NeelQuaternionDimension : GeneralSDEDimension<3, 3, 3> //thats pretty handy
    { } NeelQuaternionDimensionVar; //too get the memory space (else the compiler will optimize it away)

    ///-------------------------------------------------------------------------------------------------
    /// <summary>	Class describing the Neel relaxation problem using quaternions. </summary>
    ///
    /// <seealso cref="T:GeneralSDEProblem{NeelRelaxationQuaternion{precision,anisotropy}}"/>
    ///-------------------------------------------------------------------------------------------------
    template<typename precision, typename anisotropy>
    class NeelRelaxationQuaternion :
        public GeneralSDEProblem <NeelRelaxationQuaternion<precision, anisotropy>>
    {
    private:
        using ThisClass = NeelRelaxationQuaternion<precision, aniso>;
    public:
        using Traits = SDEProblem_Traits<ThisClass>;
        using Precision = precision;
        using Anisotropy = anisotropy;

        using Dimension = typename Traits::Dimension;
        using ProblemSettings = typename Traits::ProblemSettings;
        using UsedProperties = typename Traits::UsedProperties;
        using InitSettings = typename Traits::InitSettings;
        using NecessaryProvider = typename Traits::NecessaryProvider;
        using SimulationParameters = typename Traits::SimulationParameters;

        using DeterministicType = typename Traits::DeterministicType;
        using DependentType = typename Traits::DependentType;
        using IndependentType = typename Traits::IndependentType;
        using NoiseType = typename Traits::NoiseType;
        using StochasticMatrixType = typename Traits::StochasticMatrixType;
        using JacobiMatrixType = typename Traits::JacobiMatrixType;
        using CoordinateTransformationType = typename Traits::CoordinateTransformationType;
        using OutputType = typename Traits::OutputType;

        template<typename T>
        using BaseMatrixType = typename Traits::template BaseMatrixType<T>;

        using Matrix_3x3 = typename Traits::Matrix_3x3;
    private:
        //Useful type definitions
        using AngleAxisType = Traits::AngleAxisType;
        using QuaternionType = Traits::QuaternionType;
    private: // Important: Have often used Parameters at the top of the class defintion!
        Helpers::NeelParams<Precision>	mParams;	//Particle parameters
        IndependentType mEasyAxis;					//Easy axis Direction

        OutputType magnetisationdirection{ OutputType::Zero() };
        OutputType newmagnetisationdirection{ OutputType::Zero() };

        AngleAxisType angleaxis;
        
        Matrix_3x3 helper{ Matrix_3x3::Identity() };
        
        //QuaternionType sumquaternion;

        const Anisotropy				mAnisotropy;		//Anisotropy
        const ProblemSettings			mProblemSettings;	//Problem Settings
        
    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            explicit NeelRelaxationQuaternion(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
            GeneralSDEProblem<NeelRelaxationQuaternion<precision, aniso>>(NeelDimensionVar),
            mParams(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
            mEasyAxis(initEasyAxis(Init)),
            mAnisotropy(Properties.getMagneticProperties()),
            mProblemSettings(ProbSettings)
        {
            static_assert(std::is_same_v<double,Precision>,"Do not try to use this class. Not implemented correctly yet. Does not work!")
            assert(mEasyAxis.norm() <= 1.0 + 10.0 * std::numeric_limits<Precision>::epsilon() && mEasyAxis.norm() >= 1.0 - 10.0 * std::numeric_limits<Precision>::epsilon());
        };


        template<typename Derived>
        BASIC_ALWAYS_INLINE void prepareCalculations(const BaseMatrixType<Derived>& yi) const  
        {
            const auto squarednorm = yi.squaredNorm();
            const auto angleaxisnorm = std::sqrt(squarednorm);

            //Check: Do we need this?
            angleaxis = AngleAxisType(angleaxisnorm, yi / angleaxisnorm);

            if (angleaxisnorm < (math::constants::pi<Precision>*0.1))
            {
                //Linearization only works for small angle rotations!
                // This is 1/2*V(v)^-T n linearized form!
                helper = Matrix_3x3::Constant( 4.0/(4.0+squarednorm) );
                //Upper right corner
                helper(0, 1) *= -0.5*yi(2);
                helper(0, 2) *=  0.5*yi(1);
                helper(1, 2) *= -0.5*yi(0);
                //Lower left corner
                helper(1, 0) *=  0.5*yi(2);
                helper(2, 0) *= -0.5*yi(1);
                helper(2, 1) *=  0.5*yi(0);

                //Calculate new magnetisation direction
                newmagnetisationdirection = quaternion.toRotationMatrix()*magnetisationdirection;
            }
            else
            {
                assert(false); //Not implemented yet
            }
        };


        template<typename Derived, typename Derived2>
        BASIC_ALWAYS_INLINE DeterministicType getDeterministicVector(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi) const
        {
            const auto &magdir = newmagnetisationdirection;

            const auto Heff{ ((mAnisotropy.getAnisotropyField(magdir,mEasyAxis) + xi)).eval() };

            const DeterministicType omega{ -mParams.NeelFactor1*Heff + mParams.NeelFactor2*(magdir.cross(Heff)) };
            
            const DeterministicType res{ helper*omega };

            return res;
        };

        template<typename Derived>
        BASIC_ALWAYS_INLINE auto getStochasticMatrix(const BaseMatrixType<Derived>& yi) const
        {
            StochasticMatrixType StochasticMatrix; //{ -mParams.NeelNoise_H_Pre1*StochasticMatrixType::Identity() };

            //Diagonal:
            StochasticMatrix(0, 0) = -mParams.NeelNoise_H_Pre1;
            StochasticMatrix(1, 1) = -mParams.NeelNoise_H_Pre1;
            StochasticMatrix(2, 2) = -mParams.NeelNoise_H_Pre1;

            const auto yi2{ mParams.NeelNoise_H_Pre2*newmagnetisationdirection };
            // Upper right corner
            StochasticMatrix(0, 1) = -yi2(2);
            StochasticMatrix(0, 2) =  yi2(1);
            StochasticMatrix(1, 2) = -yi2(0);

            //Lower left corner
            StochasticMatrix(1, 0) =  yi2(2);
            StochasticMatrix(2, 0) = -yi2(1);
            StochasticMatrix(2, 1) =  yi2(0);

            StochasticMatrix = helper*StochasticMatrix;
            return StochasticMatrix;
        };

        template<typename Derived>
        BASIC_ALWAYS_INLINE auto getDrift(const BaseMatrixType<Derived>& yi) const
        {
            //TODO: calculate
            return DeterministicType::Zero();
        };

        template<typename Derived>
        BASIC_ALWAYS_INLINE void finishCalculations(BaseMatrixType<Derived>& yi) const
        {

        };

        
        template<typename Derived>
        BASIC_ALWAYS_INLINE void prepareJacobiCalculations(BaseMatrixType<Derived>& yi)
        {};

        template<typename Derived, typename Derived2>
        BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiDeterministic(const BaseMatrixType<Derived>& yi, const BaseMatrixType<Derived2>& xi, const Precision& dt) const
        {
            //Deterministc Jacobi Matrix
            const auto HeffJacobi{ mAnisotropy.getJacobiAnisotropyField(yi, mEasyAxis) };
            JacobiMatrixType JacobiDet{ JacobiMatrixType::Zero() };
            return JacobiDet;
        }

        BASIC_ALWAYS_INLINE JacobiMatrixType getJacobiStochastic(const NoiseType& dW) const
        {
            JacobiMatrixType JacobiSto{ JacobiMatrixType::Zero() };
            return JacobiSto;
        }

        template<typename Derived>
        BASIC_ALWAYS_INLINE void finishJacobiCalculations(BaseMatrixType<Derived>& jacobi) const
        {

        }
            

        template<typename Derived>
        BASIC_ALWAYS_INLINE OutputType calculateOutputResult(const BaseMatrixType<Derived>& yi) const noexcept
        {
            return static_cast<const Derived&>(yi);
        }

        auto getStart(const InitSettings& init) noexcept
        {
            std::random_device rd;
            std::normal_distribution<Precision> nd{ 0,1 };

            if (init.getUseRandomInitialMagnetisationDir())	{
                //Calculate random direction!
                for (unsigned int i = 0; i < magnetisationdirection.size(); ++i) {
                    magnetisationdirection(i) = nd(rd);
                }
            }
            else {
                //Use given direction!
                magnetisationdirection = init.getInitialMagnetisationDirection();
            }
            magnetisationdirection.normalize();

            return DependentType::Zero(); //We begin without any initial rotation. 
        }


        static auto getWeighting(const UsedProperties &Properties) noexcept
        {
            OutputType scale{ OutputType::Ones() };
            return (scale * Properties.getMagneticProperties().getSaturationMoment()).eval();
        };

    private:	
        template<typename Derived, typename Derived2>
        static BASIC_ALWAYS_INLINE void staticVectorChecks(const BaseMatrixType<Derived> &yi, const Derived2 &tester) noexcept
        {
            using ToTest = Derived;
            using TestType = Derived2;
            static_assert(std::is_same_v<typename ToTest::Scalar, typename TestType::Scalar>, "Matrix scalar types do not agree!");
            static_assert(ToTest::RowsAtCompileTime == TestType::RowsAtCompileTime, "Number of rows do not agree!");
            static_assert(ToTest::ColsAtCompileTime == TestType::ColsAtCompileTime, "Number of cols do not agree!");
        };

        ///-------------------------------------------------------------------------------------------------
        /// <summary>	Calculates the easy axis direction from some init settings. </summary>
        ///
        /// <param name="init">	The initialization settings. </param>
        ///
        /// <returns>	The calculated easy axis. </returns>
        ///-------------------------------------------------------------------------------------------------
        static IndependentType calcEasyAxis(const InitSettings& init) 
        {
            std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
            std::normal_distribution<precision> nd{ 0,1 };

            //Rotation of Coordinates (theta',phi') to (theta,phi) -90� around rotated y'-axis;
            if (init.getUseRandomInitialParticleOrientation())
            {
                IndependentType Orientation;
                for (typename IndependentType::Index i = 0; i < 3; ++i)
                {
                    Orientation(i) = nd(rd);
                }
                Orientation.normalize();
                return Orientation;
            }
            else
            {
                IndependentType EulerAngles = init.getInitialParticleOrientation();
                IndependentType Orientation(1, 0, 0);
                Matrix_3x3 tmp;
                const auto &a = EulerAngles[0]; //!< Alpha
                const auto &b = EulerAngles[1];	//!< Beta
                const auto &g = EulerAngles[2]; //!< Gamma
                tmp << cos(a)*cos(g) - sin(a)*cos(b)*sin(g), sin(a)*cos(g) + cos(a)*cos(b)*sin(g), sin(b)*sin(g),
                    -cos(a)*sin(g) - sin(a)*cos(b)*cos(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), sin(b)*cos(g),
                    sin(a)*sin(b), -cos(a)*sin(b), cos(b);
                Orientation = tmp*Orientation;
                return Orientation;
            }
        };



    };

}


#endif	// INC_NeelRelaxationQuaternion_H
// end of NeelRelaxationQuaternion.h
