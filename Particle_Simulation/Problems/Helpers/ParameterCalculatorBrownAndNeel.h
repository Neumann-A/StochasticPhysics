///---------------------------------------------------------------------------------------------------
// file:		ParameterCalculatorBrownAndNeel.h
//
// summary: 	Declares the parameter calculator brown and neel class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 05.02.2017

#ifndef INC_ParameterCalculatorBrownAndNeel_H
#define INC_ParameterCalculatorBrownAndNeel_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "basics/BasicMacros.h"

#include <Eigen/Core>

#include "Properties/ParticleProperties.h"

#include "ParameterCalculatorNeel.h"
#include "ParameterCalculatorBrown.h"

//#include "BrownAndNeel_Definitions.h"

namespace Problems
{
	namespace Helpers
	{
		template <typename prec>
		using SubMatrix = Eigen::Matrix<prec, 3, 3>;
		
		template <typename precision>
		struct BrownAndNeelMixed
		{
			precision NeelBrownMixPre;
			precision Brown_H_Noise;
			precision Neel_F_Noise;
			//precision NeelPre2PlusMixPre;
			//precision Neel_F_Noise; //Same as BrownDiff;
		};

		//template <typename precision>
		//struct BrownAndNeelMixedDriftSimplified
		//{
		//	precision min_a_2;  //Brown Rotation Drift (- a^2)
		//	precision min__c_2_plus__b_plus_d__2;  //Neel Rotation Drift ( - (c^2+(b+d)^2))
		//};

		//template <typename precision>
		//struct BrownAndNeelMixedDriftFull
		//{
		//	SubMatrix<precision> order1a;
		//	SubMatrix<precision> order1b;
		//	SubMatrix<precision> order2a;
		//	SubMatrix<precision> order3a;
		//	SubMatrix<precision> order3b;
		//};

		template<typename precision>
		class BrownAndNeelRotationCalculator
		{
		public:
			static constexpr const precision kB = 1.3806485279E-23; // Boltzman constant;

			static auto calcBrownNeelMixed(const Properties::ParticlesProperties<precision>& props, const BrownRotationParams<precision> &brown, const NeelParams<precision> &neel)
			{
				BrownAndNeelMixed<precision> param;
				param.NeelBrownMixPre = props.getMagneticProperties().getSaturationMoment()*brown.BrownPrefactor;
				param.Brown_H_Noise = param.NeelBrownMixPre*neel.NoisePrefactor; //(The last term is neel diffusion term)
				param.Neel_F_Noise = param.NeelBrownMixPre*brown.BrownDiffusion;
				//param.NeelPre2PlusMixPre = param.NeelBrownMixPre + neel.NeelFactor2 ;
				return param;
			}
		};

		template <typename precision>
		class BrownAndNeelMixedParams
		{
		private:

			using Precision = precision;

			BrownRotationParams<Precision>				BrownParams;
			NeelParams<Precision>						NeelParams;
			BrownAndNeelMixed<Precision>				BrownAndNeel;
			//BrownAndNeelMixedDriftSimplified<Precision> BrownAndNeelDriftSimplified;
			//BrownAndNeelMixedDriftFull<Precision>		BrownAndNeelDriftFull;

			using Matrix3x3 = SubMatrix<precision>;
		private:
			BASIC_ALWAYS_INLINE void calcBrownNeelDriftHelper() BASIC_NOEXCEPT
			{
				////Helpers
				//const Precision a = Brown_F_Noise();
				//const Precision a_2 = pow(a, 2);
				//const Precision b = Brown_H_Noise();
				//const Precision b_2 = pow(b, 2);

				//const Precision c = NeelPre2_H_Noise();
				//const Precision c_2 = pow(c, 2);
				//const Precision d = NeelPre1_H_Noise();

				//const Precision e = Neel_F_Noise();
				//const Precision e_2 = pow(e, 2);

				//const Precision b_plus_d__2 = pow(Brown_H_Noise()+NeelPre1_H_Noise(),2);
				//const Precision b_d = b*d;

				////These Values have been obtained by Mathematica using the LLG + Brown SDE;
				////They describe the needed drift correction from ito to stratonovich! 
				////(And can also be used for the millstein scheme)
				//	
				////Drift Factors;
				//BrownAndNeelDriftSimplified.min_a_2 = -1.0 * a_2;
				//BrownAndNeelDriftSimplified.min__c_2_plus__b_plus_d__2 = -(c_2 + b_plus_d__2);

				////Order1a (Brown)
				//BrownAndNeelDriftFull.order1a   << -a_2,  b_2,  b_2,
				//								    b_2, -a_2,  b_2,
				//								    b_2,  b_2, -a_2;

				////Order2a (Brown)
				//BrownAndNeelDriftFull.order2a   << 1.0, 0.5, 0.5,
				//								   0.5, 1.0, 0.5,
				//								   0.5, 0.5, 1.0;

				//BrownAndNeelDriftFull.order2a *= b*c;
				//
				////Order3a (Brown)
				//{
				//	auto tmp1 = 0.5*b_d*(Matrix3x3::Constant(1.0) - Matrix3x3::Identity());
				//	auto tmp2 = -1.0* b_2 * (Matrix3x3::Constant(1.0) - 0.5*Matrix3x3::Identity());
				//	BrownAndNeelDriftFull.order3a = (tmp1 + tmp2).eval();
				//}

				////Order1b (Neel)
				//BrownAndNeelDriftFull.order1b << -(c_2 + b_plus_d__2), e_2, e_2,
				//									e_2, -(c_2 + b_plus_d__2), e_2,
				//									e_2, e_2, -(c_2 + b_plus_d__2);

				////Order3b (Neel)
				//{
				//	auto tmp1 = -0.5*a*e*(Matrix3x3::Constant(1.0) - Matrix3x3::Identity());
				//	auto tmp2 = 0.5*e_2* (Matrix3x3::Constant(3.0) - 2 * Matrix3x3::Identity());
				//	BrownAndNeelDriftFull.order3b = (tmp1 + tmp2).eval();
				//}

				return;
			}

		public:
			////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			//Hopefully the compiler will optimize all those accessors away!
			BASIC_ALWAYS_INLINE const auto& BrownPrefactor() const BASIC_NOEXCEPT { return BrownParams.BrownPrefactor; }
			BASIC_ALWAYS_INLINE const auto& BrownDiffusion() const BASIC_NOEXCEPT { return BrownParams.BrownDiffusion; }
			BASIC_ALWAYS_INLINE const auto& NeelPrefactor1() const BASIC_NOEXCEPT { return NeelParams.NeelFactor1; }
			BASIC_ALWAYS_INLINE const auto& NeelPrefactor2() const BASIC_NOEXCEPT { return NeelParams.NeelFactor2; }
			//BASIC_ALWAYS_INLINE const auto& NeelPre1_H_Noise() const BASIC_NOEXCEPT { return NeelParams.NeelNoise_H_Pre1; }
			//BASIC_ALWAYS_INLINE const auto& NeelPre2_H_Noise() const BASIC_NOEXCEPT { return NeelParams.NeelNoise_H_Pre2; }
			//BASIC_ALWAYS_INLINE const auto& Brown_F_Noise() const BASIC_NOEXCEPT { return BrownParams.Brown_F_Noise; }
			//BASIC_ALWAYS_INLINE const auto& Neel_F_Noise() const BASIC_NOEXCEPT { return BrownAndNeel.Neel_F_Noise; }
			//BASIC_ALWAYS_INLINE const auto& Brown_H_Noise() const BASIC_NOEXCEPT { return BrownAndNeel.Brown_H_Noise; }
			//BASIC_ALWAYS_INLINE const auto& NeelBrownMixPre() const BASIC_NOEXCEPT { return BrownAndNeel.NeelBrownMixPre; }
			//BASIC_ALWAYS_INLINE const auto& NeelPre2PlusMixPre() const BASIC_NOEXCEPT { return BrownAndNeel.NeelPre2PlusMixPre; }
			//BASIC_ALWAYS_INLINE const auto& order1a() const BASIC_NOEXCEPT { return BrownAndNeelDriftFull.order1a; }
			//BASIC_ALWAYS_INLINE const auto& order1b() const BASIC_NOEXCEPT { return BrownAndNeelDriftFull.order1b; }
			//BASIC_ALWAYS_INLINE const auto& order2a() const BASIC_NOEXCEPT { return BrownAndNeelDriftFull.order2a; }
			//BASIC_ALWAYS_INLINE const auto& order3a() const BASIC_NOEXCEPT { return BrownAndNeelDriftFull.order3a; }
			//BASIC_ALWAYS_INLINE const auto& order3b() const BASIC_NOEXCEPT { return BrownAndNeelDriftFull.order3b; }
			//BASIC_ALWAYS_INLINE const auto& min_a_2() const BASIC_NOEXCEPT { return BrownAndNeelDriftSimplified.min_a_2; }
			//BASIC_ALWAYS_INLINE const auto& min__c_2_plus__b_plus_d__2() const BASIC_NOEXCEPT { return BrownAndNeelDriftSimplified.min__c_2_plus__b_plus_d__2; }


			BrownAndNeelMixedParams(const Properties::ParticlesProperties<precision>& Props)
				: BrownParams(BrownianRotationCalculator<precision>::calcBrownRotationParams(Props.getHydrodynamicProperties(), Props.getTemperature())),
				NeelParams(NeelCalculator<precision>::calcNeelParams(Props.getMagneticProperties(), Props.getTemperature())), 
				BrownAndNeel(BrownAndNeelRotationCalculator<precision>::calcBrownNeelMixed(Props,BrownParams,NeelParams))
			{
				calcBrownNeelDriftHelper();
			}
		};



	}
}

#endif	// INC_ParameterCalculatorBrownAndNeel_H
// end of ParameterCalculatorBrownAndNeel.h
///---------------------------------------------------------------------------------------------------
