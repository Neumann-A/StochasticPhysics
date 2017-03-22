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
			precision NeelPre2PlusMixPre;
			//precision Neel_F_Noise; //Same as BrownDiff;
		};

		template <typename precision>
		struct BrownAndNeelMixedDriftFull
		{
			precision b_2;
			precision bc_div_2;
			precision a_b_half;
		};

		template <typename precision>
		struct BrownAndNeelMixedDriftSimplified
		{
			precision half_min_a_2;
			precision half_min_b_2_min_c_2_plus_d_2;
		};

		template <typename precision>
		struct BrownAndNeelMixedDriftMatrix
		{
			SubMatrix<precision> order1a;
			SubMatrix<precision> order1b;
			SubMatrix<precision> order2a;
		};

		template <typename precision>
		struct BrownAndNeelMixedDriftHelper
		{
			precision min_a_2;
			precision b_2;
			precision min_b_2_min_c_2_plus_d_2;
			precision bc_div_2;
			precision min_a_2_min_half;
			precision min_b_2_min_c_2_plus_d_2_min_b2;
			precision a_b_half_min_b2_half;
			precision b_d;
			precision b_c;
			precision a_b_half;
		};

		template<typename precision>
		class BrownAndNeelRotationCalculator
		{
		public:
			static constexpr const precision kB = 1.3806485279E-23; // Boltzman constant;

			static auto calcBrownNeelMixed(const Properties::ParticlesProperties<precision>& props, const BrownRotationParams<precision> &brown, const NeelParams<precision> &neel)
			{
				BrownAndNeelMixed<precision> param;
				param.Brown_H_Noise = props.getMagneticProperties().getSaturationMoment()*brown.BrownPrefactor*(neel.NeelNoise_H_Pre1 / neel.NeelFactor1); //(The last term is neel diffusion term)
				param.NeelBrownMixPre = props.getMagneticProperties().getSaturationMoment() * brown.BrownPrefactor;
				param.NeelPre2PlusMixPre = param.NeelBrownMixPre +neel.NeelFactor2 ;
				return param;
			};
		};

		template <typename precision>
		class BrownAndNeelMixedParams
		{
		private:

			using Precision = precision;

			BrownRotationParams<Precision>				BrownParams;
			NeelParams<Precision>						NeelParams;
			BrownAndNeelMixed<Precision>				BrownAndNeel;
			BrownAndNeelMixedDriftSimplified<Precision> BrownAndNeelDriftSimplified;
			BrownAndNeelMixedDriftFull<Precision>		BrownAndNeelDriftFull;
			BrownAndNeelMixedDriftMatrix<Precision>		BrownAndNeelDriftMatrix;


		private:
			BASIC_ALWAYS_INLINE void calcBrownNeelDriftHelper() BASIC_NOEXCEPT
			{
				BrownAndNeelMixedDriftHelper<precision> helper;

				//Helpers
				const Precision a_2 = pow(Brown_F_Noise(), 2);
				const Precision c_2 = pow(NeelPre2_H_Noise(), 2);
				const Precision d_2 = pow(NeelPre1_H_Noise(), 2);

				//These Values have been obtained by Mathematica using the LLG + Brown SDE; They describe the needed drift correction from ito to stratonovich!
				 
				//it is 1/2*o_ik do_jk/d_xk. Factor 1/2 is last step in drift calculation
				helper.min_a_2 = -1.0 * a_2; //-a^2
				helper.b_2 = 1.0 * pow(Brown_H_Noise(), 2); // b^2
				helper.min_b_2_min_c_2_plus_d_2 = -1.0 * (helper.b_2 - c_2 + d_2); //-(b^2-c^2+d^2)
				helper.bc_div_2 = 0.5*Brown_H_Noise()*NeelPre2_H_Noise(); //1/2* bc
				helper.min_a_2_min_half = helper.min_a_2 - 0.5; //-(a^2+0.5)
				helper.min_b_2_min_c_2_plus_d_2_min_b2 = -1.0 * (helper.b_2 - c_2 + d_2 + 0.5*helper.b_2);//(b^2-c^2+d^2+b^2/2)
				helper.a_b_half_min_b2_half = 0.5*(Brown_F_Noise()*Brown_H_Noise() - helper.b_2);
				helper.b_d = 1.0 * Brown_H_Noise()*NeelPre1_H_Noise(); // b*d
				helper.b_c = 1.0 * Brown_H_Noise()*NeelPre2_H_Noise();
				helper.a_b_half = 0.5*(Brown_F_Noise()*Brown_H_Noise());
				
				//Drift Factors;
				BrownAndNeelDriftSimplified.half_min_a_2 = 0.5*helper.min_a_2;
				BrownAndNeelDriftSimplified.half_min_b_2_min_c_2_plus_d_2 = 0.5*helper.min_b_2_min_c_2_plus_d_2_min_b2;

				BrownAndNeelDriftFull.a_b_half = helper.a_b_half;
				BrownAndNeelDriftFull.bc_div_2 = helper.bc_div_2;
				BrownAndNeelDriftFull.b_2 = helper.b_2;
				
				//Order2a
				BrownAndNeelDriftMatrix.order2a << -1, 0.5, -0.5,
					-0.5, 1, -0.5,
					-0.5, 0.5, -1;

				BrownAndNeelDriftMatrix.order2a *= helper.b_d;

				//Order1b
				auto& tmp1b = BrownAndNeelDriftMatrix.order1b;
				tmp1b << helper.min_b_2_min_c_2_plus_d_2, helper.a_b_half_min_b2_half, helper.a_b_half_min_b2_half,
					helper.b_2, helper.min_b_2_min_c_2_plus_d_2_min_b2, helper.a_b_half_min_b2_half,
					helper.b_2, helper.a_b_half_min_b2_half, helper.min_b_2_min_c_2_plus_d_2_min_b2;

				//Order1a
				auto& tmp1a = BrownAndNeelDriftMatrix.order1a;
				tmp1a << helper.min_a_2, helper.bc_div_2, helper.bc_div_2,
					helper.b_2, helper.min_a_2_min_half, helper.bc_div_2,
					helper.b_2, helper.bc_div_2, helper.min_a_2_min_half;

				return;
			};

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			//Hopefully the compiler will optimize all those accessors away!
			BASIC_ALWAYS_INLINE const auto& BrownPrefactor() const BASIC_NOEXCEPT { return BrownParams.BrownPrefactor; }
			BASIC_ALWAYS_INLINE const auto& BrownDiffusion() const BASIC_NOEXCEPT { return BrownParams.BrownDiffusion; }
			BASIC_ALWAYS_INLINE const auto& NeelPrefactor1() const BASIC_NOEXCEPT { return NeelParams.NeelFactor1; }
			BASIC_ALWAYS_INLINE const auto& NeelPrefactor2() const BASIC_NOEXCEPT { return NeelParams.NeelFactor2; }
			BASIC_ALWAYS_INLINE const auto& NeelPre1_H_Noise() const BASIC_NOEXCEPT { return NeelParams.NeelNoise_H_Pre1; }
			BASIC_ALWAYS_INLINE const auto& NeelPre2_H_Noise() const BASIC_NOEXCEPT { return NeelParams.NeelNoise_H_Pre2; }
			BASIC_ALWAYS_INLINE const auto& Brown_F_Noise() const BASIC_NOEXCEPT { return BrownParams.BrownDiffusion; }
			BASIC_ALWAYS_INLINE const auto& Neel_F_Noise() const BASIC_NOEXCEPT { return BrownParams.BrownDiffusion; }
			BASIC_ALWAYS_INLINE const auto& Brown_H_Noise() const BASIC_NOEXCEPT { return BrownAndNeel.Brown_H_Noise; }
			BASIC_ALWAYS_INLINE const auto& NeelBrownMixPre() const BASIC_NOEXCEPT { return BrownAndNeel.NeelBrownMixPre; }
			BASIC_ALWAYS_INLINE const auto& NeelPre2PlusMixPre() const BASIC_NOEXCEPT { return BrownAndNeel.NeelPre2PlusMixPre; }
			BASIC_ALWAYS_INLINE const auto& order1a() const BASIC_NOEXCEPT { return BrownAndNeelDriftMatrix.order1a; }
			BASIC_ALWAYS_INLINE const auto& order1b() const BASIC_NOEXCEPT { return BrownAndNeelDriftMatrix.order1b; }
			BASIC_ALWAYS_INLINE const auto& order2a() const BASIC_NOEXCEPT { return BrownAndNeelDriftMatrix.order2a; }
			BASIC_ALWAYS_INLINE const auto& half_min_a_2() const BASIC_NOEXCEPT { return BrownAndNeelDriftSimplified.half_min_a_2; }
			BASIC_ALWAYS_INLINE const auto& half_min_b_2_min_c_2_plus_d_2() const BASIC_NOEXCEPT { return BrownAndNeelDriftSimplified.half_min_b_2_min_c_2_plus_d_2; }
			BASIC_ALWAYS_INLINE const auto& b_2() const BASIC_NOEXCEPT { return BrownAndNeelDriftFull.b_2; }
			BASIC_ALWAYS_INLINE const auto& bc_div_2() const BASIC_NOEXCEPT { return BrownAndNeelDriftFull.bc_div_2; }
			BASIC_ALWAYS_INLINE const auto& a_b_half() const BASIC_NOEXCEPT { return BrownAndNeelDriftFull.a_b_half; }

			BrownAndNeelMixedParams(const Properties::ParticlesProperties<precision>& Props)
				: BrownParams(BrownianRotationCalculator<precision>::calcBrownRotationParams(Props.getHydrodynamicProperties(), Props.getTemperature())),
				NeelParams(NeelCalculator<precision>::calcNeelParams(Props.getMagneticProperties(), Props.getTemperature())), 
				BrownAndNeel(BrownAndNeelRotationCalculator<precision>::calcBrownNeelMixed(Props,BrownParams,NeelParams))
			{
				calcBrownNeelDriftHelper();
			};
		};



	}
}

#endif	// INC_ParameterCalculatorBrownAndNeel_H
// end of ParameterCalculatorBrownAndNeel.h
///---------------------------------------------------------------------------------------------------
