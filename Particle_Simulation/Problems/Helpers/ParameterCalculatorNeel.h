///---------------------------------------------------------------------------------------------------
// file:		ParameterCalculatorNeel.h
//
// summary: 	Declares the parameter calculator neel class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 02.02.2017

#ifndef INC_ParameterCalculatorNeel_H
#define INC_ParameterCalculatorNeel_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <cmath>

#include "Properties/MagneticProperties.h"

namespace Problems
{
	namespace Helpers
	{
		template<typename prec>
		struct NeelParams
		{
			using Precision = prec;
		public:
			Precision NeelFactor1{ 0.0 }; // One Prefactor from the LLG
			Precision NeelFactor2{ 0.0 }; // The other Prefactor from the LLG
			Precision DriftPrefactor{ 0.0 }; // The Prefactor needed for the conversion from Ito to Stratonovich SDE
			Precision NeelNoise_H_Pre1{ 0.0 }; // Actual Noise Prefactor in the SDE
			Precision NeelNoise_H_Pre2{ 0.0 }; // Actual Noise Prefactor2 in the SDE
			//Precision min_e_2 { 0.0 };
			Precision NoisePrefactor{ 0.0 }; // The Prefactor needed for the conversion from Ito to Stratonovich SDE
			Precision Damping { 0.0 };
		};

		template<typename precision>
		class NeelCalculator
		{
		public:
			static constexpr const precision kB = 1.3806485279E-23; // Boltzman constant;
			static auto calcNeelParams(const Properties::MagneticProperties<precision>& MagProps, const precision& Temperature)
			{
				NeelParams<precision> Params;

				const auto& alpha = MagProps.getDampingConstant();

				// we assume to gyromagnetic ratio to be positiv for an electron!
				if (std::isinf(alpha))
				{ 
					Params.NeelFactor1 = 0.0;
					Params.NeelFactor2 = 0.0;
				}
				else
				{
					Params.NeelFactor1 = -MagProps.getGyromagneticRatio() / (1.0 + std::pow(alpha, 2));
					Params.NeelFactor2 = std::abs(Params.NeelFactor1)*alpha;
				}
				precision diffsquare =2.0 * alpha*kB*Temperature / (MagProps.getGyromagneticRatio()*MagProps.getSaturationMoment());

				if (std::isinf(diffsquare) || std::isnan(diffsquare))
				{
					//Overdamped -> No magnetic movement -> no thermal diffusion
					diffsquare = 0;
				}

				const precision diff = std::sqrt(diffsquare);

				//Params.DriftPrefactor = (Params.NeelFactor2*Params.NeelFactor2- Params.NeelFactor1*Params.NeelFactor1)*diffsquare;
				Params.DriftPrefactor = -1.0 * std::pow(Params.NeelFactor1,2) * (1.0 + std::pow(alpha,2))*diffsquare;
				Params.NeelNoise_H_Pre1 = Params.NeelFactor1 * diff;
				Params.NeelNoise_H_Pre2 = Params.NeelFactor2 * diff;
				Params.NoisePrefactor = diff;
				Params.Damping = alpha;
				//Params.Damping_2 = std::pow(MagProps.getDampingConstant(), 2);
				//Params.min_e_2 = -1.0 * std::pow(Params.NeelFactor1, 2);

				return Params;
			}
		};

	}
}

#endif	// INC_ParameterCalculatorNeelh_H
// end of ParameterCalculatorNeelh.h
///---------------------------------------------------------------------------------------------------
