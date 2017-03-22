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
			Precision NeelFactor1{ 0 }; // One Prefactor from the LLG
			Precision NeelFactor2{ 0 }; // The other Prefactor from the LLG
			Precision DriftPrefactor{ 0 }; // The Prefactor needed for the conversion from Ito to Stratonovich SDE
			Precision NeelNoise_H_Pre1{ 0 }; // Actual Noise Prefactor in the SDE
			Precision NeelNoise_H_Pre2{ 0 }; // Actual Noise Prefactor2 in the SDE
		};

		template<typename precision>
		class NeelCalculator
		{
		public:
			static constexpr const precision kB = 1.3806485279E-23; // Boltzman constant;
			static auto calcNeelParams(const Properties::MagneticProperties<precision>& MagProps, const precision& Temperature)
			{
				NeelParams<precision> Params;

				Params.NeelFactor1 = -MagProps.getGyromagneticRatio() / (1 + pow(MagProps.getDampingConstant(), 2));
				Params.NeelFactor2 = Params.NeelFactor1*MagProps.getDampingConstant();
				
				const precision diffsquare =2 * MagProps.getDampingConstant()*kB*Temperature / (MagProps.getGyromagneticRatio()*MagProps.getSaturationMoment());
				const precision diff = std::sqrt(diffsquare);

				Params.DriftPrefactor = (Params.NeelFactor2*Params.NeelFactor2- Params.NeelFactor1*Params.NeelFactor1)*diffsquare;
				Params.NeelNoise_H_Pre1 = Params.NeelFactor1 * diff;
				Params.NeelNoise_H_Pre2 = Params.NeelFactor2 * diff;

				return Params;
			}
		};

	}
}

#endif	// INC_ParameterCalculatorNeelh_H
// end of ParameterCalculatorNeelh.h
///---------------------------------------------------------------------------------------------------
