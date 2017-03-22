///---------------------------------------------------------------------------------------------------
// file:		ParameterCalculatorBrown.h
//
// summary: 	Declares the parameter calculator brown class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 02.02.2017

#ifndef INC_ParameterCalculatorBrown_H
#define INC_ParameterCalculatorBrown_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <cmath>

#include "Properties/HydrodynamicProperties.h"

namespace Problems
{
	namespace Helpers
	{
		template <typename prec>
		struct BrownRotationParams
		{
			using Precision = prec;
		public:
			Precision BrownPrefactor;
			Precision BrownDiffusion; //Same as BrownNoise
		};

		template<typename precision>
		class BrownianRotationCalculator
		{
		public:
			static constexpr const precision kB = 1.3806485279E-23; // Boltzman constant;
			static BrownRotationParams<precision> calcBrownRotationParams(const Properties::HydrodynamicProperties<precision> &BrownProps, const precision& Temperature)
			{
				BrownRotationParams<precision> Param;

				Param.BrownPrefactor = -1 / (6 * BrownProps.getViscosity() * BrownProps.getHydrodynamicVolume());
				Param.BrownDiffusion = std::sqrt(2 * kB*Temperature / (6 * BrownProps.getViscosity() * BrownProps.getHydrodynamicVolume()));
				
				return Param;
			}
		};

		template<typename precision>
		class BrownianMotionCalculator
		{

		};
	}
}

#endif	// INC_ParameterCalculatorBrown_H
// end of ParameterCalculatorBrown.h
///---------------------------------------------------------------------------------------------------
