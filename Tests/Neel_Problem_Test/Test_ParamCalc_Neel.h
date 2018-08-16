///---------------------------------------------------------------------------------------------------
// file:		Test_ParamCalc_Neel.h
//
// summary: 	Declares the test parameter calculate neel class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Neumann
// date: 21.07.2017

#ifndef INC_Test_ParamCalc_Neel_H
#define INC_Test_ParamCalc_Neel_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <gtest/gtest.h>

#include "Problems/Helpers/ParameterCalculatorNeel.h"

namespace
{
	using namespace ::Problems::Helpers;

	class ParamCalcNeelTest : public ::testing::Test
	{
	public:
		static constexpr const ::Properties::IAnisotropy sAni = ::Properties::IAnisotropy::Anisotropy_uniaxial;

		using Precision = double;
		using TestParams = NeelParams<Precision>;
		using TestProperties = typename ::Properties::MagneticProperties<Precision>;

		TestProperties mProp;

		ParamCalcNeelTest() 
		{
			//Magnetic Parameters
			const Precision damping = 0.1;
			const Precision gyro = 1.76E+11;
			const Precision Ms = 477464;
			const Precision rmag = 10E-9;
			const Precision KUni = 10E4;

			mProp = TestProperties{ rmag,Ms,damping,gyro,sAni,std::vector<Precision>{ {KUni,0.0} } };
		};

		static constexpr const Precision kB = 1.3806485279E-23; // Boltzman constant;
	};
};


#endif	// INC_Test_ParamCalc_Neel_H
// end of Test_ParamCalc_Neel.h
///---------------------------------------------------------------------------------------------------
