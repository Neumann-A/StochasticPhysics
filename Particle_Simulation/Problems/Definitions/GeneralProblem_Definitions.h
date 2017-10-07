///---------------------------------------------------------------------------------------------------
// file:		Problems\Definitions\General_Definitions.h
//
// summary: 	Declares helper functions and general definitions
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 05.10.2017

#ifndef INC_General_Definitions_H
#define INC_General_Definitions_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>

#include "Settings/ProblemSettings.h"

namespace Problems
{
	enum class CoordinateSystem { cartesian, spherical, polar };
}

namespace Selectors
{

}

namespace Problems
{
	namespace detail
	{
		template <Settings::IProblem problem>
		struct is_magnetic_problem : public std::false_type {};
		template <Settings::IProblem T>
		constexpr bool is_magnetic_problem_v = is_magnetic_problem<T>::value;

		template <Settings::IProblem problem>
		struct has_ito_noise : public std::false_type {};
		template <Settings::IProblem T>
		constexpr bool has_ito_noise_v = has_ito_noise<T>::value;

		template <Settings::IProblem problem>
		struct uses_boundaries : public std::false_type {};
		template <Settings::IProblem T>
		constexpr bool uses_boundaries_v = uses_boundaries<T>::value;

		template <Settings::IProblem problem>
		struct uses_magnetic_anisotropy : public std::false_type {};
		template <Settings::IProblem T>
		constexpr bool uses_magnetic_anisotropy_v = uses_magnetic_anisotropy<T>::value;
	}
}

#endif	// INC_General_Definitions_H
// end of Problems\Definitions\General_Definitions.h
