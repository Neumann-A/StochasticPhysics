///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\Implicit_Midpoint_GSL_GSL.h
//
// summary: 	Declares the implicit midpoint SDE solver using the GSL
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 14.07.2017

#ifndef INC_Implicit_Midpoint_GSL_H
#define INC_Implicit_Midpoint_GSL_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>
#include <limits>

#include "GeneralSDESolver.h"
#include "Settings/SolverSettings.h"

#include <MyCEL/math/GSL_Implicit_Solver.h>

namespace SDE_Framework::Solvers
{
	//Euler Maruyama uses Ito intepretation
	//converges with strong order 0.5 and weak order 1
	template<typename problem, typename nfield>
	class Implicit_Midpoint_GSL : public GeneralSDESolver<Implicit_Midpoint_GSL<problem, nfield>, problem, nfield>
	{
		template<bool IsIto>
		friend struct detail::FixedTimestepSelector;
	public:
		using Problem = problem;
		using Precision = typename Problem::Traits::Precision;

		using ResultType = typename Problem::Traits::DependentType;

		using NoiseField = nfield;
		using Settings = Settings::SolverSettings<Precision>;
	private:
		using IsIto = typename Problems::SDEProblem_Traits<problem>::IsIto;

		using DependentType = typename Problem::DependentType;
		using IndependentType = typename Problem::IndependentType;
		using DeterministicType = typename Problem::DeterministicType;
		using StochasticMatrixType = typename Problem::StochasticMatrixType;

		const std::size_t MaxIteration;
		const Precision   AccuracyGoal;

		GSL_Implicit_Solver<Precision> mSolver;
	public:

		Implicit_Midpoint_GSL(const Settings& SolverSet, Problem &prob, Precision tstep);

		template<typename IndependentFunctor>
		auto getResultNextFixedTimestep(const Precision &totaltime, const DependentType &yi, const IndependentFunctor &xifunc); // -> ResultType;

	};

	namespace detail
	{
		template<typename problem, typename nfield>
		struct is_implicit_solver<Implicit_Midpoint_GSL<problem, nfield>> : std::true_type {};
	}

}

#include "Implicit_Midpoint_GSL.inl"

#endif	// INC_Implicit_Midpoint_GSL_H
// end of SDEFramework\Solver\Implicit_Midpoint_GSL.h
///---------------------------------------------------------------------------------------------------
