///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\Millstein.h
//
// summary: 	Declares the millstein class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 14.06.2017

#ifndef INC_Millstein_H
#define INC_Millstein_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "GeneralSDESolver.h"
#include "Settings/SolverSettings.h"

namespace SDE_Framework::Solvers
{
	//Euler Maruyama uses Ito intepretation
	template<typename problem, typename nfield>
	class Millstein : public GeneralSDESolver<Millstein<problem, nfield>, problem, nfield>
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

	private:
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestepIto(const DependentType& yi, const IndependentType& xi) const noexcept->ResultType;
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestepStratonovich(const DependentType& yi, const IndependentType& xi) const noexcept->ResultType;

	public:
		BASIC_ALWAYS_INLINE Millstein(const Settings& SolverSettings, Problem &prob, Precision tstep);
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const DependentType &yi, const IndependentType &xi) const noexcept; // -> ResultType;
	};
}
#include "Millstein.inl"

#endif	// INC_Millstein_H
// end of SDEFramework\Solver\Millstein.h
///---------------------------------------------------------------------------------------------------
