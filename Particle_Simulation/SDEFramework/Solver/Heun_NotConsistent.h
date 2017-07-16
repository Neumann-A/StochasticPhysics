///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\Heun_NotConsistent.h
//
// summary: 	Declares the heun not consistent class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 07.06.2017

#ifndef INC_Heun_NotConsistent_H
#define INC_Heun_NotConsistent_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "GeneralSDESolver.h"

#include "Settings/SolverSettings.h"

namespace SDE_Framework
{
	//Heun Not consistent uses Ito intepretation; It is not strongly consistent meaning it will not necessary decrease the error with decresed step size
	//See Numerical Solution of stochastic differential equations
	template<typename problem, typename nfield>
	class Heun_NotConsistent : public GeneralSDESolver<Heun_NotConsistent<problem, nfield>, problem, nfield>
	{
		template<bool IsIto>
		friend struct detail::FixedTimestepSelector;
	public:
		typedef typename problem::Precision																			   Precision;
		typedef	problem																								   Problem;
		typedef typename problem::DependentVectorType																   ResultType;

		using ResultTypeAllocator = typename Problem::Traits::DependentVectorStdAllocator;
		using NoiseField = nfield;
		using Settings = Settings::SolverSettings<Precision>;
	private:
		using IsIto = typename Problems::SDEProblem_Traits<problem>::IsIto;

		typedef typename problem::Dimension																			   Dimensions;
		typedef typename problem::DependentVectorType																   DependentVectorType;
		typedef typename problem::IndependentVectorType																   IndependentVectorType;
		typedef typename problem::DeterministicVectorType															   DeterministicVectorType;
		typedef typename problem::StochasticMatrixType																   StochasticMatrixType;

	private:
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestepIto(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept->ResultType;
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestepStratonovich(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept->ResultType;

	public:
		BASIC_ALWAYS_INLINE Heun_NotConsistent(const Settings& SolverSettings, const Problem &prob, Precision tstep);
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept; // -> ResultType;

	};
}
#include "Heun_NotConsistent.inl"

#endif	// INC_Heun_NotConsistent_H
// end of SDEFramework\Solver\Heun_NotConsistent.h
///---------------------------------------------------------------------------------------------------
