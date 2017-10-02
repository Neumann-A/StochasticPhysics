///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\EulerMaruyama_Normalized.h
//
// summary: 	Declares the euler maruyama normalized class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 02.10.2017

#ifndef INC_EulerMaruyama_Normalized_H
#define INC_EulerMaruyama_Normalized_H
///---------------------------------------------------------------------------------------------------
#pragma once


#include <type_traits>

#include "GeneralSDESolver.h"

#include "Settings/SolverSettings.h"
namespace SDE_Framework
{
	//Euler Maruyama uses Ito intepretation
	//converges with strong order 0.5 and weak order 1
	template<typename problem, typename nfield>
	class EulerMaruyama_Normalized : public GeneralSDESolver<EulerMaruyama_Normalized<problem, nfield>, problem, nfield>
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
		using IsExplicitSolver = typename  std::true_type;
		using IsImplicitSolver = typename  std::false_type;

		typedef typename problem::Dimension																			   Dimensions;
		typedef typename problem::DependentVectorType																   DependentVectorType;
		typedef typename problem::IndependentVectorType																   IndependentVectorType;
		typedef typename problem::DeterministicVectorType															   DeterministicVectorType;
		typedef typename problem::StochasticMatrixType																   StochasticMatrixType;

	private:
		template<typename IndependentVectorFunctor>
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestepIto(const Precision &time, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) const noexcept->ResultType;
		template<typename IndependentVectorFunctor>
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestepStratonovich(const Precision &time, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) const noexcept->ResultType;

	public:

		BASIC_ALWAYS_INLINE EulerMaruyama_Normalized(const Settings& SolverSettings, Problem &prob, Precision tstep);

		template<typename IndependentVectorFunctor>
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const Precision &time, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) const noexcept; // -> ResultType;

	};

}
#include "EulerMaruyama_Normalized.inl"

#endif	// INC_EulerMaruyama_Normalized_H
// end of SDEFramework\Solver\EulerMaruyama_Normalized.h
