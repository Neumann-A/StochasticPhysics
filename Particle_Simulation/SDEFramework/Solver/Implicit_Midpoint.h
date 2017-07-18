///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\Implicit_Midpoint.h
//
// summary: 	Declares the implicit midpoint SDE solver
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 14.07.2017

#ifndef INC_Implicit_Midpoint_H
#define INC_Implicit_Midpoint_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>
#include <limits>

#include "GeneralSDESolver.h"
#include "Settings/SolverSettings.h"

namespace SDE_Framework
{
	//Euler Maruyama uses Ito intepretation
	//converges with strong order 0.5 and weak order 1
	template<typename problem, typename nfield>
	class Implicit_Midpoint : public GeneralSDESolver<Implicit_Midpoint<problem, nfield>, problem, nfield>
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
		using IsExplicitSolver = typename  std::false_type;
		using IsImplicitSolver = typename  std::true_type;

		typedef typename problem::Dimension																			   Dimensions;
		typedef typename problem::DependentVectorType																   DependentVectorType;
		//typedef typename problem::IndependentVectorType															   IndependentVectorType;
		typedef typename problem::DeterministicVectorType															   DeterministicVectorType;
		typedef typename problem::StochasticMatrixType																   StochasticMatrixType;
		
		const std::size_t MaxIteration;
		const Precision   AccuracyGoal;
	public:

		BASIC_ALWAYS_INLINE Implicit_Midpoint(const Settings& SolverSet,const Problem &prob, Precision tstep);

		template<typename IndependentVectorFunctor>
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const Precision &totaltime,const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) const; // -> ResultType;

	};

}

#include "Implicit_Midpoint.inl"

#endif	// INC_Implicit_Midpoint_H
// end of SDEFramework\Solver\Implicit_Midpoint.h
///---------------------------------------------------------------------------------------------------