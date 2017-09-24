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

#include "../Basic_Library/math/GSL_Implicit_Solver.h"

namespace SDE_Framework
{
	//Euler Maruyama uses Ito intepretation
	//converges with strong order 0.5 and weak order 1
	template<typename problem, typename nfield>
	class Implicit_Midpoint_GSL : public GeneralSDESolver<Implicit_Midpoint_GSL<problem, nfield>, problem, nfield>
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

		GSL_Implicit_Solver<Precision> mSolver;
	public:

		Implicit_Midpoint_GSL(const Settings& SolverSet, Problem &prob, Precision tstep);

		template<typename IndependentVectorFunctor>
		auto getResultNextFixedTimestep(const Precision &totaltime, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc); // -> ResultType;

	};

}

#include "Implicit_Midpoint_GSL.inl"

#endif	// INC_Implicit_Midpoint_GSL_H
// end of SDEFramework\Solver\Implicit_Midpoint_GSL.h
///---------------------------------------------------------------------------------------------------
