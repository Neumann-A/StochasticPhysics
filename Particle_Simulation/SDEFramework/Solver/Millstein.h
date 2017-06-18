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

using namespace SDE_Framework;

namespace SDE_Framework
{

	//Euler Maruyama uses Ito intepretation
	template<typename problem, typename nfield>
	class Millstein : public GeneralSDESolver<Millstein<problem, nfield>, problem, nfield>
	{
	public:
		typedef typename problem::Precision																			   Precision;
		typedef	problem																								   Problem;
		typedef typename problem::DependentVectorType																   ResultType;

		using ResultTypeAllocator = typename Problem::Traits::DependentVectorStdAllocator;
		using NoiseField = nfield;

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
		BASIC_ALWAYS_INLINE Millstein(const problem &prob, Precision tstep);
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept; // -> ResultType;
	};
}
#include "Millstein.inl"

#endif	// INC_Millstein_H
// end of SDEFramework\Solver\Millstein.h
///---------------------------------------------------------------------------------------------------
