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

//Euler Maruyama uses Ito intepretation
//converges with strong order 0.5 and weak order 1
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

	typedef typename problem::Dimension																			   Dimensions;
	typedef typename problem::DependentVectorType																   DependentVectorType;
	typedef typename problem::IndependentVectorType																   IndependentVectorType;
	typedef typename problem::DeterministicVectorType															   DeterministicVectorType;
	typedef typename problem::StochasticMatrixType																   StochasticMatrixType;

private:
	//TODO: Get rid of this function pointer by static analyses of problem
	ResultType(Millstein<problem, nfield>::*toResultFixedTimestep)(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept = nullptr;
	inline auto getResultNextFixedTimestepIto(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept->ResultType;
	inline auto getResultNextFixedTimestepStratonovich(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept->ResultType;

public:

	inline Millstein(const problem &prob, Precision tstep);

	inline auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept; // -> ResultType;

};

#include "Millstein.inl"

#endif	// INC_Millstein_H
// end of SDEFramework\Solver\Millstein.h
///---------------------------------------------------------------------------------------------------
