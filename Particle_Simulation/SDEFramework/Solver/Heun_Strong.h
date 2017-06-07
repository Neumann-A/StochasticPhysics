///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\Heun_Strong.h
//
// summary: 	Declares the heun strong class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 07.06.2017

#ifndef INC_Heun_Strong_H
#define INC_Heun_Strong_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "GeneralSDESolver.h"

//Heun Strong uses Ito intepretation
//See Numerical Solution of stochastic differential equations
template<typename problem, typename nfield>
class Heun_Strong : public GeneralSDESolver<Heun_Strong<problem, nfield>, problem, nfield>
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
	ResultType(Heun_Strong<problem, nfield>::*toResultFixedTimestep)(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept = nullptr;
	inline auto getResultNextFixedTimestepIto(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept->ResultType;
	inline auto getResultNextFixedTimestepStratonovich(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept->ResultType;

public:

	inline Heun_Strong(const problem &prob, Precision tstep);

	inline auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept; // -> ResultType;

};

#include "Heun_Strong.inl"



#endif	// INC_Heun_Strong_H
// end of SDEFramework\Solver\Heun_Strong.h
///---------------------------------------------------------------------------------------------------
