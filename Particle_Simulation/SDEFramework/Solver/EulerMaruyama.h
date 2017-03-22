/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

/*SDE Solver Euler Maruyama see Kloeden and Platten Numerical methods for further information!*/

#pragma once

#ifndef _EulerMaruyama_H_
#define _EulerMaruyama_H_

#include "GeneralSDESolver.h"

//Euler Maruyama uses Ito intepretation
//converges with strong order 0.5 and weak order 1
template<typename problem, typename nfield>
class EulerMaruyama : public GeneralSDESolver<EulerMaruyama<problem, nfield>, problem, nfield>
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
	ResultType(EulerMaruyama<problem, nfield>::*toResultFixedTimestep)(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept = nullptr;
	inline auto getResultNextFixedTimestepIto(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType;
	inline auto getResultNextFixedTimestepStratonovich(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType;

public:

	inline EulerMaruyama(const problem &prob,Precision tstep);
	
	inline auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept; // -> ResultType;

};

#include "EulerMaruyama.inl"

#endif //_EulerMaruyama_H_