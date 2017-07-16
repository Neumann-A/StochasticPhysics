/*
* An explicit strong order 1.0 Solver
* Author: Alexander Neumann
* Date : 23.08.2015
*/
#pragma once

//IMPORTANT: Although seemingly correctly implemented the hysteresis of the nanoparticles was shifted with this solver! So do not use it 
//			 Furthermore Neel Relxation to zero not possible with this solver. Relaxed to something near -0.5

#ifndef _DerivativeFreeMillstein_H_
#define _DerivativeFreeMillstein_H_

#include "GeneralSDESolver.h"
#include "Settings/SolverSettings.h"

//#include "DoubleNoiseMatrix.h"
using namespace SDE_Framework;
//Converges with strong order 1
template <typename problem, typename nfield, typename nmatrix>
class Explicit_Strong_1 :
	public GeneralSDESolver<Explicit_Strong_1<problem, nfield, nmatrix>, problem, nfield>
{
	template<bool IsIto>
	friend struct detail::FixedTimestepSelector;

public:
	typedef	problem																								   Problem;
	typedef typename problem::DependentVectorType																   ResultType;
	typedef nfield																								   NoiseField;
	typedef nmatrix																							       NoiseMatrix;
	typedef typename problem::Precision																			   Precision;

	using ResultTypeAllocator = typename Problem::Traits::DependentVectorStdAllocator;
	using Settings = Settings::SolverSettings<Precision>;
private:
	using IsIto = typename Problems::SDEProblem_Traits<problem>::IsIto;

	typedef typename problem::Dimension																			   Dimensions;
	typedef typename problem::DependentVectorType																   DependentVectorType;
	typedef typename problem::IndependentVectorType																   IndependentVectorType;
	typedef typename problem::DeterministicVectorType															   DeterministicVectorType;
	typedef typename problem::StochasticMatrixType																   StochasticMatrixType;

	//Private variables to create some memory space
	mutable nfield m_dWgen;																								//Noisefield generator
	mutable nmatrix m_J_j1j2gen;																						//Double NoiseMatrix
	const Precision m_sqrttimestep = 0;

public:
	BASIC_ALWAYS_INLINE Explicit_Strong_1(const Settings& SolverSettings, const Problem &prob, Precision tstep);

	BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const -> ResultType;
};

#include "Explicit_Strong_1.inl"

#endif //_Explicit_Strong_1_H_