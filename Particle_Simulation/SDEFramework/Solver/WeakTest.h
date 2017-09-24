///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\WeakTest.h
//
// summary: 	Declares the weak test class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 08.06.2017

#ifndef INC_WeakTest_H
#define INC_WeakTest_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "GeneralSDESolver.h"
#include "Settings/SolverSettings.h"

namespace SDE_Framework
{

	template <typename problem, typename nfield, typename nmatrix>
	class WeakTest :
		public GeneralSDESolver<WeakTest<problem, nfield, nmatrix>, problem, nfield>
	{
	public:
		typedef	problem																								   Problem;
		typedef typename problem::DependentVectorType																   ResultType;
		typedef nfield																								   NoiseField;
		typedef nmatrix																							       NoiseMatrix;
		typedef typename problem::Precision																			   Precision;

		using ResultTypeAllocator = typename Problem::Traits::DependentVectorStdAllocator;
		using Settings = Settings::SolverSettings<Precision>;
	private:
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
		WeakTest(const Settings& SolverSettings, Problem &prob, Precision tstep);

		inline auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const->ResultType;
	};

}
#include "WeakTest.inl"

#endif	// INC_WeakTest_H
// end of SDEFramework\Solver\WeakTest.h
///---------------------------------------------------------------------------------------------------
