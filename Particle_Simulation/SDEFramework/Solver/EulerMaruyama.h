/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

/*SDE Solver Euler Maruyama see Kloeden and Platten Numerical methods for further information!*/

#pragma once

#ifndef _EulerMaruyama_H_
#define _EulerMaruyama_H_

#include <type_traits>

#include "GeneralSDESolver.h"
namespace SDE_Framework
{
	//Euler Maruyama uses Ito intepretation
	//converges with strong order 0.5 and weak order 1
	template<typename problem, typename nfield>
	class EulerMaruyama : public GeneralSDESolver<EulerMaruyama<problem, nfield>, problem, nfield>
	{
		template<bool IsIto>
		friend struct detail::FixedTimestepSelector;
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

		BASIC_ALWAYS_INLINE EulerMaruyama(const Problem &prob, Precision tstep);
		BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept; // -> ResultType;

	};

}
#include "EulerMaruyama.inl"

#endif //_EulerMaruyama_H_