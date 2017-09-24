
/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

#pragma once

//#include "EulerMaruyama.h" //File is included by header !

namespace SDE_Framework
{
	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE EulerMaruyama<problem, nfield>::EulerMaruyama(const Settings& SolverSettings, Problem &prob, Precision tstep)
		: GeneralSDESolver<EulerMaruyama<problem, nfield>, problem, nfield>(prob, std::move(tstep))
	{};

	template<typename problem, typename nfield>
	template<typename IndependentVectorFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestep(const Precision &time, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) const noexcept //-> ResultType
	{
		return detail::FixedTimestepSelector<IsIto::value>::SelectImpl(*this,time,yi, xifunc);
	};

	template<typename problem, typename nfield>
	template<typename IndependentVectorFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestepIto(const Precision &time, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) const noexcept -> ResultType
	{
		const auto xi = xifunc(time);
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();
		const auto a = (this->m_problem).getDeterministicVector(yi, xi);
		const auto b = (this->m_problem).getStochasticMatrix(yi);

		return (yi + a *dt + b*dW).eval();
	};

	template<typename problem, typename nfield>
	template<typename IndependentVectorFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestepStratonovich(const Precision &time, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) const noexcept -> ResultType
	{
		const auto xi = xifunc(time);
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();
		const auto a_ = (this->m_problem).getDeterministicVector(yi, xi);
		const auto bb_strich_half = (this->m_problem).getDrift(yi);
		const auto b = (this->m_problem).getStochasticMatrix(yi);

		return (yi + (a_ + bb_strich_half)*dt + b*dW).eval();
	};

}