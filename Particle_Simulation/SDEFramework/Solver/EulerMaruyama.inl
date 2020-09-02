
/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

#pragma once

//#include "EulerMaruyama.h" //File is included by header !

namespace SDE_Framework::Solvers
{
	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE EulerMaruyama<problem, nfield>::EulerMaruyama(const Settings&, Problem &prob, Precision tstep) noexcept
		: GeneralSDESolver<EulerMaruyama<problem, nfield>, problem, nfield>(prob, std::move(tstep))
	{};

	template<typename problem, typename nfield>
	template<typename IndependentFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestep(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept //-> ResultType
	{
		return detail::FixedTimestepSelector<IsIto::value>::SelectImpl(*this,time,yi, xifunc);
	};

	template<typename problem, typename nfield>
	template<typename IndependentFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestepIto(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept -> ResultType
	{
		auto res = yi;
		(this->m_problem).prepareCalculations(res);
		const auto xi = xifunc(time);
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();
		const auto a = (this->m_problem).getDeterministicVector(res, xi);
		const auto b = (this->m_problem).getStochasticMatrix(res);
		res += a*dt + b*dW;
		(this->m_problem).finishCalculations(res);
		return res;
	};

	template<typename problem, typename nfield>
	template<typename IndependentFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestepStratonovich(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept -> ResultType
	{
		//std::cout << "Input: " << yi.transpose() << '\n';
		auto res = yi;
		(this->m_problem).prepareCalculations(res);
		//std::cout << "Input after prepare: " << res.transpose() << '\n';
		const auto xi = xifunc(time);
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();
		const auto a_ = (this->m_problem).getDeterministicVector(res, xi);
		const auto bb_strich_half = (this->m_problem).getDrift(res);
		const auto b = (this->m_problem).getStochasticMatrix(res);
		res += (a_ + bb_strich_half)*dt + b*dW;
		//std::cout << "Result before finish: " << res.transpose() << '\n';
		(this->m_problem).finishCalculations(res);

		//std::cout << "DeterministicPart: " << ((a_ + bb_strich_half)*dt).transpose() << '\n';
		//std::cout << "StochasticPart: " << (b*dW).transpose() << '\n';
		//std::cout << "StochasticMatrix: \n" << b << '\n';
		//std::cout << "Determinisitc*dt: " << (a_*dt).transpose() << '\n';
		//std::cout << "Drift*dt: " << (bb_strich_half*dt).transpose() << '\n';
		//std::cout << "Noise*dW: " << (b*dW).transpose() << '\n';
		//std::cout << "Output: " << res.transpose() << '\n';
		//std::cout << "********************************************\n";
		//system("pause");


		
		return res;
	};
}
