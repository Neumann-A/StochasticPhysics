///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\EulerMaruyama_Normalized.inl
//
// summary:	Euler maruyama normalized class
///-------------------------------------------------------------------------------------------------
#pragma once

namespace SDE_Framework::Solvers
{
	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE EulerMaruyama_Normalized<problem, nfield>::EulerMaruyama_Normalized(const Settings&, Problem &prob, Precision tstep)
		: GeneralSDESolver<EulerMaruyama_Normalized<problem, nfield>, problem, nfield>(prob, std::move(tstep))
	{};

	template<typename problem, typename nfield>
	template<typename IndependentFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama_Normalized<problem, nfield>::getResultNextFixedTimestep(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept //-> ResultType
	{
		return detail::FixedTimestepSelector<IsIto::value>::SelectImpl(*this, time, yi, xifunc);
	};

	template<typename problem, typename nfield>
	template<typename IndependentFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama_Normalized<problem, nfield>::getResultNextFixedTimestepIto(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept -> ResultType
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
		(this->m_problem).normalize(res);
		return res;
	};

	template<typename problem, typename nfield>
	template<typename IndependentFunctor>
	BASIC_ALWAYS_INLINE auto EulerMaruyama_Normalized<problem, nfield>::getResultNextFixedTimestepStratonovich(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept -> ResultType
	{
		auto res = yi;
		(this->m_problem).prepareCalculations(res);
		const auto xi = xifunc(time);
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();
		const auto a_ = (this->m_problem).getDeterministicVector(res, xi);
		const auto bb_strich_half = (this->m_problem).getDrift(res);
		const auto b = (this->m_problem).getStochasticMatrix(res);
		res += (a_ + bb_strich_half)*dt + b*dW;
		(this->m_problem).finishCalculations(res);
		(this->m_problem).normalize(res);
		return res;
	};

}
