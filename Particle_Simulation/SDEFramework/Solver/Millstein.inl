///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\Millstein.inl
//
// summary:	Millstein class
///-------------------------------------------------------------------------------------------------
#pragma once

namespace SDE_Framework
{
	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE Millstein<problem, nfield>::Millstein(const problem &prob, Precision tstep)
		: GeneralSDESolver<Millstein<problem, nfield>, problem, nfield>(prob, std::move(tstep))
	{};

	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE auto Millstein<problem, nfield>::getResultNextFixedTimestep(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept //-> ResultType
	{
		return detail::FixedTimestepSelector<IsIto::value>::SelectImpl(*this, yi, xi);;
	};

	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE auto Millstein<problem, nfield>::getResultNextFixedTimestepIto(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType
	{
		//TODO: Make formular more general; is only correct for some problems!
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();
		const auto a_ = (this->m_problem).getDeterministicVector(yi, xi);
		const auto bb_strich_half = (this->m_problem).getDrift(yi);
		const auto b = (this->m_problem).getStochasticMatrix(yi);

		return (yi + (a_ + bb_strich_half) * dt + b * dW + bb_strich_half*(dW.dot(dW) - dt)).eval();
	};

	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE auto Millstein<problem, nfield>::getResultNextFixedTimestepStratonovich(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType
	{
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();
		const auto a_ = (this->m_problem).getDeterministicVector(yi, xi);
		const auto bb_strich_half = (this->m_problem).getDrift(yi);
		const auto b = (this->m_problem).getStochasticMatrix(yi);

		return (yi + a_ * dt + b * dW + bb_strich_half*dW.dot(dW)).eval();
	};
}