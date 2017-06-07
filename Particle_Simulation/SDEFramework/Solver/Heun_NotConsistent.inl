///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\Heun_NotConsistent.inl
//
// summary:	Heun not consistent class
///-------------------------------------------------------------------------------------------------
#pragma once


//TODO: Use static analyses to decide wether it is ito or stratonovich
template<typename problem, typename nfield>
inline Heun_NotConsistent<problem, nfield>::Heun_NotConsistent(const problem &prob, Precision tstep)
	: GeneralSDESolver<Heun_NotConsistent<problem, nfield>, problem, nfield>(prob, std::move(tstep))
{
	if ((prob).isIto)
		this->toResultFixedTimestep = &Heun_NotConsistent<problem, nfield>::getResultNextFixedTimestepIto;
	else
		this->toResultFixedTimestep = &Heun_NotConsistent<problem, nfield>::getResultNextFixedTimestepStratonovich;
};

template<typename problem, typename nfield>
inline auto Heun_NotConsistent<problem, nfield>::getResultNextFixedTimestep(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept //-> ResultType
{
	return (this->*toResultFixedTimestep)(yi, xi);
};

template<typename problem, typename nfield>
inline auto Heun_NotConsistent<problem, nfield>::getResultNextFixedTimestepIto(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType
{
	const auto dW = this->m_dWgen.getField();
	const auto dW2 = this->m_dWgen.getField();
	const auto ayi = (this->m_problem).getDeterministicVector(yi, xi);
	const auto byi = (this->m_problem).getStochasticMatrix(yi);
	const auto bdW = (byi*dW).eval();
	auto Support = (yi + ayi*this->m_timestep + bdW).eval();
	auto HeunStep = yi + 0.5 * ((this->m_problem).getDeterministicVector(Support, xi) + ayi)*this->m_timestep 
		+ 0.5*((this->m_problem).getStochasticMatrix(Support)+byi)*dW2;
	return HeunStep.eval();
};

template<typename problem, typename nfield>
inline auto Heun_NotConsistent<problem, nfield>::getResultNextFixedTimestepStratonovich(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType
{
	const auto dW = this->m_dWgen.getField();
	const auto dW2 = this->m_dWgen.getField();
	const auto ayi = (this->m_problem).getDeterministicVector(yi, xi) + (this->m_problem).getDrift(yi);
	const auto byi = (this->m_problem).getStochasticMatrix(yi);
	const auto bdW = (byi*dW).eval();
	auto Support = (yi + ayi*this->m_timestep + bdW).eval();
	auto HeunStep = yi + 0.5 * ((this->m_problem).getDeterministicVector(Support, xi) + (this->m_problem).getDrift(Support) + ayi)*this->m_timestep 
		+ 0.5 * ((this->m_problem).getStochasticMatrix(Support) + byi)*dW2;;
	return HeunStep.eval();
};