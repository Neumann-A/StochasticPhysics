
/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

#pragma once
//#include "EulerMaruyama.h" //File is included by header !

//TODO: Use static analyses to decide wether it is ito or stratonovich
template<typename problem, typename nfield>
inline EulerMaruyama<problem, nfield>::EulerMaruyama(const problem &prob,Precision tstep)
	: GeneralSDESolver<EulerMaruyama<problem, nfield>, problem, nfield>(prob, std::move(tstep))
{
	if ((prob).isIto)
		this->toResultFixedTimestep = &EulerMaruyama<problem, nfield>::getResultNextFixedTimestepIto;
	else
		this->toResultFixedTimestep = &EulerMaruyama<problem, nfield>::getResultNextFixedTimestepStratonovich;
};

template<typename problem, typename nfield>
inline auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestep(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept //-> ResultType
{
	return (this->*toResultFixedTimestep)(yi, xi);
};

template<typename problem, typename nfield>
inline auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestepIto(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType
{
	const auto dt = this->m_timestep;
	const auto dW = this->m_dWgen.getField();
	const auto a = (this->m_problem).getDeterministicVector(yi, xi);
	const auto b = (this->m_problem).getStochasticMatrix(yi);

	return (yi + a *dt + b*dW).eval();
};

template<typename problem, typename nfield>
inline auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestepStratonovich(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType
{
	const auto dt = this->m_timestep;
	const auto dW = this->m_dWgen.getField();
	const auto a_ = (this->m_problem).getDeterministicVector(yi, xi);
	const auto bb_strich_half = (this->m_problem).getDrift(yi);
	const auto b = (this->m_problem).getStochasticMatrix(yi);

	return (yi + (a_ + bb_strich_half)*dt + b*dW ).eval();

};
