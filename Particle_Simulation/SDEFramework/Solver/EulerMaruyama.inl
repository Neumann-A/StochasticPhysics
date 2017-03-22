
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
	return (yi + (this->m_problem).getDeterministicVector(yi, xi)*this->m_timestep+ (this->m_problem).getStochasticMatrix(yi)*this->m_dWgen.getField()).eval();
};

template<typename problem, typename nfield>
inline auto EulerMaruyama<problem, nfield>::getResultNextFixedTimestepStratonovich(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept -> ResultType
{
	//Needs drift correction! Stratonovich to ito conversion!
	/*auto dW = this->m_dWgen.getField();
	dW.head<3>() = (*this->m_problem).getStochasticMatrix(yi).block<3, 3>(0, 0)*dW.head<3>();
	dW.tail<3>() = (*this->m_problem).getStochasticMatrix(yi).block<3, 3>(3, 3)*dW.head<3>();*/

	return (yi + ((this->m_problem).getDeterministicVector(yi, xi) + (this->m_problem).getDrift(yi))*this->m_timestep + (this->m_problem).getStochasticMatrix(yi)*this->m_dWgen.getField()).eval();

};
