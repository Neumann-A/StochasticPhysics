/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

#pragma once

#ifndef _GeneralSDESolver_H_
#define _GeneralSDESolver_H_

//#include <Eigen\Core>

/* GeneralSDESolver Class
*  Represents an interface witch every SDESolver should inherit
*  Template problem is a class of type GeneralSDEProblem
*  Template precision is either float or double
*/

//
template<typename solver, typename problem, typename noisefield>
class GeneralSDESolver
{
public:
	typedef typename problem::DependentVectorType																   ResultType;
	typedef		     noisefield																					   NoiseField;
private:
	typedef typename problem::Precision																			   Precision;
	typedef typename problem::Dimension																			   Dimensions;
	typedef typename problem::DependentVectorType																   DependentVectorType;
	typedef typename problem::IndependentVectorType																   IndependentVectorType;
	typedef typename problem::DeterministicVectorType															   DeterministicVectorType;
	typedef typename problem::StochasticMatrixType																   StochasticMatrixType;
protected:
	GeneralSDESolver() {};
private:
	DISALLOW_COPY_AND_ASSIGN(GeneralSDESolver)
	//Eigen::Matrix<precision, dim::SizeOfNoiseVector, 1> generateNoiseVector()
	//{
	//	return this->solver().generateNoiseVector();
	//};

	solver& SDEsolver() // Handy Helper to remove the static_casts from the code
	{
		return static_cast<solver&>(*this);
	};
protected:
	const problem& m_problem;			// Reference to the problem; should not be a problem since this class is always used with a Simulator Class which holds the problem and the solver
	const Precision m_timestep;		//Reference to the timestep since 
	mutable noisefield m_dWgen;

public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	constexpr inline GeneralSDESolver(const problem &sdeprob,Precision timestep) : m_problem(sdeprob), m_timestep(timestep), m_dWgen(1000000, timestep) {};

	constexpr inline auto getResultNextTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept -> ResultType
	{
		return SDEsolver().getResultNextFixedTimestep(yi, xi);
	};

	constexpr inline auto getResultNextFixedTimestep(const DependentVectorType &yi, const IndependentVectorType &xi) const noexcept-> ResultType //decltyp(solver().getResultNextFixedTimestep(yi, xi))
	{
		return SDEsolver().getResultNextFixedTimestep(yi, xi);
	};

	constexpr inline const Precision& getTimestep() const noexcept { return this->m_timestep; };

	virtual ~GeneralSDESolver() {};
};

#endif