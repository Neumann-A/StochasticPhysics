///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\Implicit_Midpoint.inl
//
// summary:	Implicit midpoint solver
///-------------------------------------------------------------------------------------------------
#pragma once

#include "Implicit_Midpoint.h"

namespace SDE_Framework
{
	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE Implicit_Midpoint<problem, nfield>::Implicit_Midpoint(const Settings& SolverSet, const Problem &prob, Precision tstep)
		: GeneralSDESolver<Implicit_Midpoint<problem, nfield>, problem, nfield>(prob, std::move(tstep)) ,MaxIteration(SolverSet.getMaxIteration())
	{};

	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE auto Implicit_Midpoint<problem, nfield>::getResultNextFixedTimestep(const DependentVectorType& yi, const IndependentVectorType& xi) const noexcept //-> ResultType
	{
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();
		
		const auto& a_guess = (this->m_problem).getDeterministicVector(yi, xi);
		const auto& b_guess = (this->m_problem).getStochasticMatrix(yi);
		
		//1. Step: Calculate Guess
		auto yj{ (yi + a_guess*dt + b_guess*dW).eval() }; //Initial Guess! First Step! y_i+1
		(this->m_problem).afterStepCheck(yj);			  //Check and correct step!
		
		//2. Step: Start Newton-Raphson Algorithm
		for (auto Iter{ MaxIteration }; --MaxIteration;)
		{
			//I. Step: Calculate necessary parts
			const auto allparts = (this->m_problem).getAllProblemParts(yj, xi, dt, dW);
			const auto& a = std::get<0>(allparts); //Deterministic Matrix
			const auto& b = std::get<2>(allparts); //Stochastic Matrix
			const auto& Jac_a = std::get<1>(allparts); //Jacobi Deterministic
			const auto& Jac_b = std::get<3>(allparts); //Jacobi Stochastic Matrix
			
			//II. Step: Calculate Jacobi
			const auto S_Jacobi{ Problem::Traits::JacobiMatrixType::Zero() + 0.5*dt*Jac_a + Jac_b };
			
			//III. Step: Solve Implicit equation 
			const auto dx { S_Jacobi.partialPivLu().solve(-(a*dt+b)) };
			
			//IV. Step: Calculate y_j+1 (Here stored in previous yj)
			yj = (yj + dx).eval(); //Eval due to possible aliasing

			if (dx.norm() <= MinDelta) // We reached our accuracy goal before max iteration
			{
				break;
			}
		}
		//(this->m_problem).afterStepCheck(yj); //Normalize if needed
		return yj;
	};
};

