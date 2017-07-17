///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\Implicit_Midpoint.inl
//
// summary:	Implicit midpoint solver
///-------------------------------------------------------------------------------------------------
#pragma once

#include "Implicit_Midpoint.h"

#include "../Basic_Library/basics/BasicIncludes.h"

#include <Eigen/LU>

namespace SDE_Framework
{
	template<typename problem, typename nfield>
	BASIC_ALWAYS_INLINE Implicit_Midpoint<problem, nfield>::Implicit_Midpoint(const Settings& SolverSet, const Problem &prob, Precision tstep)
		: GeneralSDESolver<Implicit_Midpoint<problem, nfield>, problem, nfield>(prob, std::move(tstep)), 
		MaxIteration(SolverSet.getMaxIteration()), AccuracyGoal(SolverSet.getAccuracyGoal())
	{
		if (AccuracyGoal <= std::numeric_limits<Precision>::epsilon())
		{
			std::cout << "WARNING: Set AccuracyGoal may not be reachable! Goal: " << AccuracyGoal << " Epsilon: " << std::numeric_limits<Precision>::epsilon() << "\n";
		}
	
	};

	template<typename problem, typename nfield>
	template<typename IndependentVectorFunctor>
	BASIC_ALWAYS_INLINE auto Implicit_Midpoint<problem, nfield>::getResultNextFixedTimestep(const Precision &time, const DependentVectorType &yi, const IndependentVectorFunctor &xifunc) const noexcept //-> ResultType
	{
		//1. Step: Calculate Guess
		
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();


		//const auto xi = xifunc(time);
		//const auto& a_guess = (this->m_problem).getDeterministicVector(yi, xi);
		//const auto& b_guess = (this->m_problem).getStochasticMatrix(yi);
		//auto yj{ (yi + a_guess*dt + b_guess*dW).eval() }; //Initial Guess! First Step! y_i+1; Also storage for result!
		//(this->m_problem).afterStepCheck(yj);			  //Check and correct step!
		
		//Ignore the guess!
		DependentVectorType yj{ yi };

		//2. Step: Start Newton-Raphson Algorithm
		const auto xj = xifunc(time+0.5*dt);
		for (std::size_t Iter{ MaxIteration+1 }; --Iter;)
		{
			//I. Step: Calculate necessary parts
			const auto allparts = (this->m_problem).getAllProblemParts(yj, xj, dt, dW);
			const auto& a = std::get<0>(allparts); //Deterministic Matrix
			const auto& b = std::get<2>(allparts); //Stochastic Matrix
			const auto& Jac_a = std::get<1>(allparts); //Jacobi Deterministic
			const auto& Jac_b = std::get<3>(allparts); //Jacobi Stochastic Matrix
			
			//II. Step: Calculate Jacobi
			const auto S_Jacobi{ (Problem::Traits::JacobiMatrixType::Identity() + 0.5*dt*Jac_a + Jac_b).eval() };
			
			//III. Step: Solve Implicit equation 
			//Eigen::FullPivLU<typename Problem::Traits::JacobiMatrixType> Solver{ S_Jacobi };
			Eigen::PartialPivLU<typename Problem::Traits::JacobiMatrixType> Solver{ S_Jacobi };
			auto tmp{ (-(a*dt + b*dW)).eval() };
			const auto dx { Solver.solve(tmp) };

			//IV. Step: Calculate y_j+1 (Here stored in previous yj)
			yj = (yj + dx).eval(); //Eval due to possible aliasing

			//std::cout << "xj: " << xj << "\n";
			//std::cout << "a: " << a << "\n";
			//std::cout << "b: " << b << "\n";
			//std::cout << "Jac_a: " << Jac_a << "\n";
			//std::cout << "Jac_b: " << Jac_b << "\n";
			//std::cout << "******************" << "\n";
			//std::cout << "dx: " << dx << "\n";
			//std::cout << "yj: " << yj << "\n";
			//std::cout << "******************" << std::endl;
			//std::system("pause");

			if (dx.norm() <= AccuracyGoal) // We reached our accuracy goal before max iteration
			{
				//std::cout << "Accuracy Goal of " << BasicTools::toStringScientific(AccuracyGoal) << " reached after: " << std::to_string(MaxIteration - Iter) << " Iteration!\n";
				break;
			}
		}
		//(this->m_problem).afterStepCheck(yj); //Normalize if needed
		return yj;
	};
};

