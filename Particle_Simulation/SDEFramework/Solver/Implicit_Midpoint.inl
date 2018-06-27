///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\Implicit_Midpoint.inl
//
// summary:	Implicit midpoint solver
///-------------------------------------------------------------------------------------------------
#pragma once

#include <iostream>
#include <cmath>

#include "Implicit_Midpoint.h"

#include "../Basic_Library/basics/BasicIncludes.h"


//#define SOLVER_TIMING 0

#ifdef SOLVER_TIMING
#include "../Basic_Library/basics/Timer.h"
#endif

//#include <Eigen/LU>

namespace SDE_Framework::Solvers
{
	template<typename problem, typename nfield>
	Implicit_Midpoint<problem, nfield>::Implicit_Midpoint(const Settings& SolverSet, Problem &prob, Precision tstep)
		: GeneralSDESolver<Implicit_Midpoint<problem, nfield>, problem, nfield>(prob, std::move(tstep)), 
		MaxIteration(SolverSet.getMaxIteration()), AccuracyGoal(SolverSet.getAccuracyGoal()) , mSolver(SolverSet.getAccuracyGoal(), SolverSet.getAccuracyGoal(), MaxIteration)
	{
		if (AccuracyGoal <= std::numeric_limits<Precision>::epsilon())
		{
			std::cout << "WARNING: Set AccuracyGoal may not be reachable! Goal: " << AccuracyGoal << " Epsilon: " << std::numeric_limits<Precision>::epsilon() << "\n";
		}
	
	};

	template<typename problem, typename nfield>
	template<typename IndependentFunctor>
	auto Implicit_Midpoint<problem, nfield>::getResultNextFixedTimestep(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) //-> ResultType
	{
		//assert(yi.norm() < 2);
		//1. Step: Calculate Guess
		
		const auto dt = this->m_timestep;
		const auto dW = this->m_dWgen.getField();

		//const auto xi = xifunc(time);
		//const auto a_guess = (this->m_problem).getDeterministicVector(yi, xi);
		//const auto b_drift = (this->m_problem).getDrift(yi);
		//const auto b_guess = (this->m_problem).getStochasticMatrix(yi);
		//DependentType yj{ (yi + (a_guess-b_drift)*dt + b_guess*dW).eval() }; //Initial Guess! First Step! y_i+1; Also storage for result!
		//(this->m_problem).finishCalculations(yj);			  //Check and correct step!
		
		//Ignore the guess!
		DependentType yj{ yi };

		//2. Step: Start Newton-Raphson Algorithm
		const auto xj = xifunc(time+0.5*dt);
		
		auto f_functor = [&](auto &yval) -> DependentType
		{
			DependentType res{ (yval + yi)*0.5 }; //Copy the value!
			this->m_problem.prepareCalculations(res);
			const auto a = (this->m_problem).getDeterministicVector(res, xj);
			const auto b = (this->m_problem).getStochasticMatrix(res);
			res += (a*dt + b*dW).eval(); //res needs to be a valid coordinate to be transformed back 
			this->m_problem.finishCalculations(res);
			res += 0.5*yi - 1.5*yval;
			return res.eval();
		};
		auto df_functor = [&](auto &yval) -> typename Problem::Traits::JacobiMatrixType
		{
			DependentType res{ (yval + yi)*0.5 }; //Copy the value!
			this->m_problem.prepareCalculations(res);
			this->m_problem.prepareJacobiCalculations(res);
			const auto Jac_a = (this->m_problem).getJacobiDeterministic(res, xj, dt);
			const auto Jac_b = (this->m_problem).getJacobiStochastic(dW);
			auto S_Jacobi{ (-Problem::Traits::JacobiMatrixType::Identity() + 0.5*dt*Jac_a + 0.5*Jac_b).eval() };
			this->m_problem.finishJacobiCalculations(S_Jacobi);
			return S_Jacobi;
		};
		auto fdf_functor = [&](auto &yval) -> std::tuple<DependentType, typename Problem::Traits::JacobiMatrixType>
		{
			DependentType res{ (yval + yi)*0.5 }; //Copy the value!
			this->m_problem.prepareCalculations(res);
			this->m_problem.prepareJacobiCalculations(res);
			const auto a = (this->m_problem).getDeterministicVector(res, xj);
			const auto b = (this->m_problem).getStochasticMatrix(res);
			const auto Jac_a = (this->m_problem).getJacobiDeterministic(res, xj, dt);
			const auto Jac_b = (this->m_problem).getJacobiStochastic(dW);
			auto S_Jacobi{ (-Problem::Traits::JacobiMatrixType::Identity() + 0.5*dt*Jac_a + 0.5*Jac_b).eval() };
			res += (a*dt + b*dW).eval(); //res needs to be a valid coordinate to be transformed back
			this->m_problem.finishCalculations(res);
			res += 0.5*yi - 1.5*yval;
			this->m_problem.finishJacobiCalculations(S_Jacobi);
			return { res, S_Jacobi };
		};

#ifdef SOLVER_TIMING
		Timer<std::chrono::high_resolution_clock, std::chrono::nanoseconds> _Timer;
		_Timer.start();
#endif

		auto result = mSolver.getResult(f_functor, df_functor, fdf_functor, yj);

		//Precision lastnorm{ static_cast<Precision>(0.0) };
		//std::size_t Iter{ MaxIteration + 1 };
		//for (; --Iter;)
		//{
		//	//I. Step: Calculate necessary parts
		//	const auto allparts = (this->m_problem).getAllProblemParts(yj, xj, dt, dW);
		//	const DependentType& a = std::get<0>(allparts); //Deterministic Matrix
		//	const auto& b = std::get<2>(allparts); //Stochastic Matrix
		//	const typename Problem::Traits::JacobiMatrixType& Jac_a = std::get<1>(allparts); //Jacobi Deterministic
		//	const typename Problem::Traits::JacobiMatrixType& Jac_b = std::get<3>(allparts); //Jacobi Stochastic Matrix

		//	//II. Step: Calculate Jacobi
		//	const typename Problem::Traits::JacobiMatrixType S_Jacobi{ (Problem::Traits::JacobiMatrixType::Identity() - 0.5*dt*Jac_a - 0.5*Jac_b).eval() };

		//	//III. Step: Solve Implicit equation 
		//	Solver.compute(S_Jacobi);

		//	DependentType tmp{ (-(a*dt + b*dW)).eval() };
		//	const DependentType dx{ Solver.solve(tmp) };

		//	if (std::isnan(dx.norm()) || yj.norm() > 1.2)
		//	{
		//		std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
		//		std::cout << "----------" << "\n";
		//		std::cout << "yi: " << yi << "\n";
		//		std::cout << "xj: " << xj << "\n";
		//		std::cout << "----------" << "\n";
		//		std::cout << "a: " << a << "\n";
		//		std::cout << "b: " << b << "\n";
		//		std::cout << "----------" << "\n";
		//		std::cout << "Jac_a: " << Jac_a << "\n";
		//		std::cout << "Jac_b: " << Jac_b << "\n";
		//		std::cout << "S_Jacobi: " << S_Jacobi << "\n";
		//		std::cout << "******************" << "\n";
		//		std::cout << "dx: " << dx << "\n";
		//		std::cout << "dxnorm: " << dx.norm() << "\n";
		//		std::cout << "+++++++++" << "\n";
		//		std::cout << "yj: " << yj << "\n";
		//		std::cout << "yjnorm: " << yj.norm() << "\n";
		//		std::cout << "+++++++++" << "\n";
		//		std::system("pause");
		//	}

		//	//IV. Step: Calculate y_j+1 (Here stored in previous yj)
		//	yj = (yj + dx).eval(); //Eval due to possible aliasing

		//	//(this->m_problem).finishCalculations(yj);			  //Check and correct step!

		//	//std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
		//	//std::cout << "----------" << "\n";
		//	//std::cout << "yi: " << yi << "\n";
		//	//std::cout << "xj: " << xj << "\n";
		//	//std::cout << "----------" << "\n";
		//	//std::cout << "a: " << a << "\n";
		//	//std::cout << "b: " << b << "\n";
		//	//std::cout << "----------" << "\n";
		//	//std::cout << "Jac_a: " << Jac_a << "\n";
		//	//std::cout << "Jac_b: " << Jac_b << "\n";
		//	//std::cout << "S_Jacobi: " << S_Jacobi << "\n";
		//	//std::cout << "******************" << "\n";
		//	//std::cout << "dx: " << dx << "\n";
		//	//std::cout << "dxnorm: " << dx.norm() << "\n";
		//	//std::cout << "+++++++++" << "\n";
		//	//std::cout << "yj: " << yj << "\n";
		//	//std::cout << "yjnorm: " << yj.norm() << "\n";
		//	//std::cout << "+++++++++" << "\n";
		//	//std::system("pause");

		//	if (dx.norm() <= AccuracyGoal) // We reached our accuracy goal before max iteration
		//	{
		//		//std::cout << "Accuracy Goal of " << BasicTools::toStringScientific(AccuracyGoal) << " reached after: " << std::to_string(MaxIteration - Iter) << " Iteration!\n";
		//		break;
		//	}
		//}
#ifdef SOLVER_TIMING
		const auto watch = _Timer.stop();
		const auto numberofiter = (MaxIteration - Iter + 1);
		std::cout << "Finished Newton-Raphson after " << std::to_string(watch*_Timer.unitFactor()) + " s (" << std::to_string(watch / (numberofiter)) << " ns/iteration) Iterations:" << (numberofiter) << std::endl;
#endif
		
		return result;
	};
};

