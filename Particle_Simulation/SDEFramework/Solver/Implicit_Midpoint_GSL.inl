///-------------------------------------------------------------------------------------------------
// file:	SDEFramework\Solver\Implicit_Midpoint_GSL.inl
//
// summary:	Implicit midpoint solver
///-------------------------------------------------------------------------------------------------
#pragma once

#include <iostream>
#include <cmath>

#include "Implicit_Midpoint_GSL.h"

#include <MyCEL/basics/BasicIncludes.h>

//#define SOLVER_TIMING 0

#ifdef SOLVER_TIMING
#include <MyCEL/basics/Timer.h>
#endif

#include <Eigen/LU>

namespace SDE_Framework::Solvers
{
    template<typename problem, typename nfield>
    Implicit_Midpoint_GSL<problem, nfield>::Implicit_Midpoint_GSL(const Settings& SolverSet, Problem &prob, Precision tstep)
        : GeneralSDESolver<Implicit_Midpoint_GSL<problem, nfield>, problem, nfield>(prob, std::move(tstep)),
        MaxIteration(SolverSet.getMaxIteration()), AccuracyGoal(SolverSet.getAccuracyGoal()), mSolver(SolverSet.getAccuracyGoal(), SolverSet.getAccuracyGoal(),MaxIteration, Problem::Dimension::NumberOfDependentVariables, SolverSet.getImplicitGSLSolverType())
    {
        if (AccuracyGoal <= std::numeric_limits<Precision>::epsilon())
        {
            std::cout << "WARNING: Set AccuracyGoal may not be reachable! Goal: " << AccuracyGoal << " Epsilon: " << std::numeric_limits<Precision>::epsilon() << "\n";
        }

    };

    template<typename problem, typename nfield>
    template<typename IndependentFunctor>
    auto Implicit_Midpoint_GSL<problem, nfield>::getResultNextFixedTimestep(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) //-> ResultType
    {
        //1. Step: Calculate Guess

        const auto dt = this->m_timestep;
        const auto dW = this->m_dWgen.getField();

        //const auto xi = xifunc(time);
        //DependentType yj{ yi };
        //this->m_problem.prepareCalculations(yj);
        //const auto a_guess = (this->m_problem).getDeterministicVector(yj, xi);
        //const auto b_drift = (this->m_problem).getDrift(yj);
        //const auto b_guess = (this->m_problem).getStochasticMatrix(yj);
        //auto dyi = ((a_guess - b_drift)*dt + b_guess*dW).eval();
        //this->m_problem.finishCalculations(dyi);
        //yj = (yj + (a_guess-b_drift)*dt + b_guess*dW).eval(); //Initial Guess! First Step! y_i+1; Also storage for result!
        //Guess
        DependentType yj{ yi }; //Copy the value!
        //this->m_problem.prepareCalculations(yj);
        //const auto xi = xifunc(time);
        //const auto aguess = (this->m_problem).getDeterministicVector(yj, xi);
        //const auto bguess = (this->m_problem).getStochasticMatrix(yj);
        //yj += aguess*dt + bguess*dW; //Drift ignored here just as in the paper!
        //this->m_problem.finishCalculations(yj);

        //2. Step: Start Newton-Raphson Algorithm
        const auto xj = xifunc(time + 0.5*dt).eval();
        /*std::cout << "xj: " << xj.transpose() << "\n";*/
        
        auto f_functor = [&](const auto &yval) -> DependentType
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

        auto df_functor = [&](const auto &yval) -> typename Problem::Traits::JacobiMatrixType
        {
            DependentType res{ (yval + yi)*0.5 };
            this->m_problem.prepareCalculations(res);
            this->m_problem.prepareJacobiCalculations(res);
            const auto Jac_a = (this->m_problem).getJacobiDeterministic(res, xj, dt);
            const auto Jac_b = (this->m_problem).getJacobiStochastic(dW);
            auto S_Jacobi{ (0.5*(dt*Jac_a + Jac_b)).eval() };
            this->m_problem.finishJacobiCalculations(S_Jacobi);
            S_Jacobi -= Problem::Traits::JacobiMatrixType::Identity();
            return S_Jacobi.eval();
        };
        auto fdf_functor = [&](const auto &yval) -> std::tuple<DependentType, typename Problem::Traits::JacobiMatrixType>
        {
            DependentType res{ (yval + yi)*0.5 };
            this->m_problem.prepareCalculations(res);
            const auto a = (this->m_problem).getDeterministicVector(res, xj);
            const auto b = (this->m_problem).getStochasticMatrix(res);
            const auto Jac_a = (this->m_problem).getJacobiDeterministic(res, xj, dt);
            const auto Jac_b = (this->m_problem).getJacobiStochastic(dW);
            auto S_Jacobi{ (0.5*(dt*Jac_a + Jac_b)).eval() };
            res += (a*dt + b*dW).eval();
            this->m_problem.finishCalculations(res);
            this->m_problem.finishJacobiCalculations(S_Jacobi);
            S_Jacobi -= Problem::Traits::JacobiMatrixType::Identity();
            return { res.eval(), S_Jacobi.eval() };
        };

#ifdef SOLVER_TIMING
        Timer<std::chrono::high_resolution_clock, std::chrono::nanoseconds> Timer;
        Timer.start();
#endif
        auto result = mSolver.getResult(std::move(f_functor), std::move(df_functor), std::move(fdf_functor), yj);
        //std::cout << "Result: " << result.transpose() << " Result (Norm): " << result.norm() << '\n';
#ifdef SOLVER_TIMING
        const auto watch = Timer.stop();
        const auto numberofiter = (MaxIteration - Iter + 1);
        std::cout << "Finished Newton-Raphson after " << std::to_string(watch*Timer.unitFactor()) + " s (" << std::to_string(watch / (numberofiter)) << " ns/iteration) Iterations:" << (numberofiter) << std::endl;
#endif

        return result;
    };
};

