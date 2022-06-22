///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\EulerMaruyama.h
//
// summary: 	Declares the euler maruyama solver
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 07.10.2017

#ifndef INC_EulerMaruyama_H
#define INC_EulerMaruyama_H
///---------------------------------------------------------------------------------------------------
/*SDE Solver Euler Maruyama see Kloeden and Platten Numerical methods for further information!*/
#pragma once

#include <type_traits>

#include "GeneralSDESolver.h"
#include "Settings/SolverSettings.h"

namespace SDE_Framework::Solvers
{

    //Euler Maruyama uses Ito intepretation
    //converges with strong order 0.5 and weak order 1
    template<typename problem, typename nfield>
    class EulerMaruyama : public GeneralSDESolver<EulerMaruyama<problem, nfield>, problem, nfield>
    {
        template<bool IsIto>
        friend struct detail::FixedTimestepSelector;
    public:
        using Problem = problem;
        using Precision = typename Problem::Traits::Precision;
        
        using ResultType = typename Problem::Traits::DependentType;

        using NoiseField = nfield;
        using Settings = Settings::SolverSettings<Precision>;
    private:
        using IsIto = typename Problems::SDEProblem_Traits<problem>::IsIto;
        
        using DependentType = typename Problem::DependentType;
        using IndependentType = typename Problem::IndependentType;
        using DeterministicType = typename Problem::DeterministicType;
        using StochasticMatrixType = typename Problem::StochasticMatrixType;

    private:
        template<typename IndependentFunctor>
        BASIC_ALWAYS_INLINE auto getResultNextFixedTimestepIto(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept->ResultType;
        template<typename IndependentFunctor>
        BASIC_ALWAYS_INLINE auto getResultNextFixedTimestepStratonovich(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept->ResultType;

    public:
        BASIC_ALWAYS_INLINE EulerMaruyama(const Settings& SolverSettings, Problem &prob, Precision tstep) noexcept;

        template<typename IndependentFunctor>
        BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const Precision &time, const DependentType &yi, const IndependentFunctor &xifunc) const noexcept; // -> ResultType;
    };

    namespace detail
    {
        template<typename problem, typename nfield>
        struct is_explicit_solver<EulerMaruyama<problem, nfield>> : std::true_type {};
    }

}
#include "EulerMaruyama.inl"

#endif	// INC_EulerMaruyama_H
// end of SDEFramework\Solver\EulerMaruyama.h
