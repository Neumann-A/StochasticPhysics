///---------------------------------------------------------------------------------------------------
// file:		SDEFramework\Solver\GeneralSDESolver.h
//
// summary: 	Declares the general sde solver class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 07.10.2017

#ifndef INC_GeneralSDESolver_H
#define INC_GeneralSDESolver_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>

#include "../Basic_Library/basics/BasicMacros.h"
#include "../Basic_Library/stdext/std_extensions.h"

namespace SDE_Framework::Solvers
{
	namespace detail
	{
		template<typename Solver>
		struct is_implicit_solver : std::false_type {};
		template<typename Solver>
		constexpr bool is_implicit_solver_v = is_implicit_solver<Solver>::value;

		template<typename Solver>
		struct is_explicit_solver : std::false_type {};
		template<typename Solver>
		constexpr bool is_explicit_solver_v = is_explicit_solver<Solver>::value;

		template<typename Solver>
		constexpr bool is_valid_solver_v = is_explicit_solver_v<Solver> ^ is_implicit_solver_v <Solver>;

		template <bool isIto = true>
		struct FixedTimestepSelector
		{
			template<typename Solver, typename Precision, typename DependentType, typename IndependentFunctor>
			BASIC_ALWAYS_INLINE static auto SelectImpl(Solver&& sol, Precision &&time, DependentType&& yi, IndependentFunctor&& xifunc) noexcept
			{
				return sol.getResultNextFixedTimestepIto(time, yi, xifunc);
			}
		};

		template <>
		struct FixedTimestepSelector<false>
		{
			template<typename Solver, typename Precision, typename DependentType, typename IndependentFunctor>
			BASIC_ALWAYS_INLINE static auto SelectImpl(Solver&& sol, Precision &&time, DependentType&& yi, IndependentFunctor&& xifunc) noexcept
			{
				return sol.getResultNextFixedTimestepStratonovich(std::forward<Precision>(time),std::forward<DependentType>(yi), std::forward<IndependentFunctor>(xifunc));
			}
		};
	}

	///-------------------------------------------------------------------------------------------------
	/// <summary>	CRTP class for SDE Solvers </summary>
	///
	/// <typeparam name="solver">	 	Type of the derived solver. </typeparam>
	/// <typeparam name="problem">   	Type of the problem which the solver tackles. </typeparam>
	/// <typeparam name="noisefield">	Type of the noisefield the solver uses. </typeparam>
	///-------------------------------------------------------------------------------------------------
	template<typename solver, typename problem, typename noisefield>
	class GeneralSDESolver
	{
		static_assert(detail::is_valid_solver_v<solver>,"A solver must be either an explicit or implicit solver! (Maybe you forgot to define the partial specialization for it!)");
	public:
		using SolverType = solver;
		using ProblemType = problem;
		using TraitsType = typename ProblemType::Traits;

		using ResultType = typename ProblemType::DependentType;
		using NoiseField = noisefield;
	private:
		using Precision					= typename ProblemType::Precision;
		using Dimensions				= typename ProblemType::Dimension;
		using DependentType				= typename ProblemType::DependentType;
		using IndependentType			= typename ProblemType::IndependentType;
		using DeterministicType			= typename ProblemType::DeterministicType;
		using StochasticMatrixType		= typename ProblemType::StochasticMatrixType;
	protected:
		GeneralSDESolver() = default;
	private:
		DISALLOW_COPY_AND_ASSIGN(GeneralSDESolver)

		SolverType& self() noexcept// Handy Helper to remove the static_casts from the code
		{
			return *static_cast<SolverType * const>(this);
		};
	protected:
		ProblemType& m_problem;	// Reference to the problem; should not be a problem since this class should always be used with a Simulator Class which owns the problem and the solver
		const Precision m_timestep;		// Value of the Timestep 
		mutable NoiseField m_dWgen;     // Generator of the Random Field; Mutable since it will use random number generators (which cannot be const); This fact does not change the solver itself!

	public:
		constexpr BASIC_ALWAYS_INLINE GeneralSDESolver(ProblemType &sdeprob, Precision timestep) : m_problem(sdeprob), m_timestep(timestep), m_dWgen(1'000'000, timestep) {};

		constexpr BASIC_ALWAYS_INLINE auto getResultNextTimestep(const DependentType &yi, const IndependentType &xi) const noexcept -> ResultType
		{
			return self().getResultNextFixedTimestep(yi, xi);
		};

		constexpr BASIC_ALWAYS_INLINE auto getResultNextFixedTimestep(const DependentType &yi, const IndependentType &xi) const noexcept-> ResultType //decltyp(solver().getResultNextFixedTimestep(yi, xi))
		{
			return self().getResultNextFixedTimestep(yi, xi);
		};

		constexpr BASIC_ALWAYS_INLINE const Precision& getTimestep() const noexcept { return m_timestep; };
	};
}

#endif	// INC_GeneralSDESolver_H
// end of SDEFramework\Solver\GeneralSDESolver.h
