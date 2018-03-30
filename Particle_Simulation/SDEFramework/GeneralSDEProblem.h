#pragma once

#ifndef INC_GENERALSDEPROBLEM_H_
#define INC_GENERALSDEPROBLEM_H_

#include "basics/BasicMacros.h"
#include "stdext/std_extensions.h"


namespace Problems
{
	namespace traits
	{
		template<class T, typename ...Args>
		using get_start_vector_t = decltype(std::declval<T&>().getStartVector(std::declval<Args&>()...));

		template<class T, typename ...Args>
		using get_stochastic_matrix_t = decltype(std::declval<T&>().getStochasticMatrix(std::declval<Args&>()...));

		template<class T, typename ...Args>
		using get_deterministic_vector_t = decltype(std::declval<T&>().getDeterministicVector(std::declval<Args&>()...));

		template<class T, typename ...Args>
		using get_ito_stratonovich_drift = decltype(std::declval<T&>().getDrift(std::declval<Args&>()...));

		template<class T, typename ...Args>
		using perform_after_step_check_t = decltype(std::declval<T&>().finishCalculations(std::declval<Args&>()...));

		//Checks if the Problem has the start member
		template<typename Problem>
		class has_get_start : public stdext::is_detected_exact<typename Problem::DependentType, get_start_vector_t, Problem, const typename Problem::InitSettings> {};

		template<typename Problem>
		class has_get_determinisitc_matrix : public stdext::is_detected_exact<typename Problem::DeterministicType, get_stochastic_matrix_t, Problem,
			const typename Problem::DependentType, const typename Problem::IndependentType> {};

		template<typename Problem>
		class has_ito_stratonovich_drift : public stdext::is_detected_exact<typename Problem::DeterministicType, get_ito_stratonovich_drift, Problem,
			const typename Problem::DependentType> {};

		template<typename Problem>
		class has_get_stochastic_matrix : public stdext::is_detected_exact <typename Problem::StochasticMatrixType, get_stochastic_matrix_t, Problem, const typename Problem::DependentType > {};

	}

	template<unsigned int T, unsigned int U, unsigned int W>
	struct GeneralSDEDimension
	{
		static constexpr const std::size_t NumberOfDependentVariables = T;
		static constexpr const std::size_t NumberOfIndependentVariables = U;
		static constexpr const std::size_t SizeOfNoiseVector = W;
	};

	//Forward Declare Base Traits for all SDE Problems
	template<typename T>
	class SDEProblem_Traits
	{
		static_assert(T::value, "Forgot to define Traits class for the Problem!");
	};

	/* GeneralSDEProblem Class
	* Describes a General stochastic differential equation problem with dy_i = a dt + b dW in Stratonovich Form (dW is such that it is normal distributed with mean 1 or sqrt(dt))
	* a is a Matrix describing the deterministic part
	* b is a Matrix describing the random(stochastic) part
	* The Problem should also include which field is used and which anisotropy is used
	* Using CRTP idom for static polymorphism and be used as a general interface for all SDE problems
	* template parameter dim has to have the following static constexpr cons int fields: NumberOfDependentVariables,NumberOfIndependentVariables,SizeOfNoiseVector
	* Author: Alexander Neumann
	* Date:   23.08.2015
	*/

	template <class problem, typename Ito = typename SDEProblem_Traits<problem>::IsIto>
	class GeneralSDEProblem
	{
	public:
		using Derived = problem;
		using Problem = problem;
		using Traits = SDEProblem_Traits<Problem>;
		using IsIto = Ito;
		using Dimension = typename Traits::Dimension;
		using Precision = typename Traits::Precision;

		///Defined in ProblemSelector! we need it there not here!
		typedef typename Traits::ProblemSettings																ProblemSettings;
		typedef typename Traits::UsedProperties			     													UsedProperties;
		typedef typename Traits::InitSettings																	InitSettings;

		//cannot use the defined values from problem since it is an incomplete type in here
		using StochasticMatrixType = typename Traits::StochasticMatrixType;
		using DeterministicType = typename Traits::DeterministicType;
		using DependentType = typename Traits::DependentType;
		using IndependentType = typename Traits::IndependentType;
		using NoiseType = typename Traits::NoiseType;

	protected:
		GeneralSDEProblem() = default; //Should be an Interface and thus constructor is protected
		GeneralSDEProblem(Dimension a) : m_dim(a) {}; //Should be an Interface and thus constructor is protected
		~GeneralSDEProblem() = default; // not virtual but protected! 
	private:
		BASIC_ALWAYS_INLINE Derived& prob() BASIC_NOEXCEPT
		{
			return *static_cast<Derived * const>(this);
		};
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		//ALLOW_DEFAULT_COPY_AND_ASSIGN(GeneralSDEProblem<problem, Ito>)

		static constexpr bool isIto{ typename IsIto::value_type() };
		const Dimension m_dim; // TODO: Do we need this member?

		BASIC_ALWAYS_INLINE DependentType getStart(const InitSettings& init)
		{
			return prob().getStart(init);
		}

		// Gets the stochastic Matrix b of the SDE  
		BASIC_ALWAYS_INLINE StochasticMatrixType getStochasticMatrix(const DependentType& yi)
		{
			return prob().getStochasticMatrix(yi);
		};

		// Get the deterministic Matrix a of the SDE
		BASIC_ALWAYS_INLINE DeterministicType getDeterministicVector(const DependentType& yi, const IndependentType& xi)
		{
			return prob().getDeterministicVector(yi, xi);
		};

		// Get the deterministic Matrix a of the SDE
		BASIC_ALWAYS_INLINE DeterministicType getDrift(const DependentType& yi)
		{
			return prob().getDrift(yi);
		};

		// Checks the Result after the performed simulation steps and does a correction if necessary 
		BASIC_ALWAYS_INLINE void finishCalculations(DependentType& yi) const
		{
			prob().finishCalculations(yi);
		};
	};
};
#endif // _GENERALSDEPROBLEM_H_
