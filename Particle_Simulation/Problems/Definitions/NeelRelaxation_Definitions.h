///---------------------------------------------------------------------------------------------------
// file:		Problems\Definitions\NeelRelaxation_Definitions.h
//
// summary: 	Defines the necessary types for the neel problem
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 15.10.2017

#ifndef INC_NeelRelaxation_Definitions_H
#define INC_NeelRelaxation_Definitions_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <Eigen/StdVector>

#include "GeneralProblem_Definitions.h"

#include "Selectors/ProblemSelector.h"
#include "Problems/NeelRelaxation.h"

#include "Settings/ProblemSettings.h"
#include "Settings/ParticleSimulationParameters.h"
#include "Provider/ParticleProvider.h"
#include "Properties/ParticleProperties.h"

namespace Selectors
{
	template <>
	class ProblemTypeSelector<IProblem::Problem_Neel> : public BasicSelector<ProblemTypeSelector<IProblem::Problem_Neel>>
	{
	public:
		typedef IProblem			valuetype;
		static constexpr valuetype value = { IProblem::Problem_Neel };

		using IsMagneticProblem = typename Problems::detail::template is_magnetic_problem<value>;
		using UsesAnisotropy = typename Problems::detail::template uses_magnetic_anisotropy<value>;
		using UsesBoundaryCondition = typename Problems::detail::template uses_boundaries<value>;
		using IsIto = typename Problems::detail::template has_ito_noise<value>;

		template<typename prec, typename anisotropy>
		using ProblemType = Problems::NeelRelaxation<prec, anisotropy>;

		template<IAnisotropy Aniso>
		using Anisotropy = AnisotropyTypeSelector<Aniso>;

		template<typename prec, IAnisotropy AnisotropyID>
		using ProblemType_Select = Problems::NeelRelaxation<prec, typename Anisotropy<AnisotropyID>::template type<prec>>;

		template<IBoundary Bound>
		using Boundary = BoundarySelector<Bound>;

		template<typename prec>
		using UsedProperties = typename Properties::ParticlesProperties<prec>;

		template<typename prec>
		using NecessaryProvider = typename Provider::ParticleProvider<prec>;

		template<typename prec>
		using SimulationParameters = typename Parameters::ParticleSimulationParameters<prec>;

		template<typename prec>
		using ProblemSettings = typename Settings::NeelProblemSettings<prec>;

		template<typename prec>
		using InitSettings = typename Settings::ParticleSimulationInitSettings<prec>;

		using Dimension = Problems::NeelDimension;

		//Return Types
		template<typename Precision>
		using StochasticMatrixType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::SizeOfNoiseVector>;
		template<typename Precision>
		using DeterministicType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, 1>;
		template<typename Precision>
		using DependentType = DeterministicType<Precision>;
		template<typename Precision>
		using IndependentType = Eigen::Matrix<Precision, Dimension::NumberOfIndependentVariables, 1>;
		template<typename Precision>
		using NoiseType = Eigen::Matrix<Precision, Dimension::SizeOfNoiseVector, 1>;
		template<typename Precision>
		using JacobiMatrixType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::NumberOfDependentVariables>;
		template<typename Derived>
		using BaseMatrixType = Eigen::MatrixBase<Derived>;

		//Vector of the Data we want to store 
		template<typename Precision>
		using OutputType = DeterministicType<Precision>;
		//Allocator for STL containers
		template<typename Precision>
		using OutputTypeSTLAllocator =  std::allocator<OutputType<Precision>>;
	};
}

namespace Problems
{
	namespace detail
	{
		template<>
		struct has_ito_noise<Settings::IProblem::Problem_Neel> : public std::false_type {};
		template<>
		struct uses_boundaries<Settings::IProblem::Problem_Neel> : public std::false_type {};
		template<>
		struct is_magnetic_problem<Settings::IProblem::Problem_Neel> : public std::true_type {};
		template<>
		struct uses_magnetic_anisotropy<Settings::IProblem::Problem_Neel> : public std::true_type {};
	}

	template<typename prec, typename aniso>
	class SDEProblem_Traits<NeelRelaxation<prec, aniso>>
	{
	public:
		typedef prec																						    Precision;
		typedef aniso																						    Anisotropy;

		static constexpr Settings::IProblem EnumID = Settings::IProblem::Problem_Neel;
		using ProblemSelector = typename Selectors::template ProblemTypeSelector<EnumID>;

		using Dimension = typename ProblemSelector::Dimension;
		using IsIto = typename ProblemSelector::IsIto;

		// Necessary Vector Types
		using StochasticMatrixType = typename ProblemSelector::template StochasticMatrixType<prec>;
		using DeterministicType = typename ProblemSelector::template DeterministicType<prec>;
		using DependentType = typename ProblemSelector::template DependentType<prec>;
		using IndependentType = typename ProblemSelector::template IndependentType<prec>;
		using NoiseType = typename ProblemSelector::template NoiseType<prec>;
		using JacobiMatrixType = typename ProblemSelector::template JacobiMatrixType<prec>;

		template<typename T>
		using BaseMatrixType = typename ProblemSelector::template BaseMatrixType<T>;
		
		using UsedProperties = typename ProblemSelector::template UsedProperties<prec>;
		using NecessaryProvider = typename ProblemSelector::template NecessaryProvider<prec>;
		using SimulationParameters = typename ProblemSelector::template SimulationParameters<prec>;
		using ProblemSettings = typename ProblemSelector::template ProblemSettings<prec>;
		using InitSettings = typename ProblemSelector::template InitSettings<prec>;

		using OutputType = typename ProblemSelector::template OutputType<prec>;
		using OutputTypeSTLAllocator = typename ProblemSelector::template OutputTypeSTLAllocator<prec>;

	};
};

#endif	// INC_NeelRelaxation_Definitions_H
// end of Problems\Definitions\NeelRelaxation_Definitions.h
