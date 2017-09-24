///---------------------------------------------------------------------------------------------------
// file:		Problems\Definitions\NeelRelaxationSpherical_Definitions.h
//
// summary: 	Declares the neel relaxation spherical definitions class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 20.06.2017

#ifndef INC_NeelRelaxationSpherical_Definitions_H
#define INC_NeelRelaxationSpherical_Definitions_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Selectors/ProblemSelector.h"
#include "Problems/NeelRelaxationSpherical.h"
#include "Settings/ProblemSettings.h"
#include "Properties/ParticleProperties.h"
#include "Provider/ParticleProvider.h"
#include "Settings/ParticleSimulationParameters.h"

namespace Selectors
{
	template <>
	class ProblemTypeSelector<IProblem::Problem_NeelSpherical> : public BasicSelector<ProblemTypeSelector<IProblem::Problem_NeelSpherical>>
	{
	public:
		typedef std::true_type		IsMagneticProblem;
		typedef std::true_type		UsesAnisotropy;
		typedef std::false_type		UsesBoundaryCondition;
		typedef std::false_type	    IsIto;

		typedef IProblem			valuetype;
		static constexpr valuetype value = { IProblem::Problem_NeelSpherical };

		template<typename prec, typename anisotropy>
		using ProblemType = Problems::NeelRelaxationSpherical<prec, anisotropy>;

		template<IAnisotropy Aniso>
		using Anisotropy = AnisotropyTypeSelector<Aniso>;

		template<typename prec, IAnisotropy AnisotropyID>
		using ProblemType_Select = Problems::NeelRelaxationSpherical<prec, typename Anisotropy<AnisotropyID>::template type<prec>>;

		template<IBoundary Bound>
		using Boundary = BoundarySelector<Bound>;

		template<typename prec>
		using UsedProperties = typename Properties::ParticlesProperties<prec>;

		template<typename prec>
		using NecessaryProvider = typename Provider::ParticleProvider<prec>;

		template<typename prec>
		using SimulationParameters = typename Parameters::ParticleSimulationParameters<prec>;

		template<typename prec>
		using ProblemSettings = typename Settings::NeelSphericalProblemSettings<prec>;

		template<typename prec>
		using InitSettings = typename Settings::ParticleSimulationInitSettings<prec>;

		using Dimension = Problems::NeelSphericalDimension;


		//Return Types
		template<typename Precision>
		using StochasticMatrixType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::SizeOfNoiseVector>;
		template<typename Precision>
		using DeterministicVectorType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, 1>;
		template<typename Precision>
		using  DependentVectorType = DeterministicVectorType<Precision>;
		template<typename Precision>
		using  IndependentVectorType = Eigen::Matrix<Precision, Dimension::NumberOfIndependentVariables, 1>;
		template<typename Precision>
		using  NoiseVectorType = Eigen::Matrix<Precision, Dimension::SizeOfNoiseVector, 1>;
		template<typename Precision>
		using JacobiMatrixType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::NumberOfDependentVariables>;
		template<typename Precision>
		using CoordinateTransformationType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::NumberOfIndependentVariables>;
		template<typename Derived>
		using BaseMatrixType = Eigen::MatrixBase<Derived>;

		template<typename Precision>
		using DependentVectorStdAllocator = Eigen::aligned_allocator<DependentVectorType<Precision>>;

		//Sub Matrix & Vector Types!
		template<typename Precision>
		using Matrix_3x3 = Eigen::Matrix<Precision, 3, 3>;
	};
}
namespace Problems
{
	template<typename prec, typename aniso>
	class SDEProblem_Traits<NeelRelaxationSpherical<prec, aniso>>
	{
	public:
		typedef prec																						    Precision;
		typedef aniso																						    Anisotropy;

		static constexpr Settings::IProblem EnumID = Settings::IProblem::Problem_NeelSpherical;
		using ProblemSelector = typename Selectors::template ProblemTypeSelector<EnumID>;

		typedef typename ProblemSelector::Dimension								    Dimension;
		typedef typename ProblemSelector::IsIto									    IsIto;

		// Necessary Vector Types
		typedef typename ProblemSelector::template StochasticMatrixType<prec>			StochasticMatrixType;
		typedef typename ProblemSelector::template DeterministicVectorType<prec>		DeterministicVectorType;
		typedef typename ProblemSelector::template DependentVectorType<prec>			DependentVectorType;
		typedef typename ProblemSelector::template IndependentVectorType<prec>			IndependentVectorType;
		typedef typename ProblemSelector::template NoiseVectorType<prec>				NoiseVectorType;
		typedef typename ProblemSelector::template JacobiMatrixType<prec>				JacobiMatrixType;
		typedef typename ProblemSelector::template CoordinateTransformationType<prec>	CoordinateTransformationType;

		template<typename T>
		using  BaseMatrixType = typename ProblemSelector::template BaseMatrixType<T>;

		typedef typename ProblemSelector::template UsedProperties<prec>		     			UsedProperties;
		typedef typename ProblemSelector::template NecessaryProvider<prec>		     		NecessaryProvider;
		typedef typename ProblemSelector::template SimulationParameters<prec>		     	SimulationParameters;
		typedef typename ProblemSelector::template ProblemSettings<prec>		     		ProblemSettings;
		typedef typename ProblemSelector::template InitSettings<prec>		     			InitSettings;

		using DependentVectorStdAllocator = Eigen::aligned_allocator<DependentVectorType>;
		using Matrix_3x3 = typename ProblemSelector::template Matrix_3x3<prec>;
	};
};

#endif	// INC_NeelRelaxationSpherical_Definitions_H
// end of Problems\Definitions\NeelRelaxationSpherical_Definitions.h
///---------------------------------------------------------------------------------------------------
