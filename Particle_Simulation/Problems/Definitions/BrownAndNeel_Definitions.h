#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Selectors/ProblemSelector.h"

#include "Settings/ProblemSettings.h"
#include "Properties/ParticleProperties.h"
#include "Provider/ParticleProvider.h"
#include "Settings/ParticleSimulationParameters.h"

namespace Selectors
{
	template <>
	class ProblemTypeSelector<Settings::IProblem::Problem_BrownAndNeel> : public BasicSelector<ProblemTypeSelector<Settings::IProblem::Problem_BrownAndNeel>>
	{
	public:
		typedef std::true_type		IsMagneticProblem;
		typedef std::true_type		UsesAnisotropy;
		typedef std::false_type		UsesBoundaryCondition;
		typedef std::false_type	    IsIto;

		using value_type = IProblem;
		static constexpr value_type value = Settings::IProblem::Problem_BrownAndNeel;

		template<typename prec, IAnisotropy AnisotropyID>
		using Anisotropy = typename AnisotropyTypeSelector<AnisotropyID>::template type<prec>;

		template<typename prec, IAnisotropy AnisotropyID, bool SimpleModel = false>
		using ProblemType = Problems::BrownAndNeelRelaxation<prec, Anisotropy<prec, AnisotropyID>, SimpleModel>;

		template<typename prec, IAnisotropy AnisotropyID, bool SimpleModel = false>
		using SDETraits = Problems::SDEProblem_Traits<Problems::BrownAndNeelRelaxation<prec, Anisotropy<prec, AnisotropyID>, SimpleModel>>;

		template<typename prec, IAnisotropy AnisotropyID, bool SimpleModel = false>
		using ProblemType_Select = Problems::BrownAndNeelRelaxation<prec, Anisotropy<prec, AnisotropyID>, SimpleModel>;

		template<IBoundary Bound>
		using Boundary = BoundarySelector<Bound>;

		template<typename prec>
		using UsedProperties = typename Properties::ParticlesProperties<prec>;

		template<typename prec>
		using NecessaryProvider = typename Provider::ParticleProvider<prec>;

		template<typename prec>
		using SimulationParameters = typename Parameters::ParticleSimulationParameters<prec>;

		template<typename prec>
		using ProblemSettings = typename Settings::BrownAndNeelProblemSettings<prec>;

		template<typename prec>
		using InitSettings = typename Settings::ParticleSimulationInitSettings<prec>;

		using Dimension = Problems::BrownAndNeelDimension;

		//Matrix and Vector Types
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
		using DependentVectorStdAllocator = Eigen::aligned_allocator<DependentType<Precision>>;
		template<typename Precision>
		using JacobiMatrixType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::NumberOfDependentVariables>;

		//Sub Matrix & Vector Types!
		template<typename Precision>
		using SubProblemMatrix = Eigen::Matrix<Precision, 3, 3>;
		template<typename Precision>
		using SubVector = Eigen::Matrix<Precision, 3, 1>;

		//Vector of the Data we want to store 
		template<typename Precision>
		using OutputType = Eigen::Matrix<Precision, 6, 1>; //Brown: x- and y- particle axis; Neel: Magnetisation Direction 
														   //Allocator for STL containers
		template<typename Precision>
		using OutputTypeSTLAllocator = Eigen::aligned_allocator<OutputType<Precision>>;

		//Base Parameter Type
		template<typename Derived>
		using BaseMatrixType = Eigen::MatrixBase<Derived>;
	};
}

namespace Problems
{
	template<typename prec, typename aniso,bool simplemodel>
	class SDEProblem_Traits<BrownAndNeelRelaxation<prec, aniso, simplemodel>>
	{
	public:
		typedef prec																						    Precision;
		typedef aniso																						    Anisotropy;

		static constexpr Settings::IProblem EnumID = Settings::IProblem::Problem_BrownAndNeel;

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

		//Sub Matrix & Vector Types!
		using SubProblemMatrix = typename ProblemSelector::template SubProblemMatrix<prec>;
		using SubVector = typename ProblemSelector::template SubVector<prec>;

	};
}