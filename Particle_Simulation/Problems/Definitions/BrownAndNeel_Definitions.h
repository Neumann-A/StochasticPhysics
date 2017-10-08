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

		//Sub Matrix & Vector Types!
		template<typename Precision>
		using SubProblemMatrix = Eigen::Matrix<Precision, 3, 3>;
		template<typename Precision>
		using SubVector = Eigen::Matrix<Precision, 3, 1>;
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

		typedef typename ProblemSelector::Dimension								    Dimension;
		typedef typename ProblemSelector::IsIto									    IsIto;

		// Necessary Vector Types
		typedef typename ProblemSelector::template StochasticMatrixType<prec>				StochasticMatrixType;
		typedef typename ProblemSelector::template DeterministicType<prec>			DeterministicType;
		typedef typename ProblemSelector::template DependentType<prec>				DependentType;
		typedef typename ProblemSelector::template IndependentType<prec>				IndependentType;
		typedef typename ProblemSelector::template NoiseType<prec>					NoiseType;

		// Necessary Paramter Types
		typedef typename ProblemSelector::template UsedProperties<prec>		     			UsedProperties;
		typedef typename ProblemSelector::template NecessaryProvider<prec>		     		NecessaryProvider;
		typedef typename ProblemSelector::template SimulationParameters<prec>		     	SimulationParameters;
		typedef typename ProblemSelector::template ProblemSettings<prec>		     		ProblemSettings;
		typedef typename ProblemSelector::template InitSettings<prec>		     			InitSettings;

		using DependentVectorStdAllocator = Eigen::aligned_allocator<DependentType>;

		//Sub Matrix & Vector Types!
		using SubProblemMatrix = typename ProblemSelector::template SubProblemMatrix<prec>;
		using SubVector = typename ProblemSelector::template SubVector<prec>;
	};
}