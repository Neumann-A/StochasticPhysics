#pragma once

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Selectors/ProblemSelector.h"
#include "Problems/NeelRelaxation.h"
#include "Settings/ProblemSettings.h"
#include "Properties/ParticleProperties.h"
#include "Provider/ParticleProvider.h"
#include "Settings/ParticleSimulationParameters.h"

namespace Selectors
{
	template <>
	class ProblemTypeSelector<IProblem::Problem_Neel> : public BasicSelector<ProblemTypeSelector<IProblem::Problem_Neel>>
	{
	public:
		typedef std::true_type		IsMagneticProblem;
		typedef std::true_type		UsesAnisotropy;
		typedef std::false_type		UsesBoundaryCondition;
		typedef std::false_type	    IsIto;

		typedef IProblem			valuetype;
		static constexpr valuetype value = { IProblem::Problem_Neel };

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
		using DeterministicVectorType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, 1>;
		template<typename Precision>
		using  DependentVectorType = DeterministicVectorType<Precision>;
		template<typename Precision>
		using  IndependentVectorType = Eigen::Matrix<Precision, Dimension::NumberOfIndependentVariables, 1>;
		template<typename Precision>
		using  NoiseVectorType = Eigen::Matrix<Precision, Dimension::SizeOfNoiseVector, 1>;

		template<typename Precision>
		using DependentVectorStdAllocator = Eigen::aligned_allocator<DependentVectorType<Precision>>;
	};
}
namespace Problems
{
	template<typename prec, typename aniso>
	class SDEProblem_Traits<NeelRelaxation<prec, aniso>>
	{
	public:
		typedef prec																						    Precision;
		typedef aniso																						    Anisotropy;

		static constexpr Settings::IProblem EnumID = Settings::IProblem::Problem_Neel;
		using ProblemSelector = typename Selectors::template ProblemTypeSelector<EnumID>;

		typedef typename ProblemSelector::Dimension								    Dimension;
		typedef typename ProblemSelector::IsIto									    IsIto;

		// Necessary Vector Types
		typedef typename ProblemSelector::template StochasticMatrixType<prec>			StochasticMatrixType;
		typedef typename ProblemSelector::template DeterministicVectorType<prec>		DeterministicVectorType;
		typedef typename ProblemSelector::template DependentVectorType<prec>			DependentVectorType;
		typedef typename ProblemSelector::template IndependentVectorType<prec>			IndependentVectorType;
		typedef typename ProblemSelector::template NoiseVectorType<prec>				NoiseVectorType;

		typedef typename ProblemSelector::template UsedProperties<prec>		     			UsedProperties;
		typedef typename ProblemSelector::template NecessaryProvider<prec>		     		NecessaryProvider;
		typedef typename ProblemSelector::template SimulationParameters<prec>		     	SimulationParameters;
		typedef typename ProblemSelector::template ProblemSettings<prec>		     		ProblemSettings;
		typedef typename ProblemSelector::template InitSettings<prec>		     			InitSettings;

		using DependentVectorStdAllocator = Eigen::aligned_allocator<DependentVectorType>;

	};
};