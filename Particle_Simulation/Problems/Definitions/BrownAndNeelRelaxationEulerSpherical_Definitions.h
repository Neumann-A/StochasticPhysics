///---------------------------------------------------------------------------------------------------
// file:		Problems\Definitions\BrownAndNeelRelaxationEulerSpherical_Definitions.h
//
// summary: 	Declares the brown and neel relaxation euler spherical definitions class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 17.10.2017

#ifndef INC_BrownAndNeelRelaxationEulerSpherical_Definitions_H
#define INC_BrownAndNeelRelaxationEulerSpherical_Definitions_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <type_traits>

#include <Eigen/Core>
//#include <Eigen/StdVector>

#include "GeneralProblem_Definitions.h"

#include "Selectors/ProblemSelector.h"
#include "Problems/BrownAndNeelRelaxationEulerSpherical.h"
#include "Settings/ProblemSettings.h"
#include "Properties/ParticleProperties.h"
#include "Provider/ParticleProvider.h"
#include "Settings/ParticleSimulationParameters.h"

namespace Selectors
{
    template <>
    class ProblemTypeSelector<IProblem::Problem_BrownAndNeelEulerSpherical> : public BasicSelector<ProblemTypeSelector<IProblem::Problem_BrownAndNeelEulerSpherical>>
    {
    public:
        typedef IProblem			valuetype;
        static constexpr valuetype value = { IProblem::Problem_BrownAndNeelEulerSpherical };

        using IsMagneticProblem = typename Problems::detail::template is_magnetic_problem<value>;
        using UsesAnisotropy = typename Problems::detail::template uses_magnetic_anisotropy<value>;
        using UsesBoundaryCondition = typename Problems::detail::template uses_boundaries<value>;
        using IsIto = typename Problems::detail::template has_ito_noise<value>;

        template<typename prec, typename anisotropy>
        using ProblemType = Problems::BrownAndNeelRelaxationEulerSpherical<prec, anisotropy>;

        template<IAnisotropy Aniso>
        using Anisotropy = AnisotropyTypeSelector<Aniso>;

        template<typename prec, IAnisotropy AnisotropyID>
        using ProblemType_Select = Problems::BrownAndNeelRelaxationEulerSpherical<prec, typename Anisotropy<AnisotropyID>::template type<prec>>;

        template<IBoundary Bound>
        using Boundary = BoundarySelector<Bound>;

        template<typename prec>
        using UsedProperties = typename Properties::ParticlesProperties<prec>;

        template<typename prec>
        using NecessaryProvider = typename Provider::ParticleProvider<prec>;

        template<typename prec>
        using SimulationParameters = typename Parameters::ParticleSimulationParameters<prec>;

        template<typename prec>
        using ProblemSettings = typename Settings::BrownAndNeelEulerSphericalProblemSettings<prec>;

        template<typename prec>
        using InitSettings = typename Settings::ParticleSimulationInitSettings<prec>;

        using Dimension = Problems::BrownAndNeelRelaxationEulerSphericalDimension;
        
        //Return Types
        template<typename Precision>
        using DeterministicType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, 1>;
        template<typename Precision>
        using DependentType = DeterministicType<Precision>;
        template<typename Precision>
        using IndependentType = Eigen::Matrix<Precision, Dimension::NumberOfIndependentVariables, 1>;
        template<typename Precision>
        using JacobiMatrixType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::NumberOfDependentVariables>;

        //Return Types for SDE descriptions
        template<typename Precision>
        using NoiseType = Eigen::Matrix<Precision, Dimension::SizeOfNoiseVector, 1>;
        template<typename Precision>
        using StochasticMatrixType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::SizeOfNoiseVector>;

        //Vector of the Data we want to store 
        template<typename Precision>
        using OutputType = Eigen::Matrix<Precision, 9, 1>; //Brown: x- and y- particle axis; Neel: Magnetisation Direction 
        //Allocator for STL containers
        template<typename Precision>
        using OutputTypeSTLAllocator =  std::allocator<OutputType<Precision>>;

        template<typename Precision>
        using CoordinateTransformationType = Eigen::Matrix<Precision, Dimension::NumberOfDependentVariables, Dimension::NumberOfIndependentVariables>;

        //Base Parameter Type
        template<typename Derived>
        using BaseMatrixType = Eigen::MatrixBase<Derived>;

        //Sub Matrix & Vector Types!
        template<typename Precision>
        using Matrix_3x3 = Eigen::Matrix<Precision, 3, 3>;
        template<typename Precision>
        using Matrix_2x3 = Eigen::Matrix<Precision, 2, 3>;
        template<typename Precision>
        using NeelDependentType = Eigen::Matrix<Precision, 2, 1>;
        template<typename Precision>
        using BrownDependentType = Eigen::Matrix<Precision, 3, 1>;
    };
}
namespace Problems
{
    namespace detail
    {
        template<>
        struct has_ito_noise<Settings::IProblem::Problem_BrownAndNeelEulerSpherical> : public std::false_type {};
        template<>
        struct uses_boundaries<Settings::IProblem::Problem_BrownAndNeelEulerSpherical> : public std::false_type {};
        template<>
        struct is_magnetic_problem<Settings::IProblem::Problem_BrownAndNeelEulerSpherical> : public std::true_type {};
        template<>
        struct uses_magnetic_anisotropy<Settings::IProblem::Problem_BrownAndNeelEulerSpherical> : public std::true_type {};
    }

    template<typename prec, typename aniso>
    class SDEProblem_Traits<BrownAndNeelRelaxationEulerSpherical<prec, aniso>>
    {
        friend BrownAndNeelRelaxationEulerSpherical<prec, aniso>;
    public:
        using Precision = prec;
        using Anisotropy = aniso;

        static constexpr Settings::IProblem EnumID = Settings::IProblem::Problem_BrownAndNeelEulerSpherical;
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
        using CoordinateTransformationType = typename ProblemSelector::template CoordinateTransformationType<prec>;

        template<typename T>
        using BaseMatrixType = typename ProblemSelector::template BaseMatrixType<T>;

        using UsedProperties = typename ProblemSelector::template UsedProperties<prec>;
        using NecessaryProvider = typename ProblemSelector::template NecessaryProvider<prec>;
        using SimulationParameters = typename ProblemSelector::template SimulationParameters<prec>;
        using ProblemSettings = typename ProblemSelector::template ProblemSettings<prec>;
        using InitSettings = typename ProblemSelector::template InitSettings<prec>;

        using OutputType = typename ProblemSelector::template OutputType<prec>;
        using OutputTypeSTLAllocator = typename ProblemSelector::template OutputTypeSTLAllocator<prec>;

        using Matrix_3x3 = typename ProblemSelector::template Matrix_3x3<prec>;
        using Matrix_2x3 = typename ProblemSelector::template Matrix_2x3<prec>;
        using BrownDependentType = typename ProblemSelector::template BrownDependentType<prec>;
        using NeelDependentType = typename ProblemSelector::template NeelDependentType<prec>;
    };
}

#endif	// INC_BrownAndNeelRelaxationEulerSpherical_Definitions_H
// end of Problems\Definitions\BrownAndNeelRelaxationEulerSpherical_Definitions.h
