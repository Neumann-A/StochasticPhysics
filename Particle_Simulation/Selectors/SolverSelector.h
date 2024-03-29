///---------------------------------------------------------------------------------------------------
// file:		SolverSelector.h
//
// summary: 	Declares the solver selector class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.07.2016

#ifndef INC_SolverSelector_H
#define INC_SolverSelector_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "BasicSelector.h"
#include "Settings/SolverSettings.h"
#include "SDEFramework/Solver/SDESolvers.h"

#ifdef USE_PCG_RANDOM
#include <pcg_random.hpp>
#endif
#ifdef SIMD_NORMAL_DIST

#endif

namespace Selectors
{
#define SOLVERSLECTORMAKRO(E)
    using namespace Settings;
    using namespace SDE_Framework::Solvers;


    namespace {
#ifndef SIMD_NORMAL_DIST
#ifdef USE_BOOST_RANDOM 
#ifndef USE_PCG_RANDOM
        template<typename problem>
        using NoiseFieldType = NoiseField<typename problem::Precision, problem::Dimension::SizeOfNoiseVector, typename boost::random::mt19937_64>;
#else
        template<typename problem>
        using NoiseFieldType = NoiseField<typename problem::Precision, problem::Dimension::SizeOfNoiseVector, pcg64_k1024_fast>;
#endif
#elif defined(USE_PCG_RANDOM)
        template<typename problem>
        using NoiseFieldType = NoiseField<typename problem::Precision, problem::Dimension::SizeOfNoiseVector, pcg64_k1024_fast>;
#else
        template<typename problem>
        using NoiseFieldType = NoiseField<typename problem::Precision, problem::Dimension::SizeOfNoiseVector, std::mt19937_64>;// Describes the Random Noise Field
#endif
#else
        template<typename problem>
        using NoiseFieldType = NoiseField_SIMD<typename problem::Precision, problem::Dimension::SizeOfNoiseVector>;
#endif
    }
    ///-------------------------------------------------------------------------------------------------
    /// <summary>	General Template class for the following specialized SolverSelectors. 
    /// 			Mainly used to hold transform the enum from the paramters class into 
    /// 			the appriopiate classes and hold some general information.</summary>
    ///
    /// <seealso cref="T:BasicSelector{SolverSelector{ISolver::Solver_EulerMaruyama}}"/>
    ///-------------------------------------------------------------------------------------------------
    template<ISolver Solver>
    class SolverSelector : public BasicSelector<SolverSelector<Solver>>
    {
        /// <summary>	Defines an alias representing the usage of the drift correction between Ito to Stratonovich. </summary>
        //typedef	std::false_type					UsesDrift;

        /// <summary>	Defines an alias representing the uses of a double noise matrix. </summary>
        //typedef	std::false_type					UsesDoubleNoiseMatrix;
    };

    ///-------------------------------------------------------------------------------------------------
    /// <summary>	A solverselector for euler maruyama>. </summary>
    ///
    /// <seealso cref="T:BasicSelector{SolverSelector{ISolver::Solver_EulerMaruyama}}"/>
    ///-------------------------------------------------------------------------------------------------
    template<>
    class SolverSelector<ISolver::Solver_EulerMaruyama> : public BasicSelector<SolverSelector<ISolver::Solver_EulerMaruyama>>
    {
    public:
        typedef	std::true_type					UsesDrift;
        typedef	std::false_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

        template<typename problem,typename ...Args>
        using SolverType = EulerMaruyama<problem,NField<problem>>;
    };

    template<>
    class SolverSelector<ISolver::Solver_EulerMaruyamaNormalized> : public BasicSelector<SolverSelector<ISolver::Solver_EulerMaruyamaNormalized>>
    {
    public:
        typedef	std::true_type					UsesDrift;
        typedef	std::false_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

        template<typename problem, typename ...Args>
        using SolverType = EulerMaruyama_Normalized<problem, NField<problem>>;
    };

    template<>
    class SolverSelector<ISolver::Solver_Millstein> : public BasicSelector<SolverSelector<ISolver::Solver_Millstein>>
    {
    public:
        typedef	std::true_type					UsesDrift;
        typedef	std::false_type					UsesDoubleNoiseMatrix;
        
        template<typename problem>
        using NField = NoiseFieldType<problem>;

        template<typename problem, typename ...Args>
        using SolverType = Millstein<problem, NField<problem>>;
    };

    template<>
    class SolverSelector<ISolver::Solver_Heun_Strong> : public BasicSelector<SolverSelector<ISolver::Solver_Heun_Strong>>
    {
    public:
        typedef	std::true_type					UsesDrift;
        typedef	std::false_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

        template<typename problem, typename ...Args>
        using SolverType = Heun_Strong<problem, NField<problem>>;
    };

    template<>
    class SolverSelector<ISolver::Solver_Heun_NotConsistent> : public BasicSelector<SolverSelector<ISolver::Solver_Heun_NotConsistent>>
    {
    public:
        typedef	std::true_type					UsesDrift;
        typedef	std::false_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

        template<typename problem, typename ...Args>
        using SolverType = Heun_NotConsistent<problem, NField<problem>>;
    };


    using namespace SDE_Framework::Solvers;
    ///-------------------------------------------------------------------------------------------------
    /// <summary>	A solverslector for the explicit strong 1 0 solver. </summary>
    ///
    /// <seealso cref="T:BasicSelector{SolverSelector{ISolver::Solver_ExplicitStrong1_0}}"/>
    ///-------------------------------------------------------------------------------------------------
    template<>
    class SolverSelector<ISolver::Solver_ExplicitStrong1_0> : public BasicSelector<SolverSelector<ISolver::Solver_ExplicitStrong1_0>>
    {
        
    public:
        typedef	std::false_type					UsesDrift;
        typedef	std::true_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

#ifdef USE_BOOST_RANDOM 
#ifndef USE_PCG_RANDOM
            template<typename problem, int order>
        using DNMatrix = DoubleNoiseMatrix<typename problem::Precision, order, problem::Dimension::SizeOfNoiseVector, typename boost::random::mt19937_64>;
#else
        template<typename problem, int order>
        using DNMatrix = DoubleNoiseMatrix<typename problem::Precision, order, problem::Dimension::SizeOfNoiseVector, pcg64_k1024_fast>;
#endif
#elif defined(USE_PCG_RANDOM)
            template<typename problem, int order>
        using DNMatrix = DoubleNoiseMatrix<typename problem::Precision, order, problem::Dimension::SizeOfNoiseVector, pcg64_k1024_fast>;
#else
            template<typename problem,int order>
        using DNMatrix = DoubleNoiseMatrix<typename problem::Precision, order, problem::Dimension::SizeOfNoiseVector, std::mt19937_64>;// Describes the Random Noise Field
#endif

        template<typename problem, int order, typename ...Args>
        using SolverType = Explicit_Strong_1<problem, NField<std::decay_t<problem>>, DNMatrix<std::decay_t<problem>, order>>;
    };

    template<>
    class SolverSelector<ISolver::Solver_WeakTest> : public BasicSelector<SolverSelector<ISolver::Solver_WeakTest>>
    {

    public:
        typedef	std::false_type					UsesDrift;
        typedef	std::true_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

#ifdef USE_BOOST_RANDOM 
#ifndef USE_PCG_RANDOM
        template<typename problem, int order>
        using DNMatrix = DoubleNoiseMatrix<typename problem::Precision, order, problem::Dimension::SizeOfNoiseVector, typename boost::random::mt19937_64>;
#else
        template<typename problem, int order>
        using DNMatrix = DoubleNoiseMatrix<typename problem::Precision, order, problem::Dimension::SizeOfNoiseVector, pcg64_k1024_fast>;
#endif
#elif defined(USE_PCG_RANDOM)
        template<typename problem, int order>
        using DNMatrix = DoubleNoiseMatrix<typename problem::Precision, order, problem::Dimension::SizeOfNoiseVector, pcg64_k1024_fast>;
#else
        template<typename problem, int order>
        using DNMatrix = DoubleNoiseMatrix<typename problem::Precision, order, problem::Dimension::SizeOfNoiseVector, std::mt19937_64>;// Describes the Random Noise Field
#endif

        template<typename problem, int order, typename ...Args>
        using SolverType = WeakTest<problem, NField<std::decay_t<problem>>, DNMatrix<std::decay_t<problem>, order>>;
    };

    template<>
    class SolverSelector<ISolver::Solver_Implicit_Midpoint> : public BasicSelector<SolverSelector<ISolver::Solver_Implicit_Midpoint>>
    {

    public:
        typedef	std::false_type					UsesDrift;
        typedef	std::false_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

        template<typename problem, typename ...Args>
        using SolverType = Implicit_Midpoint<problem, NField<std::decay_t<problem>> >;
    };


#ifdef WITH_GSL_SOLVERS
    template<>
    class SolverSelector<ISolver::Solver_Implicit_Midpoint_GSL> : public BasicSelector<SolverSelector<ISolver::Solver_Implicit_Midpoint_GSL>>
    {

    public:
        typedef	std::false_type					UsesDrift;
        typedef	std::false_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

        template<typename problem, typename ...Args>
        using SolverType = Implicit_Midpoint_GSL<problem, NField<std::decay_t<problem>> >;
    };

    template<>
    class SolverSelector<ISolver::Solver_Implicit_Midpoint_GSL_Derivative_Free> : public BasicSelector<SolverSelector<ISolver::Solver_Implicit_Midpoint_GSL_Derivative_Free>>
    {

    public:
        typedef	std::false_type					UsesDrift;
        typedef	std::false_type					UsesDoubleNoiseMatrix;

        template<typename problem>
        using NField = NoiseFieldType<problem>;

        template<typename problem, typename ...Args>
        using SolverType = Implicit_Midpoint_GSL_Derivative_Free<problem, NField<std::decay_t<problem>> >;
    };

#endif
}

#endif	// INC_SolverSelector_H
// end of SolverSelector.h
///---------------------------------------------------------------------------------------------------
