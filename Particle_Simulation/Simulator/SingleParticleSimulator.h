///---------------------------------------------------------------------------------------------------
// file:		SingleParticleSimulator.h
//
// summary: 	Declares the single particle simulator v 2 class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 26.06.2016
#pragma once
#ifndef INC_SingleParticleSimulator_H
#define INC_SingleParticleSimulator_H
///---------------------------------------------------------------------------------------------------

#include <vector>

#include <atomic>
#include <mutex>

#include "ISingleParticleSimulator.h" 
// We can use the interface / ABC approach here since the simulator is only called a few times to start and get the result of the simulation
// As this the pure virtual call to the interface class will not cost alot of performance !!! That is very nice

#include "Results/SingleSimulationResult.h"
#include "Results/MeanSimulationResult.h"

#include <MyCEL/basics/Timer.h>

template<typename T>
class SimulatorTraits;

namespace
{
    template <typename T>
    struct range_t
    {
        T b, e;
        constexpr range_t(T x, T y) : b(x), e(y) {}
        constexpr T begin()
        {
            return b;
        }
        constexpr T end()
        {
            return e;
        }
    };
    template <typename T>
    constexpr range_t<T> range(T b, T e)
    {
        return range_t<T>(b, e);
    }
}
//Task of this class; Simulate a particle with a given problem and field;
// Needs a Solver and a actual problem

template <typename solver, typename extfield>
class SingleParticleSimulator
{
public:
    using Solver = solver;
    using Field = extfield;
    using ThisClass = SingleParticleSimulator<Solver, Field>;

    using Traits = SimulatorTraits<ThisClass>;

    using Precision = typename Traits::Precision;

    using Problem = typename Traits::Problem;
                
    using FieldVectorList = typename Traits::FieldVectorList;
    using OutputVectorType = typename Traits::OutputVectorType;
    using OutputType = typename Traits::OutputVectorType::value_type;
    using StepList = typename Traits::StepList;
    
    struct ProblemParameters
    {
        typename Problem::UsedProperties		Properties;
        typename Problem::InitSettings			Init;
    };

private:
    using CalculationType = typename Traits::CalculationType;
    using SingleSimulationResult = typename Traits::SingleResultType;
    using MeanSimulationResult = typename Traits::MeanResultType;
    using SolverSettings = typename solver::Settings;

    Problem		m_problem;      //SDE describing the problem;
    Solver		m_solver;       //Used solver
    Field		m_field;        //Used external field
    const Precision m_timestep; //Used Timestep

    OutputVectorType	m_resvec;   //Vector with the results

    const uint64_t m_SID;           //SimulatorID
    Timer<std::chrono::high_resolution_clock, std::chrono::nanoseconds> m_Timer;

    ProblemParameters mProblemParameters;

    static std::atomic<uint64_t> mNumberOfActiveSimulators;
    static std::atomic<uint64_t> mNumberOfRunSimulation;
    static std::atomic<bool>	 mFieldandTimeCached;
    static std::mutex			 mFieldandTimeMutex;
    static FieldVectorList		 mFields;
    static StepList				 mTimes;

    void initSimulation(const uint64_t &NumberOfSteps, const uint64_t &OverSampling = 1)
    {
        if (NumberOfSteps % OverSampling != 0)
        {
            Log("OverSampling is not divider of NumberOfSteps. Last point in results will be wrong");
        }

        if (std::lock_guard<std::mutex> lg(mFieldandTimeMutex); !mFieldandTimeCached.exchange(true))
        {
            calculateTimeAndField(NumberOfSteps, OverSampling);
        }

        //Starting Clock
        m_Timer.start();

        //Allocating Memory for the result and init with zero!
        const std::size_t result_size = NumberOfSteps / OverSampling;
        m_resvec.resize(result_size, OutputType::Zero());
    };

    void startSimulation(const uint64_t& /*NumberOfSteps*/, const uint64_t &OverSampling = 1)
    {
        Log("Starting a new simulation");
        
        OutputType      sum{ OutputType::Zero() }; //Temporary result storage (for oversampling)
        OutputType      comp{ OutputType::Zero() };//Compensation storage for kahan summation
        CalculationType yi{ m_problem.getStart(mProblemParameters.Init) }; //Get starting point from problem
        m_resvec[0] = m_problem.calculateOutputResult(yi); //Write starting point into result vector

        std::size_t counter{ 0 }; //Counter to calculate the total time. 
        auto fieldLambda = [this](const Precision& time) { return m_field.getField(time); };
        for (auto& outputelem : range(m_resvec.begin()+1, m_resvec.end()))
        {
            for (auto l = OverSampling; l--;)
            {
                //std::cout << "yi: " << yi.transpose() << '\n';
                const auto time = m_timestep*((double)++counter); //Current total simulation time
                yi = this->m_solver.getResultNextFixedTimestep(time, yi, fieldLambda);

                {//Kahan summation for oversampling!
                    const OutputType y = m_problem.calculateOutputResult(yi) - comp;
                    const OutputType t = sum + y;
                    comp = (t - sum) - y;
                    sum = std::move(t);
                }
            }
            sum /= static_cast<Precision>(OverSampling);
            std::swap(outputelem, sum);
            //std::cout << "res: " << outputelem.transpose() << '\n';
        }
    };

    void stopSimulation(const uint64_t & /*NumberOfSteps*/, const uint64_t &OverSampling = 1)
    {
        //Stopping Clock
        //const auto time = (std::size_t)(m_Timer.stop());
        const auto time = static_cast<std::size_t>(m_Timer.stop());
        const auto timestr = std::to_string((double)time*m_Timer.unitFactor());
        const auto tperstepstr = std::to_string(time / (m_resvec.size()*OverSampling));
        Log("Finished Simulation after " + timestr + " s ("+ tperstepstr +" ns/step)");
    };

    void Log(std::string s1)
    {
        std::stringstream msg;
        msg << "Simulator " << m_SID << ": " << s1 <<'\n';
        Logger::Log(msg);
    };

public:
    //TODO: Move must be handled in another way!
    ALLOW_DEFAULT_MOVE_AND_ASSIGN(SingleParticleSimulator)
    
    explicit SingleParticleSimulator(const Problem &problem, const Field &field, const Precision &timestep, const SolverSettings& solverset) :
        m_problem(problem), // Copy the Problem
        m_solver(solverset, m_problem, timestep), //Link problem with Solver
        m_field(field),
        m_timestep(timestep),
        m_SID(++(SingleParticleSimulator::mNumberOfRunSimulation))
    {
        ++(SingleParticleSimulator::mNumberOfActiveSimulators);
    };

    explicit SingleParticleSimulator(const Problem &problem, const Field &field, const Precision &timestep, const SolverSettings& solverset, const ProblemParameters& ProbParams) :
        m_problem(problem), // Copy the Problem
        m_solver(solverset, m_problem, timestep), //Link problem with Solver
        m_field(field),
        m_timestep(timestep),
        m_resvec(),
        m_SID(++(SingleParticleSimulator::mNumberOfRunSimulation)),
        m_Timer(),
        mProblemParameters(ProbParams)
    {
        ++(SingleParticleSimulator::mNumberOfActiveSimulators);
    };

    ~SingleParticleSimulator()
    {
        --(SingleParticleSimulator::mNumberOfActiveSimulators);
        if (SingleParticleSimulator::mNumberOfActiveSimulators == 0)
        {
            resetClassStatics();
        }
    };

    static void resetClassStatics() noexcept
    {
        mNumberOfActiveSimulators = 0;
        mNumberOfRunSimulation = 0;
        mFieldandTimeCached = false;
    }

    bool doSimulation(const std::size_t &NumberOfSteps, const std::size_t &OverSampling)
    {
        this->initSimulation(NumberOfSteps, OverSampling);
        this->startSimulation(NumberOfSteps, OverSampling);
        this->stopSimulation(NumberOfSteps, OverSampling);
        return true;
    };

    SingleSimulationResult getSimulationResult()
    {
        SingleSimulationResult result{ mProblemParameters.Properties, std::move(m_resvec), m_problem.getWeighting(mProblemParameters.Properties), ThisClass::mTimes, ThisClass::mFields };
        
        return result;
    }

    void calculateTimeAndField(const std::size_t &NumberOfSteps, const std::size_t &OverSampling)
    {
        //FieldVectorList Fields;
        //StepList Times;

        //Allocating Memory for the result
        const std::size_t points{ NumberOfSteps / OverSampling };
        
        ThisClass::mFields.resize(points, Problem::IndependentType::Zero());
        ThisClass::mTimes.resize(points);

        typename Traits::FieldVector tmp{ Problem::IndependentType::Zero() }; // mean vector
                
        Precision start{ 0 };
        Precision end{ 0 };
        const Precision timeshift { m_timestep*(static_cast<Precision>(OverSampling))*0.5 } ;

        auto timeit = ThisClass::mTimes.begin();
        auto fieldit = ThisClass::mFields.begin();
        *timeit = start;                    //Startpunkt ist t = 0
        *fieldit = m_field.getField(start); //Feld bei t = 0;

        ++timeit; ++fieldit;

        for (std::size_t counter{ 0 }; fieldit != ThisClass::mFields.end() || timeit != ThisClass::mTimes.end(); ++fieldit, ++timeit)
        {
            start = end + m_timestep; //Neue Startzeit ist die alte Endzeit + ein zeitschritt
            for (std::size_t l = OverSampling; l--;)
            {
                tmp += m_field.getField(m_timestep*((double)++counter));
            }
            end = m_timestep*((double)counter) + start; //Endzeit (+ start gehoert logischerweise in die erste Zeile aber der compiler kann a*b+c mittels cpu befehl besser optimieren!)
            tmp /= static_cast<Precision>(OverSampling);
            std::swap(*fieldit, tmp);
            *timeit = (end - start) - timeshift;
        }

    }

};

template <typename solver, typename field>
std::atomic<uint64_t> SingleParticleSimulator<solver, field>::mNumberOfActiveSimulators = { 0 };

template <typename solver, typename field>
std::atomic<uint64_t> SingleParticleSimulator<solver, field>::mNumberOfRunSimulation = { 0 };

template <typename solver, typename field>
std::atomic<bool> SingleParticleSimulator<solver, field>::mFieldandTimeCached = { false };

template <typename Solver, typename Field>
typename SingleParticleSimulator<Solver, Field>::FieldVectorList SingleParticleSimulator<Solver, Field>::mFields = {};

template <typename Solver, typename Field>
typename SingleParticleSimulator<Solver, Field>::StepList SingleParticleSimulator<Solver, Field>::mTimes = {};

template <typename Solver, typename Field>
std::mutex	SingleParticleSimulator<Solver, Field>::mFieldandTimeMutex = {};




template <typename solver, typename field>
class SimulatorTraits<SingleParticleSimulator<solver,field>>
{
public:
    using SimulatorType = SingleParticleSimulator<solver, field>;
    using Solver = solver;
    using Field = field;
    using Problem = typename Solver::Problem;

    static_assert(std::is_same<typename Field::Precision, typename Solver::Precision>::value, "Cannot mix classes with different floating point precisions!");
    static_assert(std::is_same<typename Field::Precision, typename Problem::Precision>::value, "Cannot mix classes with different floating point precisions!");
    static_assert(std::is_same<typename Solver::Precision, typename Problem::Precision>::value, "Cannot mix classes with different floating point precisions!"); // For completness

    using Precision = typename Solver::Precision;

    using FieldProperties = typename Field::FieldProperties;
    using FieldVector = typename Field::FieldVector;
    using FieldAlloc = typename Field::Traits::FieldVectorStdAllocator;
    using FieldVectorList = std::vector<FieldVector, FieldAlloc>;

    using StepList = std::vector<Precision>;
    using CalculationType = typename Solver::ResultType;

    using OutputType = typename Problem::Traits::OutputType;
    using OutputTypeSTLAllocatpr = typename Problem::Traits::OutputTypeSTLAllocator;
    using OutputVectorType = std::vector<OutputType, OutputTypeSTLAllocatpr>;

    using SingleResultType = Results::SingleSimulationResult<SimulatorType>;
    using MeanResultType = Results::MeanSimulationResult<SimulatorType>;
};


#endif	// INC_SingleParticleSimulator_H
// end of SingleParticleSimulator.h
///---------------------------------------------------------------------------------------------------
