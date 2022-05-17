///---------------------------------------------------------------------------------------------------
// file:		SimulationSettings.h
//
// summary: 	Declares the simulation settings class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 25.06.2016

#ifndef INC_SimulationSettings_H
#define INC_SimulationSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <SerAr/Core/NamedValue.h>

#include <map>

namespace Settings
{
    enum class ISimulator { Simulator_undefined, Simulator_AllSingle, Simulator_AllStepwise };

    extern const std::map<ISimulator, std::string> ISimulatorMap;

    std::string to_string(const ISimulator&);

    template<typename T>
    T from_string(const std::string&);
    template<>
    ISimulator from_string<ISimulator>(const std::string&);

    template <typename prec>
    class SimulationSettings
    {
    private:
        using ThisClass = SimulationSettings<prec>;

        ISimulator m_Simulator{ ISimulator::Simulator_undefined };
        prec m_Timestep{ static_cast<prec>(1E-6) };
        std::size_t m_NumberOfSteps{ 10000 };
        std::size_t m_OverSampling{ 100 };
        std::size_t m_NumberOfSimulators{ 4 };
        std::size_t m_NumberOfSimulations{ 1 };

    protected:
    public:
        using Precision = prec;

        constexpr SimulationSettings(const ISimulator &sim, const prec &timestep, const std::size_t &NoSteps,const std::size_t &oversampling,const std::size_t &threads,const std::size_t &simulations)
            : m_Simulator(sim), m_Timestep(timestep), m_NumberOfSteps(NoSteps), m_OverSampling(oversampling),
            m_NumberOfSimulators(threads), m_NumberOfSimulations(simulations) {};
        SimulationSettings() {};

        // Access the Simulator
        inline const ISimulator& getSimulator(void) const noexcept { return(m_Simulator); }
        inline const prec& getTimestep() const noexcept { return m_Timestep; };
        inline const std::size_t& getNumberOfSteps() const noexcept { return m_NumberOfSteps; };
        inline const std::size_t& getOverSampling() const noexcept { return m_OverSampling; };
        inline const std::size_t& getNumberOfSimulations() const noexcept { return m_NumberOfSimulations; };
        inline void setNumberOfSimulations(const std::size_t& sims) noexcept { m_NumberOfSimulations = sims; };
        inline const std::size_t& getNumberOfSimulators() const noexcept { return m_NumberOfSimulators; };
        inline void setNumberOfSimulators(const std::size_t& simulators ) noexcept { m_NumberOfSimulators = simulators; };

        static inline std::string getSectionName() { return std::string{ "Simulation_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Timestep", m_Timestep));
            ar(Archives::createNamedValue("Number_of_steps", m_NumberOfSteps));
            ar(Archives::createNamedValue("Oversampling", m_OverSampling));
            ar(Archives::createNamedValue("Number_of_simulations", m_NumberOfSimulations));
            ar(Archives::createNamedValue("Number_of_simulators", m_NumberOfSimulators));

            std::string str{ to_string(m_Simulator) };
            ar(Archives::createNamedValue(std::string{ "Simulator_type" }, str));
            m_Simulator = from_string<decltype(m_Simulator)>(str);
        }
    };

    template class SimulationSettings<float>;
    template class SimulationSettings<double>;
}

#endif	// INC_SimulationSettings_H
// end of SimulationSettings.h
///---------------------------------------------------------------------------------------------------
