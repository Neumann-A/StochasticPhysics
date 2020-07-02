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

	static const std::map<ISimulator, std::string> ISimulatorMap{ { { ISimulator::Simulator_undefined,"undefined" },{ ISimulator::Simulator_AllSingle,"AllSingle" },{ ISimulator::Simulator_AllStepwise ,"AllStepwise" } } };


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

		ISimulator _Simulator{ ISimulator::Simulator_undefined };
		prec _Timestep{ static_cast<prec>(1E-6) };
		std::size_t _NumberOfSteps{ 10000 };
		std::size_t _OverSampling{ 100 };
		std::size_t _NumberOfSimulators{ 4 };
		std::size_t _NumberOfSimulations{ 1 };

	protected:
	public:
		using Precision = prec;

		constexpr SimulationSettings(const ISimulator &sim, const prec &timestep, const std::size_t &NoSteps,const std::size_t &oversampling,const std::size_t &threads,const std::size_t &simulations)
			: _Simulator(sim), _Timestep(timestep), _NumberOfSteps(NoSteps), _OverSampling(oversampling),
			_NumberOfSimulators(threads), _NumberOfSimulations(simulations) {};
		SimulationSettings() {};

		// Access the Simulator
		inline const ISimulator& getSimulator(void) const noexcept { return(_Simulator); }
		inline const prec& getTimestep() const noexcept { return _Timestep; };
		inline const std::size_t& getNumberOfSteps() const noexcept { return _NumberOfSteps; };
		inline const std::size_t& getOverSampling() const noexcept { return _OverSampling; };
		inline const std::size_t& getNumberOfSimulations() const noexcept { return _NumberOfSimulations; };
		inline void setNumberOfSimulations(const std::size_t& sims) noexcept { _NumberOfSimulations = sims; };
		inline const std::size_t& getNumberOfSimulators() const noexcept { return _NumberOfSimulators; };
		inline void setNumberOfSimulators(const std::size_t& simulators ) noexcept { _NumberOfSimulators = simulators; };

		static inline std::string getSectionName() { return std::string{ "Simulation_Settings" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			ar(Archives::createNamedValue("Timestep", _Timestep));
			ar(Archives::createNamedValue("Number_of_steps", _NumberOfSteps));
			ar(Archives::createNamedValue("Oversampling", _OverSampling));
			ar(Archives::createNamedValue("Number_of_simulations", _NumberOfSimulations));
			ar(Archives::createNamedValue("Number_of_simulators", _NumberOfSimulators));

			std::string str{ to_string(_Simulator) };
			ar(Archives::createNamedValue(std::string{ "Simulator_type" }, str));
			_Simulator = from_string<decltype(_Simulator)>(str);
		}
	};

	template class SimulationSettings<float>;
	template class SimulationSettings<double>;
}

#endif	// INC_SimulationSettings_H
// end of SimulationSettings.h
///---------------------------------------------------------------------------------------------------
