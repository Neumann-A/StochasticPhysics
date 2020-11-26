///-------------------------------------------------------------------------------------------------
// file:	SimulationSettings.cpp
//
// summary:	Implements the simulation settings class
///-------------------------------------------------------------------------------------------------
#include "SimulationSettings.h"

namespace Settings
{
    const std::map<ISimulator, std::string> ISimulatorMap = { { { ISimulator::Simulator_undefined,"undefined" },
                                                                { ISimulator::Simulator_AllSingle,"AllSingle" },
                                                                { ISimulator::Simulator_AllStepwise ,"AllStepwise" } } };

    std::string to_string(const ISimulator& field)
    {
        return ISimulatorMap.at(field);
    };

    template<>
    ISimulator from_string<ISimulator>(const std::string &String)
    {
        for (auto it : ISimulatorMap)
            if (it.second == String)
                return it.first;

        throw std::runtime_error{ std::string{ "SimulationSettings: Type of Simulator unknown! " } +String };
    };
}
