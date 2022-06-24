#pragma once

#include <optional>

#include <MyCEL/basics/BasicMacros.h>

#include <boost/outcome/outcome.hpp>
namespace outcome = BOOST_OUTCOME_V2_NAMESPACE;

#include "stophysim_export.h"
#include "arch/InstructionSets.hpp"

template<typename prec>
class SimulationManagerSettings;

class Simulator {
protected:
    Simulator() = default;
public:
    class Output;
    class Parameters {
        SimulationManagerSettings<double> *manager_set;
    };
    virtual ~Simulator() = default;
    virtual void setupSimulation(const Parameters &settings) = 0;
    virtual bool performSimulation(const std::uint64_t &NumberOfSteps, const std::uint64_t &OverSampling) = 0;
    virtual void resetSimulation() = 0;
    virtual Output getResults() = 0;
};

//template<MyCEL::SystemInfo::InstructionSet set>
//class Simulator_ArchDependent : public Simulator {
//
//};

STOPHYSIM_EXPORT std::unique_ptr<Simulator> createSimulator(MyCEL::SystemInfo::InstructionSet set, Simulator::Parameters &params);


