#pragma once

#include "Simulator/Simulator_arch.hpp"

template<MyCEL::SystemInfo::InstructionSet set>
STOPHYSIM_EXPORT std::unique_ptr<Simulator> createSimulator(Simulator::Parameters &params) {

    return nullptr;
}
