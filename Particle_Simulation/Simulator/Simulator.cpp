#include "Simulator.hpp"
#include "Simulator/Simulator_arch.hpp"

#include <cstdio>
#include "fmt/format.h"

//template<MyCEL::SystemInfo::InstructionSet set>
//STOPHYSIM_EXPORT std::unique_ptr<Simulator> createSimulator(Simulator::Parameters &params) {
//    return nullptr;
//};

STOPHYSIM_EXPORT std::unique_ptr<Simulator> createSimulator(MyCEL::SystemInfo::InstructionSet set, Simulator::Parameters &params) {
    try
    {
        switch(set)
        { 
#ifdef WITH_NONE
        case MyCEL::SystemInfo::InstructionSet::NONE:
            createSimulator<MyCEL::SystemInfo::InstructionSet::NONE>(params);
            break;
#endif
#ifdef WITH_AVX
        case MyCEL::SystemInfo::InstructionSet::AVX:
            createSimulator<MyCEL::SystemInfo::InstructionSet::AVX>(params);
            break;
#endif
#ifdef WITH_AVX2
        case MyCEL::SystemInfo::InstructionSet::AVX2:
            createSimulator<MyCEL::SystemInfo::InstructionSet::AVX2>(params);
            break;
#endif
#ifdef WITH_AVX512
        case MyCEL::SystemInfo::InstructionSet::AVX512:
            createSimulator<MyCEL::SystemInfo::InstructionSet::AVX512>(params);
            break;
#endif
        default:
            fmt::print("Requested Architecture '{}' not supported!",to_string(set));
            break;
        }

    }
    catch (std::runtime_error &e)
    {
        fmt::print("{}", e.what());
    }
    catch (std::exception &e)
    {
        fmt::print("{}", e.what());
    }
    catch (...)
    {
        fmt::print("An unknown error occured somewhere! Please debug me!\n");
        throw; //Need to rethrow
    }
};
