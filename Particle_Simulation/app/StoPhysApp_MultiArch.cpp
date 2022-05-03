
//We will not access floating point env.
//#pragma STDC FENV_ACCESS off
//#include <cfenv>
//#include <cerrno>

//Setup
#include "General/Setup.h"

//Command Options
#include "General/CommandOptions.h"

#include "StoPhys_arch.hpp"

//Starting Archive
#include <SerAr/ConfigFile/ConfigFile_Archive.h>

//Testincludes
#include "Settings/SystemMatrix_SimulationManagerSettings_Factory.h"

#include "InputParams.h"


using AppTraits = SimulationApplication::SimulationManagerTraits<SimulationApplication::SimulationManager<PREC>>;
using AppInputParams = InputParams<AppTraits>;

template<MyCEL::SystemInfo::InstructionSet set>
void runAppWithArch(AppInputParams&& input)
{
    constexpr const std::string_view prelude = "Running with instruction set: ";
    const std::string_view arch = MyCEL::SystemInfo::to_string_view(MyCEL::SystemInfo::getInstructionSetMap(), set);
    std::string archstring{ std::string{prelude} + std::string{arch} + '\n' };
    std::puts(archstring.c_str());
    using Application = SimulationApplication::SimulationManager<PREC,set>;    
    Settings::SimulationManagerSettings<PREC> test{input.getAppParams()};
    Application SimManager{ test };

    SimManager.StartSimulationManager();
    SimManager.waitUntilFinsihed();
}

int main(int argc, char** argv)
{
    const std::filesystem::path pathToExe{ argv[0] };
    const std::filesystem::path path{ pathToExe.parent_path() };

    AppInputParams in_params(argc,argv);

    try
    {
        const auto arch = in_params.options.arch;
        switch(arch)
        { 
#ifdef WITH_NONE
        case MyCEL::SystemInfo::InstructionSet::NONE:
            runAppWithArch<MyCEL::SystemInfo::InstructionSet::NONE>(std::move(in_params));
            break;
#endif
#ifdef WITH_AVX
        case MyCEL::SystemInfo::InstructionSet::AVX:
            runAppWithArch<MyCEL::SystemInfo::InstructionSet::AVX>(std::move(in_params));
            break;
#endif
#ifdef WITH_AVX2
        case MyCEL::SystemInfo::InstructionSet::AVX2:
            runAppWithArch<MyCEL::SystemInfo::InstructionSet::AVX2>(std::move(in_params));
            break;
#endif
#ifdef WITH_AVX512
        case MyCEL::SystemInfo::InstructionSet::AVX512:
            runAppWithArch<MyCEL::SystemInfo::InstructionSet::AVX512>(std::move(in_params));
            break;
#endif
        default:
            std::puts("Architecture not supported by application. At least a CPU supporting AVX is required!\n");
            break;
        }

    }
    catch (std::runtime_error &e)
    {
        Logger::Log(e.what());
    }
    catch (std::exception &e)
    {
        Logger::Log(e.what());
    }
    catch (...)
    {
        Logger::Log("An unknown error occured somewhere! Please debug me!\n");
        throw; //Need to rethrow
    }

       return 0;
}
