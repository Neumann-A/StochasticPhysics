#pragma once

#include <MyCEL/basics/BasicMacros.h>

#include "arch/InstructionSets.hpp"

namespace SDE_Framework::Solvers {

    class GeneralSolver {
    DISALLOW_COPY_AND_ASSIGN(GeneralSolver)
    protected:
        GeneralSolver() noexcept();

    };

    template<MyCEL::SystemInfo::InstructionSet set>
    class GeneralSolver_ArchitectureDependent : public GeneralSolver {
    DISALLOW_COPY_AND_ASSIGN(GeneralSolver_ArchitectureDependent)
    protected:
        GeneralSolver_ArchitectureDependent() noexcept();
    };

    template<MyCEL::SystemInfo::InstructionSet set>
    std::unique_ptr<GeneralSolver> createSolver() {
        
    };

}