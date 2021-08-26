///---------------------------------------------------------------------------------------------------
// file:		SimulationManagerTraits.h
//
// summary: 	Defines Basic Traits for the SimulationManager
//
// Copyright (c) 2019 Alexander Neumann.
//
// author: Alexander Neumann
// date: 04.03.2019


///---------------------------------------------------------------------------------------------------
#pragma once

//#include <SerAr/ConfigFile/ConfigFile_Archive.h>
#include <SerAr/AllArchiveIncludes.hpp>

//Paramter Includes
#include "archive/archive_preprocessing.hpp"


#include "arch/InstructionSets.hpp"
namespace SimulationApplication
{
    template<typename prec, MyCEL::SystemInfo::InstructionSet set = MyCEL::SystemInfo::InstructionSet::NONE>
    class SimulationManager;

    template<typename SimManager>
    class SimulationManagerTraits;

    template<typename prec>
    class SimulationManagerTraits<SimulationManager<prec>>
    {
    public:
        using StartInputArchive =  SerAr::file_input_archive_variants;			// Defines where to read the configs from
        using StartOutputArchive = SerAr::file_output_archive_variants;			// Defines where to write the configs to

        using ResultOutputArchive = Archives::IOutputArchive;

        using Parameters = Settings::SimulationManagerSettings<prec>;
    };
}
