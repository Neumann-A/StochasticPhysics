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

#include "ConfigFile_Archive/ConfigFile_Archive.h"

//Paramter Includes
#include "Settings/SimulationManagerSettings.h"

namespace SimulationApplication
{
    template<typename prec>
    class SimulationManager;

    template<typename SimManager>
    class SimulationManagerTraits;

    template<typename prec>
    class SimulationManagerTraits<SimulationManager<prec>>
    {
    public:
        using StartInputArchive = typename Archives::ConfigFile_InputArchive;			// Defines where to read the configs from
        using StartOutputArchive = typename Archives::ConfigFile_OutputArchive;			// Defines where to write the configs to

        using ResultOutputArchive = typename Archives::IOutputArchive;

        using Parameters = Settings::SimulationManagerSettings<prec>;
    };
}
