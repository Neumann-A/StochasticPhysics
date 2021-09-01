///---------------------------------------------------------------------------------------------------
// file:		ResultManagerFactory.h
//
// summary: 	Declares the result manager factory class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 16.01.2017

#ifndef INC_ResultManagerFactory_H
#define INC_ResultManagerFactory_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <memory>
#include <exception>
#include <MyCEL/basics/Logger.h>
#include <SerAr/SerAr.hpp>

#include "Settings/ResultSettings.h"
#include "Results/SimulationResultManager.h"

namespace Results
{
    class ResultManagerFactory
    {
    private:
        using ResultSettings = Settings::ResultSettings;
    public:
        
        template<typename Simulator, typename Precision = typename Simulator::Precision>
        static std::unique_ptr<ISimulationResultManager<Precision>> createResultManager(const ResultSettings &Set)
        {
            auto ptr = std::make_unique<SimulationResultManager<Simulator>>(Set);
            return ptr;
        }
    };
}

#endif	// INC_ResultManagerFactory_H
// end of ResultManagerFactory.h
///---------------------------------------------------------------------------------------------------
