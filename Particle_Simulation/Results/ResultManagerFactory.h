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
#include <SerAr/AllArchiveIncludes.h>

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
            switch (Set.getFileType())
            {
            case Settings::IResultFileType::ResultFileType_MATLAB:
            {
#ifdef SERAR_HAS_MATLAB
                auto ptr = std::make_unique<SimulationResultManager<Archives::MatlabOutputArchive, Simulator>>(Set);
                return std::move(ptr);
#else
                Logger::Log("ResultManagerFactory: MATLAB not support, because it was not compiled with MATLAB archive!\n");
                throw std::runtime_error("ResultManagerFactory: MATLAB not support, because it was not compiled with MATLAB archive!");
#endif
            }
            case Settings::IResultFileType::ResultFileType_HDF5:
            {
#ifdef SERAR_HAS_HDF5
                Logger::Log("ResultManagerFactory: HDF5 Archives not yet fully tested!\n");
                auto ptr = std::make_unique<SimulationResultManager<Archives::HDF5_OutputArchive, Simulator>>(Set);
                return std::move(ptr);
#else
                Logger::Log("ResultManagerFactory: HDF5 not support, because it was not compiled with HDF5 archive!\n");
                throw std::runtime_error("ResultManagerFactory: HDF5 not support, because it was not compiled with HDF5 archive!");
#endif

            } 
            case Settings::IResultFileType::ResultFileType_undefined:
                Logger::Log("ResultManagerFactory: Unknown FileType not supported!\n");
                throw std::runtime_error("ResultManagerFactory: Unknown FileType. Cannot generate ResultManager");
            default:
            {
                
                throw std::runtime_error("ResultManagerFactory: Unknown FileType. Cannot generate ResultManager");
            }
            }
        }

    };
}

#endif	// INC_ResultManagerFactory_H
// end of ResultManagerFactory.h
///---------------------------------------------------------------------------------------------------
