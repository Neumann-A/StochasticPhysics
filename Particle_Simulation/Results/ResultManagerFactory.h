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

#include "basics/Logger.h"

#include "MATLAB_Archive/Matlab_Archive.h"

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
				return std::make_unique<SimulationResultManager<Archives::MatlabOutputArchive, Simulator>>(Set);
			case Settings::IResultFileType::ResultFileType_HDF5:
			{
				Logger::Log("ResultManagerFactory: HDF5 Archives not yet supported!");
				return nullptr;
			}
			default:
			{
				Logger::Log("ResultManagerFactory: Unknown FileType not supported!");
				throw std::runtime_error("ResultManagerFactory: Unknown FileType. Cannot generate ResultManager");
			}
			}
		};

	};
}

#endif	// INC_ResultManagerFactory_H
// end of ResultManagerFactory.h
///---------------------------------------------------------------------------------------------------
