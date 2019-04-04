///-------------------------------------------------------------------------------------------------
// file:	CommandOptions.h
//
// summary:	Declares the command options class
///-------------------------------------------------------------------------------------------------
#pragma once

#ifndef INC_COMMANDOPTIONS_H_
#define INC_COMMANDOPTIONS_H_

//The Template does nothing! Look at CPP to see specializations
template<typename App>
class CommandOptions
{
private:
protected:
	CommandOptions() = default;
public:
	using Application = App;
	using InputArchive = typename Application::StartInputArchive;
	using OutputArchive = typename Application::StartOutputArchive;

	static void registerOptions() {};
	static void analyseCommandParameters(const int /*argc*/, char** /*argv*/) {};
	static InputArchive getInputArchive();
	//Application getApplication() { return Application{ Application::Parameters{} }; };
};


/*
Kommandozeilenparameter-Kombinationen:
- keiner: "Example_Simulation_Settings.ini" ohne Multivoxel (bei Fehlen von -sysmatrix wird keine Bsp.-Systemmatrix erstellt und direkt angewandt)
- -parfile: Parameterfile ohne Multivoxel
- -sysmatrix: "Example_Simulation_Settings.ini" mit Multivoxel
- -parfile -sysmatrix: Parameterfile mit Multivoxel
*/

#include <string>
#include <memory>
#include <utility>

#include <Eigen/Core> // Required before including the Simulation Manager Traits

#include "General/GeneralDefines.h"
#include "basics/StartOptions.h"

//#include "Simulator/SimulationManager.h" // Should include everything!
#include "Simulator/SimulationManagerTraits.h"
template<>
class CommandOptions<SimulationApplication::SimulationManager<PREC>>
{
public:
	using Application = SimulationApplication::SimulationManager<PREC>;
    using ApplicationTraits = SimulationApplication::SimulationManagerTraits<Application>;
	using InputArchive = typename ApplicationTraits::StartInputArchive;
	using OutputArchive = typename ApplicationTraits::StartOutputArchive;

	//Statische variablen in einer rein statischen Klasse k�nnen gerne public sein. keine notwendigkeit f�r getter und setter!
	static bool useSystemMatrix;

private:
	static StartOptions StartOptions;
	static std::unique_ptr<InputArchive> pCFG_Input;
	static std::unique_ptr<InputArchive> pCFG_InputSysMat;

	// Simulation Parameter Option
	static void SimulationParametersLoad(std::string filename);
	static void SimulationParametersCreate();
	static void SimulationParametersRegister();
	
	//SystemMatrix Parameters
	static void SystemMatrixParametersLoad(std::string filename);
	static void SystemMatrixParametersCreate();
	static void SystemMatrixParametersRegister();

	//Help Option
	static void HelpLoad(std::string);
	static void RegisterHelp();

	//Registers all Options
	static void RegisterAll();

protected:
	//Make sure we do not create a instance of this class somewhere
	CommandOptions() = default;
	~CommandOptions() = default;
public:

	static void registerOptions()
	{
		RegisterAll();
	};
	static void analyseCommandParameters(const int argc, char** argv)
	{
		StartOptions.analyzeStartArguments(argc, argv);
	};
	static InputArchive getInputArchive()
	{
		return std::move(*pCFG_Input);
	};
	static InputArchive getInputSystemMatrixArchive()
	{
		return std::move(*pCFG_InputSysMat);
	};
};

#endif //_COMMANDOPTIONS_H_
