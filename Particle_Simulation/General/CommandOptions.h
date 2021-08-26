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
#include <filesystem>

#include <Eigen/Core> // Required before including the Simulation Manager Traits

#include "General/GeneralDefines.h"
#include <MyCEL/basics/StartOptions.h>

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

    static bool useSystemMatrix;

private:
    static StartOptions startOptions;
    // Simulation Parameter Option
    static void SimulationParametersLoad(std::string filestr);
    static void SimulationParametersCreate();
    static void SimulationParametersRegister();
    
    //SystemMatrix Parameters
    static void SystemMatrixParametersLoad(std::string filestr);
    static void SystemMatrixParametersCreate();
    static void SystemMatrixParametersRegister();

    //Help Option
    static void HelpLoad(std::string);
    static void RegisterHelp();

    //Registers all Options
    static void RegisterAll();

    static auto& input_archive()
    {
        static std::unique_ptr<InputArchive> input_archive{nullptr};
        return input_archive;
    }

    static auto& input_archive_sysmat()
    {
        static std::unique_ptr<InputArchive> input_archive_sysmat{nullptr};
        return input_archive_sysmat;
    }
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
        startOptions.analyzeStartArguments(argc, argv);
    };

    static auto& getInputArchive()
    {
        return *input_archive();
    };
    static auto& getInputSystemMatrixArchive()
    {
        return *input_archive_sysmat();
    };
};

#endif //_COMMANDOPTIONS_H_
