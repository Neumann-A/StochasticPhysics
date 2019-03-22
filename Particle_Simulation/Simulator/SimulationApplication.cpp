/*
* Particle Simulation Application
* Author: Alexander Neumann
* Date : 23.08.2015
*/
#include <filesystem>

//We will not access floating point env.
//#pragma STDC FENV_ACCESS off
//#include <cfenv>
//#include <cerrno>

//Setup
#include "General/Setup.h"

//Command Options
#include "General/CommandOptions.h"

//Application
#include "Simulator/SimulationManager.h"

//Starting Archive
#include "ConfigFile_Archive/ConfigFile_Archive.h"

//Testincludes
#include "Settings/SystemMatrix_SimulationManagerSettings_Factory.h"

int main(int argc, char** argv)
{
	// Applications Definitions and Types
	using Application = SimulationApplication::SimulationManager<PREC>;
	using CmdOpts = CommandOptions<Application>;
	
	//TODO: Add informations to global parameters for easier access; 
	Logger::Log("Buildtime: " + std::string{ __TIME__ } + "\tBuilddate: " + __DATE__ );

	std::cout << "MATH_ERRNO is "
		<< MATH_ERRNO << '\n'
		<< "MATH_ERREXCEPT is "
		<< MATH_ERREXCEPT << '\n';

#ifdef _MSC_VER
	std::cout << "MSCV_LANG: " << _MSVC_LANG << '\n';
	std::cout << "FMA3 flag: " << _get_FMA3_enable() <<'\n';

#endif

	//Eigen::initParallel();
	
	//TODO: Put all those Informations into extra MAKRO switches and a global class
#ifdef USE_BOOST_RANDOM
	Logger::Log("Using BOOST Random!");
#endif
#ifdef USE_PCG_RANDOM
	Logger::Log("Using PCG Random!");
#endif
	Logger::Log("Used Instructions Sets in Eigen:" + std::string{ Eigen::SimdInstructionSetsInUse() });
	
	Logger::Log("Eigen_Comp_MSCV: " + std::to_string(EIGEN_COMP_MSVC) );
	Logger::Log("Eigen_Comp_LLVM: " + std::to_string(EIGEN_COMP_LLVM) );
	Logger::Log("Eigen_Comp_MINGW: " + std::to_string(EIGEN_COMP_MINGW) );

	const std::filesystem::path pathToExe{ argv[0] };
	const std::filesystem::path path{ pathToExe.parent_path() };

	Logger::Log("Path I run in: " + path.string() );
	GlobalParameters::Path = path;

	CmdOpts::registerOptions();
	CmdOpts::analyseCommandParameters(argc, argv);
	
	try
	{
		CmdOpts::InputArchive CFG{ CmdOpts::getInputArchive() };

		//Loading Application Parameters vom Archive
		Application::Parameters AppParams{ Archives::LoadConstructor<Application::Parameters>::construct(CFG) };

		Logger::Log("*********************Starting Application*********************");

		if (CmdOpts::useSystemMatrix) //TODO Remove this somehow
		{
			//Get Archive from CmdOpts
			CmdOpts::InputArchive CFGSysMat{ CmdOpts::getInputSystemMatrixArchive() };
					
			//Load Settings from archive
			Settings::SystemMatrixSettings<PREC> SysMatSettings;
			CFGSysMat(Archives::createNamedValue(SysMatSettings.getSectionName(),SysMatSettings));

			//Create the Parameters for the simulations
			auto simManSettingsVec = Settings::SystemMatrix_SimulationManagerSettings_Factory::template createSimulationManagerSettingsSystemMatrix<PREC>(AppParams, SysMatSettings);

			//Some numbers for progress tracking
			std::size_t counter{ 0 };
			std::size_t NoOfParams{ simManSettingsVec.size() };

			for (const auto& elem : simManSettingsVec)
			{
				std::stringstream tmp;

				const auto SimSettings = std::get<0>(elem);
				const auto& VoxelInfo = std::get<1>(elem);

				tmp << "Current Field: " << SimSettings.getFieldProperties().getAmplitudes().at(0).transpose() << std::endl;
				Logger::Log("Current File: " + SimSettings.getResultSettings().getFilepath().string());
				Logger::Log("Current Voxel: " + SimSettings.getResultSettings().getSaveFilepathSingle().string());
				Logger::Log(tmp);

				{ 
					//Creating Application
					Application SimManager{ SimSettings };

					//HACK: Silly hack to get the right progress in the job manager! (In the future replace with something more sophisticated)
					SimManager.ProgressFactor = static_cast<double>(1) / static_cast<double>(NoOfParams);
					SimManager.ProgressModifier = static_cast<double>(counter) / static_cast<double>(NoOfParams);

					//Starting Application
					SimManager.StartSimulationManager();

					//Wait until the simulation finished
					SimManager.waitUntilFinsihed();
				} //Destroy application to be able to save additional information!

				//Write additional Information to Result File. Application needs to be destroyed before! ():
				switch (SimSettings.getResultSettings().getFileType())
				{
				case Settings::IResultFileType::ResultFileType_MATLAB:
				{
					Archives::MatlabOutputArchive Ar{ SimSettings.getResultSettings().getFilepath(),Archives::MatlabOptions::update };
					Ar(Archives::createNamedValue(VoxelInfo.getSectionName(),VoxelInfo));
					Ar(Archives::createNamedValue(SysMatSettings.getSectionName(), SysMatSettings));
					break;
				}
				default:
				{
					Logger::Log("Voxelinformation saveable (Not supported archive)");
				};
				};

				Logger::Log(std::to_string(++counter) + " von " + std::to_string(simManSettingsVec.size()) + " Voxeln erledigt.");
			}

		}
		else
		{
			//Creating Application
			Application SimManager{ AppParams };

			//Starting Application
			SimManager.StartSimulationManager();

			//Wait until the simulation finished
			SimManager.waitUntilFinsihed();
		}
		Logger::Log("*********************Finishing Application*********************");
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
		Logger::Log("An unknown error occured somewhere! Please debug me!");
		throw; //Need to rethrow
	}
	//CFG.writeContentsToConsole(); // For Debugging
	//system("pause");
	return 0;
};
