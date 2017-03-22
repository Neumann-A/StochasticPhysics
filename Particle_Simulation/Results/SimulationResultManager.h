///---------------------------------------------------------------------------------------------------
// file:		SimulationResultManager.h
//
// summary: 	Declares the simulation result manager class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 22.10.2016

#ifndef INC_SimulationResultManager_H
#define INC_SimulationResultManager_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <iosfwd>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "basics/BasicMacros.h"

#include "SingleSimulationResult.h"
#include "MeanSimulationResult.h"

#include "Settings/ResultSettings.h"

#include "Settings/SimulationManagerSettings.h"

#include "Archive/NamedValue.h"

namespace Results
{
	template<typename Precision>
	class ISimulationResultManager
	{
		MY_VIRTUAL_INTERFACE(ISimulationResultManager)
	protected:
		std::mutex							_ResultMutex;					//! Mutex for the Results
		std::condition_variable				_ResultCond;					//! ResultCondition Variable!
	protected:
		std::size_t							_ResultCounter{ 0 };
	public:

		void waitUntilFinished(const std::size_t& NoOfResults)
		{
			std::unique_lock<std::mutex> lck(_ResultMutex);
			_ResultCond.wait(lck, [this,&NoOfResults] {return this->isFinished(NoOfResults); });
		}

		bool isFinished(const std::size_t& NoOfResults) const
		{
			const bool test{ (NoOfResults <= _ResultCounter) };
			return test;
		}

		virtual void addSingleSimulationResult(Results::ISingleSimulationResult&&) = 0;
		virtual void writeSimulationManagerSettings(const Settings::SimulationManagerSettings<Precision>& params) = 0;
		void finish()
		{
			_ResultCond.notify_all();
		}
	};

	template<typename Archive, typename Simulator, typename Precision = typename Simulator::Precision>
	class SimulationResultManager : public ISimulationResultManager<Precision>
	{
	private:
		using Base = ISimulationResultManager<Precision>;
	public:
		using SingleSimulationResult = SingleSimulationResult<Simulator>;
		using MeanSimulationResult = MeanSimulationResult<Simulator>;
		using UsedArchive = Archive;
	private:
	
		const Settings::ResultSettings						mResultSettings;
		MeanSimulationResult								mMeanResult;
		bool												mFirstResult{ true };
		Archive												mSaveArchive;
		


	public:
		explicit SimulationResultManager(Settings::ResultSettings resparams) 
			: mResultSettings(std::move(resparams)), mSaveArchive(mResultSettings.getFilepath(), Archives::MatlabOptions::write_v73)
		{

		};

		~SimulationResultManager()
		{
			try
			{
				mSaveArchive(Archives::createNamedValue("Mean_Results", mMeanResult));
			}
			catch (...)
			{
				std::cerr << "SimulationResultManager: Saving the data failed for some reason!" << std::endl;
			}
		};

		void writeSimulationManagerSettings(const Settings::SimulationManagerSettings<Precision>& params) override final
		{
			mSaveArchive(Archives::createNamedValue(Settings::SimulationManagerSettings<Precision>::getSectionName(),params));
		};

		void addSingleResult(SingleSimulationResult&& res)
		{
			{
				std::unique_lock<std::mutex> lock(Base::_ResultMutex);
			
				const bool saveThis{ ((++Base::_ResultCounter % mResultSettings.getSaveInterval()) == 0) };
				if (mResultSettings.saveSingleSimulations() && saveThis)
				{
					Archives::MatlabOptions opt{ Archives::MatlabOptions::update };
					if (mFirstResult)
					{
						mFirstResult = false;
						opt = Archives::MatlabOptions::write_v73;
					}

					if (mResultSettings.getSaveFilepathSingle() == mResultSettings.getFilepath())
					{
						throw std::runtime_error{ "Writing SingleResults and MeanResult to same file currently not supported!" };
					}

						Archive ar{ mResultSettings.getSaveFilepathSingle(), opt };
						const std::string name = mResultSettings.getSingleFilePrefix() + "_" + BasicTools::toStringScientific(Base::_ResultCounter);
						ar(Archives::createNamedValue(name, res));
				}
				mMeanResult += std::move(res);
			}
		}

		void addSingleSimulationResult(Results::ISingleSimulationResult&& irhs) override final
		{
			assert((dynamic_cast<SingleSimulationResult const *>(&irhs) != nullptr)); //Security check

			auto rhs = reinterpret_cast<SingleSimulationResult&>(irhs); //We use reinterpretcast here because it is save and we now the type! (and it is faster)

			addSingleResult(std::move(rhs));
		}
	};


}




#endif	// INC_SimulationResultManager_H
// end of SimulationResultManager.h
///---------------------------------------------------------------------------------------------------
