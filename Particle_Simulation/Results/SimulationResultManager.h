///---------------------------------------------------------------------------------------------------
// file:        SimulationResultManager.h
//
// summary:     Declares the simulation result manager class
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
#include <string>
#include <cstdint>
#include <condition_variable>

#include <MyCEL/basics/BasicMacros.h>

#include "SingleSimulationResult.h"
#include "MeanSimulationResult.h"

#include "Settings/ResultSettings.h"

#include "Settings/SimulationManagerSettings.h"

#include "meta/metainfo.hpp"

#include <SerAr/Core/NamedValue.h>
#include <SerAr/SerAr.hpp>

namespace Results
{
    template<typename Precision>
    class ISimulationResultManager
    {
    //protected:
    //    ISimulationResultManager() = default;
    public:
        ISimulationResultManager() {};
        virtual ~ISimulationResultManager() noexcept {};
        //MY_VIRTUAL_INTERFACE(ISimulationResultManager)
    protected:
        std::mutex                            _ResultMutex{};                    //! Mutex for the Results
        std::condition_variable               _ResultCond{};                    //! ResultCondition Variable!
    protected:
        std::size_t                           _ResultCounter{ 0 };
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

    template<typename Simulator, typename Precision = typename Simulator::Precision>
    class SimulationResultManager : public ISimulationResultManager<Precision>
    {
    private:
        using Base = ISimulationResultManager<Precision>;
    public:
        using SingleSimulationResult = ::Results::SingleSimulationResult<Simulator>;
        using MeanSimulationResult = ::Results::MeanSimulationResult<Simulator>;
    private:
    
        const Settings::ResultSettings                      mResultSettings;
        MeanSimulationResult                                mMeanResult{};
        bool                                                mFirstResult{ true };
        SerAr::AllFileOutputArchiveWrapper                  mSaveArchive;
        


    public:
        explicit SimulationResultManager(Settings::ResultSettings resparams)
            : mResultSettings(std::move(resparams)), mSaveArchive(mResultSettings.getFilepath(), SerAr::ArchiveOutputMode::Overwrite )
        {};

        virtual ~SimulationResultManager() override
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
            mSaveArchive(Archives::createNamedValue("Project_Information",ProjectMetaInfo{}));
        };

        void addSingleResult(SingleSimulationResult&& res)
        {
            std::unique_lock<std::mutex> lock(Base::_ResultMutex);
        
            const bool saveThis{ ((++Base::_ResultCounter % mResultSettings.getSaveInterval()) == 0) };
            if (mResultSettings.saveSingleSimulations() && saveThis)
            {
                auto opt{ createOptionsSingle() };

                if (mResultSettings.getSaveFilepathSingle() == mResultSettings.getFilepath())
                {
                    throw std::runtime_error{ "Writing SingleResults and MeanResult to same file currently not supported!" };
                }

                const std::string singlename = mResultSettings.getSingleFilePrefix() + "_" + BasicTools::toStringScientific(Base::_ResultCounter);

                const auto getsinglesavepath = [&]() {
                    if (mResultSettings.useExtraFileForSingleSimulations())
                    {
                        std::filesystem::path singlesavepath = mResultSettings.getSaveFilepathSingle();
                        const auto ext = singlesavepath.extension();
                        singlesavepath.replace_filename(singlename);
                        singlesavepath.replace_extension(ext);
                        return singlesavepath;
                    }
                    else
                    {
                        return mResultSettings.getSaveFilepathSingle();
                    }
                };
                SerAr::AllFileOutputArchiveWrapper ar{ getsinglesavepath() , opt };
                ar(Archives::createNamedValue(singlename, res));
            }
            mMeanResult += std::move(res);
        }

        auto createOptionsSingle() noexcept
        {
            if (mFirstResult  || mResultSettings.useExtraFileForSingleSimulations())
            {
                mFirstResult = false;
                return SerAr::ArchiveOutputMode::Overwrite;
            }
            return SerAr::ArchiveOutputMode::CreateOrAppend;
        }

        void addSingleSimulationResult(Results::ISingleSimulationResult&& irhs) override final
        {
            assert((dynamic_cast<SingleSimulationResult const *>(&irhs) != nullptr)); //Security check

            auto rhs = reinterpret_cast<SingleSimulationResult&>(irhs); //We use reinterpretcast here because it is save and we now the type! (and it is faster)

            addSingleResult(std::move(rhs));
        }
    };


}




#endif    // INC_SimulationResultManager_H
// end of SimulationResultManager.h
///---------------------------------------------------------------------------------------------------
