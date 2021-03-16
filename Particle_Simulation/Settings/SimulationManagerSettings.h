///---------------------------------------------------------------------------------------------------
// file:        SimulationManagerSettings.h
//
// summary:     Declares the simulation manager settings class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 21.06.2016

#ifndef INC_SimulationManagerSettings_H
#define INC_SimulationManagerSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <SerAr/Core/LoadConstructor.h>
#include <SerAr/Core/NamedValue.h>

#include <MyCEL/basics/Logger.h>

#include <memory>

#include "ProblemSettings.h"
#include "Properties/FieldProperties.h"
#include "Provider/ParticleProvider.h"
#include "ResultSettings.h"
#include "SimulationSettings.h"
#include "SolverSettings.h"


namespace Settings
{

    template <typename prec>
    class SimulationManagerSettings
    {
    private:
        typedef SimulationManagerSettings<prec> ThisClass;
        typedef Settings::SolverSettings<prec> SolverSettings;
        typedef Provider::ParticleProvider<prec> Provider;
        typedef Settings::ResultSettings ResultSettings;
        typedef Properties::FieldProperties<prec> FieldProperties;
        typedef Settings::IProblemSettings<prec> ProblemSettings;
        typedef Settings::SimulationSettings<prec> SimulationSettings;

        SimulationSettings _SimulationSettings;
        SolverSettings _SolverSettings;
        ResultSettings _ResultSettings;
        FieldProperties _FieldProperties;

        std::unique_ptr<ProblemSettings> _pProblemSettings{nullptr};
        std::unique_ptr<Provider> _pParticleProvider{nullptr};

    public:
        friend Archives::LoadConstructor<ThisClass>;

        typedef prec Precision;

        // Access the SolverParams
        const SolverSettings& getSolverSettings(void) const noexcept { return (_SolverSettings); }
        void setSolverSettings(const SolverSettings& solverParams) { _SolverSettings = solverParams; }

        // Access the ParticleProvider
        Provider& getProvider(void) const noexcept { return (*_pParticleProvider); }
        void setProvider(const Provider& particleProvider) { *_pParticleProvider = particleProvider; }

        // Access the ResultSettings
        const ResultSettings& getResultSettings(void) const noexcept { return (_ResultSettings); }
        ResultSettings& getResultSettings(void) noexcept { return (_ResultSettings); }

        void setResultSettings(const ResultSettings& resultSettings) noexcept { _ResultSettings = resultSettings; }

        // Access the FieldProperties
        const FieldProperties& getFieldProperties(void) const noexcept { return (_FieldProperties); }
        FieldProperties& getFieldProperties(void) noexcept { return (_FieldProperties); }
        void setFieldProperties(const FieldProperties& fieldProperties) noexcept { _FieldProperties = fieldProperties; }

        // Access the SimulationSettings
        const SimulationSettings& getSimulationSettings(void) const { return (_SimulationSettings); }
        void setSimulationSettings(const SimulationSettings& simulationSettings)
        {
            _SimulationSettings = simulationSettings;
        }

        // Access the ProblemSettings
        const ProblemSettings& getProblemSettings(void) const { return (*_pProblemSettings); }
        void setProblemSettings(const std::unique_ptr<ProblemSettings>& problemSettings)
        {
            _pProblemSettings = problemSettings;
        }

        SimulationManagerSettings(const Provider& provider, const SimulationSettings& sim, const SolverSettings& solver,
                                  const ResultSettings& result, const ProblemSettings& problem,
                                  const FieldProperties& field)
            : _SimulationSettings(sim)
            , _SolverSettings(solver)
            , _ResultSettings(result)
            , _FieldProperties(field)
            , _pProblemSettings(problem.clone())
            , _pParticleProvider(std::make_unique<Provider>(provider))
        {
            if (_SimulationSettings.getNumberOfSimulations() < _pParticleProvider->getNumberOfNecessarySimulations()) {
                Logger::Log(
                    "Warning: Number of performed Simulations is smaller than Number of provided Properties!\n");
            }
        }
        SimulationManagerSettings(const SimulationManagerSettings& tocopy)
            : _SimulationSettings(tocopy._SimulationSettings)
            , _SolverSettings(tocopy._SolverSettings)
            , _ResultSettings(tocopy._ResultSettings)
            , _FieldProperties(tocopy._FieldProperties)
            , _pProblemSettings((*tocopy._pProblemSettings).clone())
            , _pParticleProvider(std::make_unique<Provider>(*tocopy._pParticleProvider)){};

        SimulationManagerSettings operator=(const SimulationManagerSettings& tocopy)
        {
            return SimulationManagerSettings{*tocopy._pParticleProvider, tocopy._SimulationSettings,
                                             tocopy._SolverSettings,     tocopy._ResultSettings,
                                             *tocopy._pProblemSettings,  tocopy._FieldProperties};
        }

        static inline std::string getSectionName() { return std::string{"Simulation_Manager_Settings"}; };

        template <typename Archive>
        void save(Archive& ar) const
        {
            ar(Archives::createNamedValue(SolverSettings::getSectionName(), _SolverSettings));
            ar(Archives::createNamedValue(ResultSettings::getSectionName(), _ResultSettings));
            ar(Archives::createNamedValue(::Properties::Fields::General<prec>::getSectionName(), _FieldProperties));
            ar(Archives::createNamedValue(SimulationSettings::getSectionName(), _SimulationSettings));
            ar(Archives::createNamedValue(ProblemSettings::getSectionName(), *_pProblemSettings));
            ar(Archives::createNamedValue(Provider::getSectionName(), *_pParticleProvider));
        }

        template <typename Archive>
        void load(Archive& ar)
        {
            ar(Archives::createNamedValue(SolverSettings::getSectionName(), _SolverSettings));
            ar(Archives::createNamedValue(ResultSettings::getSectionName(), _ResultSettings));
            ar(Archives::createNamedValue(::Properties::Fields::General<prec>::getSectionName(), _FieldProperties));
            ar(Archives::createNamedValue(SimulationSettings::getSectionName(), _SimulationSettings));

            _pProblemSettings = Archives::LoadConstructor<std::decay_t<decltype(*_pProblemSettings)>>::construct(ar);
            _pParticleProvider = std::make_unique<Provider>(Archives::LoadConstructor<Provider>::construct(ar));
        }
    };
} // namespace Settings

namespace Archives
{
    template <typename prec>
    class LoadConstructor<Settings::SimulationManagerSettings<prec>>
    {
    public:
        using type = Settings::SimulationManagerSettings<prec>;

        template <typename Archive>
        static inline type construct(InputArchive<Archive>& ar)
        {
            typename type::SimulationSettings SimulationSettings;
            typename type::SolverSettings SolverSettings;
            typename type::ResultSettings ResultSettings;
            typename type::FieldProperties FieldProperties;

            std::unique_ptr<typename type::ProblemSettings> pProblemSettings{nullptr};
            std::unique_ptr<typename type::Provider> pParticleProvider{nullptr};

            ar(Archives::createNamedValue(type::SolverSettings::getSectionName(), SolverSettings));
            ar(Archives::createNamedValue(type::ResultSettings::getSectionName(), ResultSettings));
            ar(Archives::createNamedValue(::Properties::Fields::General<prec>::getSectionName(), FieldProperties));
            ar(Archives::createNamedValue(type::SimulationSettings::getSectionName(), SimulationSettings));

            pProblemSettings = Archives::LoadConstructor<std::decay_t<decltype(*pProblemSettings)>>::construct(ar);

            pParticleProvider = std::make_unique<typename type::Provider>(
                Archives::LoadConstructor<typename type::Provider>::construct(ar));

            type ConstructedType{*pParticleProvider, SimulationSettings, SolverSettings,
                                 ResultSettings,     *pProblemSettings,  FieldProperties};
            return ConstructedType;
        }
    };
} // namespace Archives

#endif // INC_SimulationManagerSettings_H
// end of SimulationManagerSettings.h
///---------------------------------------------------------------------------------------------------
