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
        //typedef Settings::IProblemSettings<prec> ProblemSettings;
        using ProblemSettings = ProblemSettingsWrapper<prec>;
        typedef Settings::SimulationSettings<prec> SimulationSettings;

        SimulationSettings m_SimulationSettings;
        SolverSettings m_SolverSettings;
        ResultSettings m_ResultSettings;
        FieldProperties m_FieldProperties;
        ProblemSettings m_ProblemSettings {{IProblem::Problem_Neel},{}};

        //std::unique_ptr<ProblemSettings> _pProblemSettings{nullptr};
        std::unique_ptr<Provider> _pParticleProvider{nullptr};

    public:
        friend Archives::LoadConstructor<ThisClass>;

        typedef prec Precision;

        // Access the SolverParams
        const SolverSettings& getSolverSettings(void) const noexcept { return (m_SolverSettings); }
        void setSolverSettings(const SolverSettings& solverParams) { m_SolverSettings = solverParams; }

        // Access the ParticleProvider
        Provider& getProvider(void) const noexcept { return (*_pParticleProvider); }
        void setProvider(const Provider& particleProvider) { *_pParticleProvider = particleProvider; }

        // Access the ResultSettings
        const ResultSettings& getResultSettings(void) const noexcept { return (m_ResultSettings); }
        ResultSettings& getResultSettings(void) noexcept { return (m_ResultSettings); }

        void setResultSettings(const ResultSettings& resultSettings) noexcept { m_ResultSettings = resultSettings; }

        // Access the FieldProperties
        const FieldProperties& getFieldProperties(void) const noexcept { return (m_FieldProperties); }
        FieldProperties& getFieldProperties(void) noexcept { return (m_FieldProperties); }
        void setFieldProperties(const FieldProperties& fieldProperties) noexcept { m_FieldProperties = fieldProperties; }

        // Access the SimulationSettings
        const SimulationSettings& getSimulationSettings(void) const { return (m_SimulationSettings); }
        void setSimulationSettings(const SimulationSettings& simulationSettings)
        {
            m_SimulationSettings = simulationSettings;
        }

        // Access the ProblemSettings
        const ProblemSettings& getProblemSettings(void) const { return (m_ProblemSettings); }
        void setProblemSettings(const ProblemSettings& problemSettings)
        {
            m_ProblemSettings = problemSettings;
        }

        SimulationManagerSettings(const Provider& provider, const SimulationSettings& sim, const SolverSettings& solver,
                                  const ResultSettings& result, const ProblemSettings& problem,
                                  const FieldProperties& field)
            : m_SimulationSettings(sim)
            , m_SolverSettings(solver)
            , m_ResultSettings(result)
            , m_FieldProperties(field)
            , m_ProblemSettings(problem)
            , _pParticleProvider(std::make_unique<Provider>(provider))
        {
            if (m_SimulationSettings.getNumberOfSimulations() < _pParticleProvider->getNumberOfNecessarySimulations()) {
                Logger::Log(
                    "Warning: Number of performed Simulations is smaller than Number of provided Properties!\n");
            }
        }
        SimulationManagerSettings(const SimulationManagerSettings& tocopy)
            : m_SimulationSettings(tocopy.m_SimulationSettings)
            , m_SolverSettings(tocopy.m_SolverSettings)
            , m_ResultSettings(tocopy.m_ResultSettings)
            , m_FieldProperties(tocopy.m_FieldProperties)
            , m_ProblemSettings(tocopy.m_ProblemSettings)
            , _pParticleProvider(std::make_unique<Provider>(*tocopy._pParticleProvider)){};

        SimulationManagerSettings operator=(const SimulationManagerSettings& tocopy)
        {
            return SimulationManagerSettings{*tocopy._pParticleProvider, tocopy.m_SimulationSettings,
                                             tocopy.m_SolverSettings,     tocopy.m_ResultSettings,
                                             tocopy._pProblemSettings,  tocopy.m_FieldProperties};
        }

        static inline std::string getSectionName() { return std::string{"Simulation_Manager_Settings"}; };

        template <typename Archive>
        void save(Archive& ar) const
        {
            ar(Archives::createNamedValue(SolverSettings::getSectionName(), m_SolverSettings));
            ar(Archives::createNamedValue(ResultSettings::getSectionName(), m_ResultSettings));
            ar(Archives::createNamedValue(::Properties::Fields::General<prec>::getSectionName(), m_FieldProperties));
            ar(Archives::createNamedValue(SimulationSettings::getSectionName(), m_SimulationSettings));
            //ar(Archives::createNamedValue(ProblemSettings::getSectionName(), m_ProblemSettings));
            ar(Archives::createNamedValue(m_ProblemSettings));
            ar(Archives::createNamedValue(Provider::getSectionName(), *_pParticleProvider));
        }

        template <typename Archive>
        void load(Archive& ar)
        {
            ar(Archives::createNamedValue(SolverSettings::getSectionName(), m_SolverSettings));
            ar(Archives::createNamedValue(ResultSettings::getSectionName(), m_ResultSettings));
            ar(Archives::createNamedValue(::Properties::Fields::General<prec>::getSectionName(), m_FieldProperties));
            ar(Archives::createNamedValue(SimulationSettings::getSectionName(), m_SimulationSettings));
            ar(Archives::createNamedValue(m_ProblemSettings));
            //ar(::SerAr::createNamedEnumVariant("Type_of_problem",[](auto&& d){ return d.getSectionName()},m_ProblemSettings.variant)(),m_ProblemSettings));
            //_pProblemSettings = Archives::LoadConstructor<std::decay_t<decltype(*_pProblemSettings)>>::construct(ar);
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

            typename type::ProblemSettings ProblemSettings;
            std::unique_ptr<typename type::Provider> pParticleProvider{nullptr};

            ar(Archives::createNamedValue(type::SolverSettings::getSectionName(), SolverSettings));
            ar(Archives::createNamedValue(type::ResultSettings::getSectionName(), ResultSettings));
            ar(Archives::createNamedValue(::Properties::Fields::General<prec>::getSectionName(), FieldProperties));
            ar(Archives::createNamedValue(type::SimulationSettings::getSectionName(), SimulationSettings));
            ar(Archives::createNamedValue(ProblemSettings));
            //pProblemSettings = Archives::LoadConstructor<std::decay_t<decltype(*pProblemSettings)>>::construct(ar);

            pParticleProvider = std::make_unique<typename type::Provider>(
                Archives::LoadConstructor<typename type::Provider>::construct(ar));

            type ConstructedType{*pParticleProvider, SimulationSettings, SolverSettings,
                                 ResultSettings,     ProblemSettings,  FieldProperties};
            return ConstructedType;
        }
    };
} // namespace Archives

#endif // INC_SimulationManagerSettings_H
// end of SimulationManagerSettings.h
///---------------------------------------------------------------------------------------------------
