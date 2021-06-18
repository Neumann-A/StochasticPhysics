///---------------------------------------------------------------------------------------------------
// file:        ParticleSimulationParameters.h
//
// summary:     Declares the particle simulation parameters class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.06.2016

#ifndef INC_ParticleSimulationParameters_H
#define INC_ParticleSimulationParameters_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <SerAr/Core/NamedValue.h>

#include "Properties/ParticleProperties.h"
#include "ParticleSimulationInitSettings.h"
#include "ParticleSimulationSettings.h"

namespace Parameters
{
    template <typename prec>
    struct AnisotropyDistributionInit
    {
        using ParticleProperties = typename Properties::ParticlesProperties<prec>;
        using ParticleSimulationSettings = typename Settings::ParticleSimulationSettings<prec>;
        using AnisotropyDistribution = typename ParticleProperties::MagneticProperties::anisotropy_distribution_variant; 

        const ParticleProperties& ref_particle_properties;
        AnisotropyDistribution& ref_to_anisotropy_distribution;


        explicit AnisotropyDistributionInit(const ParticleProperties& props,AnisotropyDistribution& ani_dist) 
            : ref_particle_properties(props), ref_to_anisotropy_distribution(ani_dist) {};

        template <typename Archive>
        void serialize(Archive& ar)
        {
            ref_particle_properties.getMagneticProperties().serializeDistribution(ref_to_anisotropy_distribution,ar);
        }
    };

    ///-------------------------------------------------------------------------------------------------
    /// <summary>    Aggregation of all parameters needed for a simulation</summary>
    ///-------------------------------------------------------------------------------------------------
    template <typename prec>
    class ParticleSimulationParameters
    {
    private:
        typedef ParticleSimulationParameters<prec> ThisClass;

    public:
        using Precision                         = prec;
        using ParticleProperties                = Properties::ParticlesProperties<prec>;
        using ParticleSimulationSettings        = Settings::ParticleSimulationSettings<prec>;
        using ParticleSimulationInitialization  = Settings::ParticleSimulationInitSettings<prec>;

    private:
        ParticleProperties _particleProperties{};
        ParticleSimulationSettings _particleSimSettings{};
        ParticleSimulationInitialization _particleSimInit{};

    public:
        // Access the ParticleProperties
        const ParticleProperties& getParticleProperties(void) const noexcept { return (_particleProperties); }
        ParticleProperties& modParticleProperties(void) noexcept { return (_particleProperties); }
        void setParticleProperties(const ParticleProperties& particleProperties) noexcept
        {
            _particleProperties = particleProperties;
        }

        ParticleProperties getNewParticleProperties(void) noexcept
        {
            return (this->_particleSimSettings.applySettingsToParticleProperties(this->_particleProperties));
        }

        // Access the ParticleSimSettings
        const ParticleSimulationSettings& getParticleSimulationSettings(void) const noexcept
        {
            return (_particleSimSettings);
        }
        void setParticleSimulationSettings(const ParticleSimulationSettings& particleSimSettings) noexcept
        {
            _particleSimSettings = particleSimSettings;
        }

        // Access the ParticleSimInit
        const ParticleSimulationInitialization& getParticleSimulationInitialization(void) const noexcept
        {
            return (_particleSimInit);
        }
        void setParticleSimulationInitialization(const ParticleSimulationInitialization& particleSimInit) noexcept
        {
            _particleSimInit = particleSimInit;
        }

        ParticleSimulationParameters(const ParticleSimulationSettings& ParSimSet,
                                     const ParticleSimulationInitialization& ParSimInt,
                                     const ParticleProperties& ParProps) noexcept
            : _particleProperties(ParProps)
            , _particleSimSettings(ParSimSet)
            , _particleSimInit(ParSimInt){};
        ParticleSimulationParameters() = default;

        static inline std::string getSectionName() noexcept { return std::string{"Particle_Simulation_Parameters"}; };

        template <typename Archive>
        void serialize(Archive& ar)
        {
            ar(Archives::createNamedValue(ParticleProperties::getSectionName(), _particleProperties));
            ar(Archives::createNamedValue(ParticleSimulationInitialization::getSectionName(), _particleSimInit));

            ar(Archives::createNamedValue(ParticleSimulationSettings::getSectionName(), _particleSimSettings));

            // This is required to put the Distribution into the correct subfield
            // Would probably be better to move it into the serialize function of ParticleSimulationSettings somehow
            AnisotropyDistributionInit<prec> AniDistInit { _particleProperties, _particleSimSettings.anisotropyDistribution};
            ar(Archives::createNamedValue(ParticleSimulationSettings::getSectionName(), AniDistInit)); 

        }
    };
} // namespace Parameters

#endif // INC_ParticleSimulationParameters_H
// end of ParticleSimulationParameters.h
///---------------------------------------------------------------------------------------------------
