///---------------------------------------------------------------------------------------------------
// file:    ParticleSimulationSettings.h
//
// summary:     Declares the particle simulation settings class
//
//            Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 06.06.2016

#ifndef INC_ParticleSimulationSettings_H
#define INC_ParticleSimulationSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once
#include <memory>
#include <exception>

#include "Properties/ParticleProperties.h"
//#include "DistributionSettings.h"

#include <MyCEL/math/DistributionHelper.h>

#include <SerAr/Core/NamedValue.h>

namespace Settings
{
    template <typename prec>
    class ParticleSimulationSettings // Rename to ParticleDistributionSettings ?
    {
    private:
        using ThisClass                 = ParticleSimulationSettings<prec>;
        using ParticleProperties        = typename ::Properties::ParticlesProperties<prec>;
        using Vec3D                     = typename Eigen::Matrix<prec, 3, 1>;
        using AnisotropyDistribution    = typename ::Properties::ParticlesProperties<prec>::MagneticProperties::anisotropy_distribution_variant;

        //Use Relative Distributions Widths?
        bool                                            m_useRelativeMagneticRadiusDistributionWidth{ true };
        bool                                            m_useRelativeHydrodynamicShellDistributionWidth{ true };

        Distribution::IDistribution                                 m_magneticRadiusDistributionType{ Distribution::IDistribution::Distribution_delta };
        prec                                                        m_magneticRadiusDistributionWidth{ 0.0 };
        std::unique_ptr<Distribution::IDistributionHelper<prec>>    m_magRadiusDistHelper{ nullptr };

        Distribution::IDistribution                                 m_hydrodynamicShellDistributionType{ Distribution::IDistribution::Distribution_delta };
        prec                                                        m_hydrodynamicShellDistributionWidth{ 0.0 };
        std::unique_ptr<Distribution::IDistributionHelper<prec>>    m_hydroShellDistHelper{ nullptr };

public:
        AnisotropyDistribution anisotropyDistribution{};
private:
        void applyAnisotropyDistribution(ParticleProperties &ParProperties)
        {
            std::visit([&ParProperties](auto& dist) { 
                std::visit( [&dist](auto& anisotropy) {
                    if constexpr (std::is_same_v<typename std::decay_t<decltype(anisotropy)>::Distribution, std::decay_t<decltype(dist)>>)
                        anisotropy = dist.applyDistribution(anisotropy);
                    else
                        throw std::runtime_error{ "Anisotropy distribution type does not match anisotropy type!" };
                }, ParProperties.modMagneticProperties().Anisotropy.variant);
            }, anisotropyDistribution);
        };

        void applyMagneticRadiusDistribution(ParticleProperties &ParProperties)
        {
            if (m_magRadiusDistHelper == nullptr)
            {
                if(!m_useRelativeMagneticRadiusDistributionWidth)
                    initMagneticRadiusDists(ParProperties.getMagneticProperties().getMagneticRadius());
                else
                    initMagneticRadiusDists(1);
            }
            if (!m_useRelativeMagneticRadiusDistributionWidth)
                ParProperties.modMagneticProperties().setMagneticRadius(m_magRadiusDistHelper->getValueFromDistribution());
            else
                ParProperties.modMagneticProperties().setMagneticRadius(ParProperties.getMagneticProperties().getMagneticRadius()*m_magRadiusDistHelper->getValueFromDistribution());
        };

        void applyHydrodynamicShellDistribution(ParticleProperties &ParProperties)
        {
            prec shell = ParProperties.getHydrodynamicProperties().getHydrodynamicRadius() - ParProperties.getMagneticProperties().getMagneticRadius();
            if (m_hydroShellDistHelper == nullptr)
            {
                if(!m_useRelativeHydrodynamicShellDistributionWidth)
                    initHydrodynamicShellDists(shell);
                else
                    initHydrodynamicShellDists(1);
            }            
            if (!m_useRelativeHydrodynamicShellDistributionWidth)
                ParProperties.modHydrodynamicProperties().setHydrodynamicRadius(m_hydroShellDistHelper->getValueFromDistribution()+ ParProperties.getMagneticProperties().getMagneticRadius());
            else
                ParProperties.modHydrodynamicProperties().setHydrodynamicRadius(m_hydroShellDistHelper->getValueFromDistribution()*shell + ParProperties.getMagneticProperties().getMagneticRadius());
        };

        void initHydrodynamicShellDists(const prec& mean)
        {
            m_hydroShellDistHelper = ::Distribution::initDistribution(m_hydrodynamicShellDistributionType, mean, mean*m_hydrodynamicShellDistributionWidth);
        };

        void initMagneticRadiusDists(const prec& mean) 
        {
            m_magRadiusDistHelper = ::Distribution::initDistribution(m_magneticRadiusDistributionType, mean, mean*m_magneticRadiusDistributionWidth);
        };

    protected:
    public:
        ParticleSimulationSettings(AnisotropyDistribution aniso,
            bool UseMagDist, const Distribution::IDistribution& MagDist, const prec& MagWidth, bool UseHydroDist, const Distribution::IDistribution& HydroDist, const prec& HydroWidth)
        : m_useRelativeMagneticRadiusDistributionWidth(UseMagDist), m_useRelativeHydrodynamicShellDistributionWidth(UseHydroDist),
          m_magneticRadiusDistributionType(MagDist), m_magneticRadiusDistributionWidth(MagWidth),
          m_hydrodynamicShellDistributionType(HydroDist), m_hydrodynamicShellDistributionWidth(HydroWidth),
          anisotropyDistribution(aniso)
        {
            initDistributions();
        };

        ParticleSimulationSettings() = default;

        ParticleSimulationSettings(const ThisClass& Other) :
            m_useRelativeMagneticRadiusDistributionWidth(Other.m_useRelativeMagneticRadiusDistributionWidth),
            m_useRelativeHydrodynamicShellDistributionWidth(Other.m_useRelativeHydrodynamicShellDistributionWidth),
            m_magneticRadiusDistributionType(Other.m_magneticRadiusDistributionType),
            m_magneticRadiusDistributionWidth(Other.m_magneticRadiusDistributionWidth),
            m_hydrodynamicShellDistributionType(Other.m_hydrodynamicShellDistributionType),
            m_hydrodynamicShellDistributionWidth(Other.m_hydrodynamicShellDistributionWidth),
            anisotropyDistribution(Other.anisotropyDistribution)
        {
            initDistributions();
        }

        void initDistributions()
        {
            if (m_useRelativeMagneticRadiusDistributionWidth)
                initMagneticRadiusDists(1);

            if (m_useRelativeHydrodynamicShellDistributionWidth)
                initHydrodynamicShellDists(1);
        }

        ThisClass & operator=(ThisClass Other)
        {
            std::swap(*this, Other);
            return *this;
        }

        ParticleProperties applySettingsToParticleProperties(const ParticleProperties &ParProperties) 
        {
            ParticleProperties TmpPar{ ParProperties };
            applyAnisotropyDistribution(TmpPar);
            applyMagneticRadiusDistribution(TmpPar);
            applyHydrodynamicShellDistribution(TmpPar);
            return TmpPar;
        }
        
        static inline std::string getSectionName() { return std::string{ "Particle_Simulation_Settings" }; };

        template<typename Archive>
        void save(Archive &ar) const
        {
            std::visit([&ar](const auto& arg) { 
                ar(Archives::createNamedValue(::Properties::Anisotropy::Distribution<prec>::getSectionName(), arg ));
                }, anisotropyDistribution);

            std::string tmp;

            //Magnetic Radius
            ar(Archives::createNamedValue("Use_relative_distribution_width_for_magnetic_radius", m_useRelativeMagneticRadiusDistributionWidth));
            tmp = to_string(m_magneticRadiusDistributionType);
            ar(Archives::createNamedValue("Magnetic_radius_distribution_type", tmp));
            ar(Archives::createNamedValue("Magnetic_radius_distribution_width", m_magneticRadiusDistributionWidth));

            //Hydrodynamic Shell
            ar(Archives::createNamedValue("Use_relative_distribution_width_for_hydrodynamic_shell", m_useRelativeHydrodynamicShellDistributionWidth));
            tmp = to_string(m_hydrodynamicShellDistributionType);
            ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_type", tmp));
            ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_width", m_hydrodynamicShellDistributionWidth));
        }

        template<typename Archive>
        void load(Archive &ar)
        {
            //Magnetic Radius
            ar(Archives::createNamedValue("Use_relative_distribution_width_for_magnetic_radius", m_useRelativeMagneticRadiusDistributionWidth));
            std::string tmp { to_string(m_magneticRadiusDistributionType) };
            ar(Archives::createNamedValue("Magnetic_radius_distribution_type", tmp));
            m_magneticRadiusDistributionType = Distribution::from_string<Distribution::IDistribution>(tmp);
            ar(Archives::createNamedValue("Magnetic_radius_distribution_width", m_magneticRadiusDistributionWidth));

            //Hydrodynamic Shell
            ar(Archives::createNamedValue("Use_relative_distribution_width_for_hydrodynamic_shell", m_useRelativeHydrodynamicShellDistributionWidth));
            tmp = to_string(m_hydrodynamicShellDistributionType);
            ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_type", tmp));
            m_hydrodynamicShellDistributionType = Distribution::from_string<Distribution::IDistribution>(tmp);
            ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_width", m_hydrodynamicShellDistributionWidth));

            if (m_useRelativeMagneticRadiusDistributionWidth)
                initMagneticRadiusDists(1);

            if (m_useRelativeHydrodynamicShellDistributionWidth)
                initHydrodynamicShellDists(1);
        }
    };
}



#endif    // INC_ParticleSimulationSettings_H
// end of ParticleSimulationSettings.h
///---------------------------------------------------------------------------------------------------
