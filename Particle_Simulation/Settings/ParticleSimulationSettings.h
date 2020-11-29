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
        bool                                            _useRelativeMagneticRadiusDistributionWidth{ true };
        bool                                            _useRelativeHydrodynamicShellDistributionWidth{ true };

        Distribution::IDistribution                                 _magneticRadiusDistributionType{ Distribution::IDistribution::Distribution_lognormal };
        prec                                                        _magneticRadiusDistributionWidth{ 0 };
        std::unique_ptr<Distribution::IDistributionHelper<prec>>    _magRadiusDistHelper{ nullptr };

        Distribution::IDistribution                                 _hydrodynamicShellDistributionType{ Distribution::IDistribution::Distribution_lognormal };
        prec                                                        _hydrodynamicShellDistributionWidth{ 0 };
        std::unique_ptr<Distribution::IDistributionHelper<prec>>    _hydroShellDistHelper{ nullptr };

public:
        AnisotropyDistribution anisotropyDistribution;
private:
        void applyAnisotropyDistribution(ParticleProperties &ParProperties)
        {
            std::visit([&ParProperties](auto& dist) { 
                std::visit([&dist](auto& anisotropy) {
                    if constexpr (std::is_same_v<std::decay_t<decltype(anisotropy)>::Distribution, std::decay_t<decltype(dist)>>)
                        anisotropy = dist.applyDistribution(anisotropy);
                    else
                        throw std::runtime_error{ "Anisotropy distribution type does not match anisotropy type!" };
                }, ParProperties.modMagneticProperties().AnisotropyProperties);
            }, anisotropyDistribution);
        };

        void applyMagneticRadiusDistribution(ParticleProperties &ParProperties)
        {
            if (_magRadiusDistHelper == nullptr)
            {
                if(!_useRelativeMagneticRadiusDistributionWidth)
                    initMagneticRadiusDists(ParProperties.getMagneticProperties().getMagneticRadius());
                else
                    initMagneticRadiusDists(1);
            }
            if (!_useRelativeMagneticRadiusDistributionWidth)
                ParProperties.modMagneticProperties().setMagneticRadius(_magRadiusDistHelper->getValueFromDistribution());
            else
                ParProperties.modMagneticProperties().setMagneticRadius(ParProperties.getMagneticProperties().getMagneticRadius()*_magRadiusDistHelper->getValueFromDistribution());
        };

        void applyHydrodynamicShellDistribution(ParticleProperties &ParProperties)
        {
            prec shell = ParProperties.getHydrodynamicProperties().getHydrodynamicRadius() - ParProperties.getMagneticProperties().getMagneticRadius();
            if (_hydroShellDistHelper == nullptr)
            {
                if(!_useRelativeHydrodynamicShellDistributionWidth)
                    initHydrodynamicShellDists(shell);
                else
                    initHydrodynamicShellDists(1);
            }            
            if (!_useRelativeHydrodynamicShellDistributionWidth)
                ParProperties.modHydrodynamicProperties().setHydrodynamicRadius(_hydroShellDistHelper->getValueFromDistribution()+ ParProperties.getMagneticProperties().getMagneticRadius());
            else
                ParProperties.modHydrodynamicProperties().setHydrodynamicRadius(_hydroShellDistHelper->getValueFromDistribution()*shell + ParProperties.getMagneticProperties().getMagneticRadius());
        };

        void initHydrodynamicShellDists(const prec& mean)
        {
            _hydroShellDistHelper = ::Distribution::initDistribution(_hydrodynamicShellDistributionType, mean, mean*_hydrodynamicShellDistributionWidth);
        };

        void initMagneticRadiusDists(const prec& mean) 
        {
            _magRadiusDistHelper = ::Distribution::initDistribution(_magneticRadiusDistributionType, mean, mean*_magneticRadiusDistributionWidth);
        };

    protected:
    public:
        ParticleSimulationSettings(AnisotropyDistribution aniso,
            bool UseMagDist, const Distribution::IDistribution& MagDist, const prec& MagWidth, bool UseHydroDist, const Distribution::IDistribution& HydroDist, const prec& HydroWidth)
        : _useRelativeMagneticRadiusDistributionWidth(UseMagDist), _useRelativeHydrodynamicShellDistributionWidth(UseHydroDist),
          _magneticRadiusDistributionType(MagDist), _magneticRadiusDistributionWidth(MagWidth),
          _hydrodynamicShellDistributionType(HydroDist), _hydrodynamicShellDistributionWidth(HydroWidth),
          anisotropyDistribution(aniso)
        {
            initDistributions();
        };

        ParticleSimulationSettings() = default;

        ParticleSimulationSettings(const ThisClass& Other) :
            _useRelativeMagneticRadiusDistributionWidth(Other._useRelativeMagneticRadiusDistributionWidth),
            _useRelativeHydrodynamicShellDistributionWidth(Other._useRelativeHydrodynamicShellDistributionWidth),
            _magneticRadiusDistributionType(Other._magneticRadiusDistributionType),
            _magneticRadiusDistributionWidth(Other._magneticRadiusDistributionWidth),
            _hydrodynamicShellDistributionType(Other._hydrodynamicShellDistributionType),
            _hydrodynamicShellDistributionWidth(Other._hydrodynamicShellDistributionWidth),
            anisotropyDistribution(Other.anisotropyDistribution)
        {
            initDistributions();
        }

        void initDistributions()
        {
            if (_useRelativeMagneticRadiusDistributionWidth)
                initMagneticRadiusDists(1);

            if (_useRelativeHydrodynamicShellDistributionWidth)
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
            ar(Archives::createNamedValue("Use_relative_distribution_width_for_magnetic_radius", _useRelativeMagneticRadiusDistributionWidth));
            tmp = to_string(_magneticRadiusDistributionType);
            ar(Archives::createNamedValue("Magnetic_radius_distribution_type", tmp));
            ar(Archives::createNamedValue("Magnetic_radius_distribution_width", _magneticRadiusDistributionWidth));

            //Hydrodynamic Shell
            ar(Archives::createNamedValue("Use_relative_distribution_width_for_hydrodynamic_shell", _useRelativeHydrodynamicShellDistributionWidth));
            tmp = to_string(_hydrodynamicShellDistributionType);
            ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_type", tmp));
            ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_width", _hydrodynamicShellDistributionWidth));
        }

        template<typename Archive>
        void load(Archive &ar)
        {
            //Magnetic Radius
            ar(Archives::createNamedValue("Use_relative_distribution_width_for_magnetic_radius", _useRelativeMagneticRadiusDistributionWidth));
            std::string tmp { to_string(_magneticRadiusDistributionType) };
            ar(Archives::createNamedValue("Magnetic_radius_distribution_type", tmp));
            _magneticRadiusDistributionType = Distribution::from_string<Distribution::IDistribution>(tmp);
            ar(Archives::createNamedValue("Magnetic_radius_distribution_width", _magneticRadiusDistributionWidth));

            //Hydrodynamic Shell
            ar(Archives::createNamedValue("Use_relative_distribution_width_for_hydrodynamic_shell", _useRelativeHydrodynamicShellDistributionWidth));
            tmp = to_string(_hydrodynamicShellDistributionType);
            ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_type", tmp));
            _hydrodynamicShellDistributionType = Distribution::from_string<Distribution::IDistribution>(tmp);
            ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_width", _hydrodynamicShellDistributionWidth));

            if (_useRelativeMagneticRadiusDistributionWidth)
                initMagneticRadiusDists(1);

            if (_useRelativeHydrodynamicShellDistributionWidth)
                initHydrodynamicShellDists(1);
        }
    };
}



#endif    // INC_ParticleSimulationSettings_H
// end of ParticleSimulationSettings.h
///---------------------------------------------------------------------------------------------------
