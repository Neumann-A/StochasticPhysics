///---------------------------------------------------------------------------------------------------
// file:	ParticleSimulationSettings.h
//
// summary: 	Declares the particle simulation settings class
//
//			Copyright (c) 2016 Alexander Neumann.
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

#include "math/DistributionHelper.h"

#include "Archive/NamedValue.h"

namespace Settings
{
	template <typename prec>
	class ParticleSimulationSettings
	{
	private:
		typedef ParticleSimulationSettings<prec>		ThisClass;
		typedef Properties::ParticlesProperties<prec>	ParticleProperties;
		typedef Eigen::Matrix<prec, 3, 1>				Vec3D;

		//Use Relative Distributions Widths?
		bool											_useRelativeAnisotropyConstantsDistributionWidth{ true };
		bool											_useRelativeMagneticRadiusDistributionWidth{ true };
		bool											_useRelativeHydrodynamicShellDistributionWidth{ true };
		
		//Distribution
		Distribution::IDistribution											  _anisotropyConstantsDistributionType{ Distribution::IDistribution::Distribution_normal };
		std::vector<prec>												      _anisotropyConstantsDistributionWidth{ 0 };
		std::vector<std::unique_ptr<Distribution::IDistributionHelper<prec>>> _aniDistHelper;


		Distribution::IDistribution									_magneticRadiusDistributionType{ Distribution::IDistribution::Distribution_lognormal };
		prec														_magneticRadiusDistributionWidth{ 0 };
		std::unique_ptr<Distribution::IDistributionHelper<prec>>	_magRadiusDistHelper{ nullptr };

		Distribution::IDistribution									_hydrodynamicShellDistributionType{ Distribution::IDistribution::Distribution_lognormal };
		prec														_hydrodynamicShellDistributionWidth{ 0 };
		std::unique_ptr<Distribution::IDistributionHelper<prec>>	_hydroShellDistHelper{ nullptr };
	

		void applyAnisotropyDistribution(ParticleProperties &ParProperties)
		{
			if (_aniDistHelper.size() == 0)
			{
				if(!_useRelativeAnisotropyConstantsDistributionWidth)
					initAnisotropyDists(ParProperties.getMagneticProperties().getAnisotropyConstants());
				else
					initAnisotropyDists(std::vector<prec>{ 1 });
			}
            assert(ParProperties.getMagneticProperties().getAnisotropyConstants().size() == _aniDistHelper.size());
			std::vector<prec> tmp;
			auto aniso = ParProperties.getMagneticProperties().getAnisotropyConstants().begin();
			for (auto& it : _aniDistHelper)
			{
				if (!_useRelativeAnisotropyConstantsDistributionWidth)
					tmp.emplace_back(it->getValueFromDistribution());
				else
				{
					tmp.emplace_back((*aniso)*it->getValueFromDistribution());
					aniso++;
				}
				
			}
			ParProperties.modMagneticProperties().setAnisotropyConstants(tmp);
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

		std::unique_ptr<Distribution::IDistributionHelper<prec>> initDistribution(const Distribution::IDistribution& Type, const prec& mean, const prec& width)
		{
			std::unique_ptr<Distribution::IDistributionHelper<prec>> ptr;
			switch (Type)
			{
			case Distribution::IDistribution::Distribution_delta:
				ptr = std::make_unique< Distribution::DeltaDistribution<prec>>(mean);
				break;
			case Distribution::IDistribution::Distribution_lognormal:
				{
					const prec meansquare{ std::pow(mean, 2) };
					const prec var{ std::pow(width, 2) };
					const prec meanlog{ std::log(meansquare / std::sqrt(var + meansquare)) };
					const prec stdlog{ std::log1p(var / meansquare) };
					ptr = std::make_unique < Distribution::DistributionHelper<prec, std::lognormal_distribution<prec>>>(std::pair<prec, prec>{meanlog, stdlog});
				}
				break;
			case Distribution::IDistribution::Distribution_gamma:
				{
				//Mean = shape * scale
				//Variance = shape * scale^2
					const prec var{ std::pow(width, 2) };
					const prec scale{var/mean};
					const prec shape{mean/scale};
					ptr = std::make_unique < Distribution::DistributionHelper<prec, std::gamma_distribution<prec>>>(std::pair<prec, prec>{shape, scale});
				}
				break;
			case Distribution::IDistribution::Distribution_normal:
				ptr = std::make_unique < Distribution::DistributionHelper<prec, std::normal_distribution<prec>>>(std::pair<prec, prec>{mean, width});
				break;
			case Distribution::IDistribution::Distribution_unknown:
				throw std::runtime_error{ "Cannot instantiate unknown distribution!" };
			default:
				ptr = std::make_unique< Distribution::DeltaDistribution<prec>>(mean);
				break;
			}
			return std::move(ptr);
		};

		void initAnisotropyDists(const std::vector<prec>& mean) 
		{
			//assert(mean.size() <= _anisotropyConstantsDistributionWidth.size(), "You mixed vectors with different sizes!");
			auto meanval = mean.cbegin();
			for (const auto& anisowidth : _anisotropyConstantsDistributionWidth)
			{
				_aniDistHelper.push_back(initDistribution(_anisotropyConstantsDistributionType, *meanval, (*meanval)*anisowidth));
				meanval++;
			}
		};

		void initHydrodynamicShellDists(const prec& mean)
		{
			_hydroShellDistHelper = initDistribution(_hydrodynamicShellDistributionType, mean, mean*_hydrodynamicShellDistributionWidth);
		};

		void initMagneticRadiusDists(const prec& mean) 
		{
			_magRadiusDistHelper = initDistribution(_magneticRadiusDistributionType, mean, mean*_magneticRadiusDistributionWidth);
		};

	protected:
	public:
		ParticleSimulationSettings(bool UseAnisoDist,const Distribution::IDistribution& AnisoDist, const std::vector<prec>& AnisoWidths,
			bool UseMagDist, const Distribution::IDistribution& MagDist, const prec& MagWidth, bool UseHydroDist, const Distribution::IDistribution& HydroDist, const prec& HydroWidth)
		:_useRelativeAnisotropyConstantsDistributionWidth(UseAnisoDist), _useRelativeMagneticRadiusDistributionWidth(UseMagDist), _useRelativeHydrodynamicShellDistributionWidth(UseHydroDist),
			_anisotropyConstantsDistributionType(AnisoDist), _anisotropyConstantsDistributionWidth(AnisoWidths),
			_magneticRadiusDistributionType(MagDist), _magneticRadiusDistributionWidth(MagWidth),
			_hydrodynamicShellDistributionType(HydroDist), _hydrodynamicShellDistributionWidth(HydroWidth)
		{
			initDistributions();
		};

		ParticleSimulationSettings() = default;

		ParticleSimulationSettings(const ThisClass& Other) :
			_useRelativeAnisotropyConstantsDistributionWidth(Other._useRelativeAnisotropyConstantsDistributionWidth),
			_useRelativeMagneticRadiusDistributionWidth(Other._useRelativeMagneticRadiusDistributionWidth),
			_useRelativeHydrodynamicShellDistributionWidth(Other._useRelativeHydrodynamicShellDistributionWidth),
			_anisotropyConstantsDistributionType(Other._anisotropyConstantsDistributionType),
			_anisotropyConstantsDistributionWidth(Other._anisotropyConstantsDistributionWidth),
			_magneticRadiusDistributionType(Other._magneticRadiusDistributionType),
			_magneticRadiusDistributionWidth(Other._magneticRadiusDistributionWidth),
			_hydrodynamicShellDistributionType(Other._hydrodynamicShellDistributionType),
			_hydrodynamicShellDistributionWidth(Other._hydrodynamicShellDistributionWidth)	
		{
			initDistributions();
		}

		void initDistributions()
		{
			if (_useRelativeAnisotropyConstantsDistributionWidth)
			{
				std::vector<prec> means(_anisotropyConstantsDistributionWidth.size(), 1);
				initAnisotropyDists(means);
			}

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
			//Anisotropy
			ar(Archives::createNamedValue("Use_relative_distribution_width_for_anisotropy_constants", _useRelativeAnisotropyConstantsDistributionWidth));
			std::string tmp{to_string(_anisotropyConstantsDistributionType)};
			ar(Archives::createNamedValue("Anisotropy_constants_distribution_type", tmp));
			ar(Archives::createNamedValue("Anisotropy_constants_distribution_width", _anisotropyConstantsDistributionWidth));

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
			//Anisotropy
			ar(Archives::createNamedValue("Use_relative_distribution_width_for_anisotropy_constants", _useRelativeAnisotropyConstantsDistributionWidth));
			std::string tmp{ to_string(_anisotropyConstantsDistributionType) };
			ar(Archives::createNamedValue("Anisotropy_constants_distribution_type", tmp));
			_anisotropyConstantsDistributionType = Distribution::from_string<Distribution::IDistribution>(tmp);
			ar(Archives::createNamedValue("Anisotropy_constants_distribution_width", _anisotropyConstantsDistributionWidth));

			//Magnetic Radius
			ar(Archives::createNamedValue("Use_relative_distribution_width_for_magnetic_radius", _useRelativeMagneticRadiusDistributionWidth));
			tmp = to_string(_magneticRadiusDistributionType);
			ar(Archives::createNamedValue("Magnetic_radius_distribution_type", tmp));
			_magneticRadiusDistributionType = Distribution::from_string<Distribution::IDistribution>(tmp);
			ar(Archives::createNamedValue("Magnetic_radius_distribution_width", _magneticRadiusDistributionWidth));

			//Hydrodynamic Shell
			ar(Archives::createNamedValue("Use_relative_distribution_width_for_hydrodynamic_shell", _useRelativeHydrodynamicShellDistributionWidth));
			tmp = to_string(_hydrodynamicShellDistributionType);
			ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_type", tmp));
			_hydrodynamicShellDistributionType = Distribution::from_string<Distribution::IDistribution>(tmp);
			ar(Archives::createNamedValue("Hydrodynamic_shell_distribution_width", _hydrodynamicShellDistributionWidth));

			if (_useRelativeAnisotropyConstantsDistributionWidth)
			{
				std::vector<prec> means(_anisotropyConstantsDistributionWidth.size(), 1);
				initAnisotropyDists(means);
			}

			if (_useRelativeMagneticRadiusDistributionWidth)
				initMagneticRadiusDists(1);

			if (_useRelativeHydrodynamicShellDistributionWidth)
				initHydrodynamicShellDists(1);
		}
	};
}



#endif	// INC_ParticleSimulationSettings_H
// end of ParticleSimulationSettings.h
///---------------------------------------------------------------------------------------------------
