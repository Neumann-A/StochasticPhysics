///---------------------------------------------------------------------------------------------------
// file:		ParticleSimulationParameters.h
//
// summary: 	Declares the particle simulation parameters class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.06.2016

#ifndef INC_ParticleSimulationParameters_H
#define INC_ParticleSimulationParameters_H
///---------------------------------------------------------------------------------------------------
#pragma once

//#include <Eigen\Core>

#include "Archive/NamedValue.h"

#include "Properties/ParticleProperties.h"
#include "ParticleSimulationSettings.h"
#include "ParticleSimulationInitSettings.h"

namespace Parameters
{
	///-------------------------------------------------------------------------------------------------
	/// <summary>	Aggregation of all parameters needed for a simulation</summary>
	///-------------------------------------------------------------------------------------------------
	template< typename prec>
	class ParticleSimulationParameters 
	{
	private:
		typedef ParticleSimulationParameters<prec>				ThisClass;
	public:
		typedef prec											Precision;
		typedef Properties::ParticlesProperties<prec>			ParticleProperties;
		typedef Settings::ParticleSimulationSettings<prec>		ParticleSimulationSettings;
		typedef Settings::ParticleSimulationInitSettings<prec>	ParticleSimulationInitialization;

	private:
		ParticleProperties										_particleProperties;
		ParticleSimulationSettings								_particleSimSettings;
		ParticleSimulationInitialization						_particleSimInit;

	public:
		// Access the ParticleProperties
		const ParticleProperties& getParticleProperties(void) const noexcept				{ return(_particleProperties);				}
		ParticleProperties&		modParticleProperties(void) noexcept { return(_particleProperties); }
		void  setParticleProperties(const ParticleProperties& particleProperties) noexcept	{ _particleProperties = particleProperties;	}

		ParticleProperties getNewParticleProperties(void) noexcept { return (this->_particleSimSettings.applySettingsToParticleProperties(this->_particleProperties)); }

		// Access the ParticleSimSettings
		const ParticleSimulationSettings& getParticleSimulationSettings(void) const noexcept					{ return(_particleSimSettings);					}
		void setParticleSimulationSettings(const ParticleSimulationSettings& particleSimSettings) noexcept	{ _particleSimSettings = particleSimSettings;	}

		// Access the ParticleSimInit
		const ParticleSimulationInitialization& getParticleSimulationInitialization(void) const noexcept				{ return(_particleSimInit);				}
		void setParticleSimulationInitialization(const ParticleSimulationInitialization& particleSimInit) noexcept 	{ _particleSimInit = particleSimInit;	}

		ParticleSimulationParameters(const ParticleSimulationSettings& ParSimSet, const ParticleSimulationInitialization& ParSimInt, const ParticleProperties& ParProps) noexcept
			: _particleProperties(ParProps), _particleSimSettings(ParSimSet), _particleSimInit(ParSimInt)
		{	};
		ParticleSimulationParameters() = default;

		static inline std::string getSectionName() noexcept { return std::string{ "Particle_Simulation_Parameters" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			ar(Archives::createNamedValue(ParticleProperties::getSectionName(), _particleProperties));
			ar(Archives::createNamedValue(ParticleSimulationSettings::getSectionName(), _particleSimSettings));
			ar(Archives::createNamedValue(ParticleSimulationInitialization::getSectionName(), _particleSimInit));
		}
	};
}

#endif	// INC_ParticleSimulationParameters_H
// end of ParticleSimulationParameters.h
///---------------------------------------------------------------------------------------------------
