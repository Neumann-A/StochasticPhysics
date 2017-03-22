///---------------------------------------------------------------------------------------------------
// file:	ParticleProvider.h
//
// summary: 	Declares the particle provider class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 05.06.2016

#ifndef INC_ParticleProvider_H
#define INC_ParticleProvider_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <experimental/filesystem>

#include <Eigen/Core>

#include "Archive/NamedValue.h"
#include "Archive/InputArchive.h"
#include "Archive/LoadConstructor.h"
//#include "../Archive/Matlab_Archive.h"

#include "PropertyProvider.h"

#include "Settings/ParticleSimulationParameters.h"

namespace Archives
{
	class MatlabOutputArchive;
}


///-------------------------------------------------------------------------------------------------
/// <signature>	Provider </signature>
///
/// <summary>	summary: Namespace for the different Providers of Objects </summary>
///-------------------------------------------------------------------------------------------------
namespace Provider
{
	///-------------------------------------------------------------------------------------------------
	/// <summary>	A particle provider. </summary>
	///
	/// <remarks>	Alexander Neumann, 05.06.2016. </remarks>
	///
	/// <typeparam name="prec">	Precission of used Floating Point. </typeparam>
	///
	/// <seealso cref="T:PropertyProvider{prec}"/>
	///-------------------------------------------------------------------------------------------------
	template<typename prec>
	class ParticleProvider : public IGeneralProvider<prec>
	{
	private:
		typedef ParticleProvider<prec>							ThisClass;
	public:
		typedef prec											Precision;
		typedef std::string										ParticleName;
		typedef std::string										PathToFile;
		typedef std::size_t										NumberOfParticles;
		typedef	Parameters::ParticleSimulationParameters<prec>	ParticleSimulationParameters;
	
		typedef std::tuple<ParticleName, PathToFile, NumberOfParticles, ParticleSimulationParameters> ParticleInformation;

	private:
		bool													_useDiscreteDistribution{ false };
		bool													_saveParticleSettings{ true };

		/// <summary>	Number of used particles. Internal memory </summary>
		std::vector<NumberOfParticles>							_numberOfUsedParticles;
		
		/// <summary>	Particle Informations. </summary>
		std::vector<ParticleInformation>						_particleInformations;

		/// <summary>	Distributions used to get the next Particle Parameters </summary>
		Distribution::DistributionHelperDiscrete<std::size_t> _DistHelper;

		Distribution::DistributionHelperDiscrete<std::size_t> buildDist(const std::vector<ParticleInformation> &ParInfos)
		{
			std::vector<std::size_t> tmp;
			for (const auto& Vals : ParInfos)
			{
				tmp.emplace_back(std::get<2>(Vals));
			}
			Distribution::DistributionHelperDiscrete<std::size_t> tmp2{ tmp };
			return tmp2;
		};

		template<typename Archive>
		std::enable_if_t<std::is_same<Archives::MatlabOutputArchive, std::decay_t<Archive>>::value>
			serializeParticleParameters(Archive &ar, const ParticleName &ParName, const ParticleSimulationParameters &SimParams, const std::experimental::filesystem::path &) const
		{
			ar(Archives::createNamedValue(ParName, SimParams));
		};

		template<typename Archive>
		std::enable_if_t<!std::is_same<Archives::MatlabOutputArchive, std::decay_t<Archive>>::value>
		    serializeParticleParameters(Archive &ar, const ParticleName &ParName, const ParticleSimulationParameters &SimParams, const std::experimental::filesystem::path &Path) const
		{
			if (!Path.empty())
			{
				Archive ar2{ Path };
				ar2(SimParams);
			}
			else
			{
				ar(Archives::createNamedValue(ParName, SimParams));
			}
		};

	protected:
	public:
		constexpr bool usesDiscreteDistribution() const noexcept { return _useDiscreteDistribution; };

		explicit ParticleProvider(const std::vector<ParticleInformation> &ParInfos, bool UseDiscreteDist, bool saveSingleParticleSettings)
			:  _useDiscreteDistribution(UseDiscreteDist), _saveParticleSettings(saveSingleParticleSettings), _numberOfUsedParticles(ParInfos.size(), 0),
			_particleInformations(ParInfos), _DistHelper(buildDist(ParInfos))
		{};
	
		static inline std::string getSectionName() { return std::string{ "Particle_Provider" }; };

		template<typename Archive>
		void save(Archive &ar) const// here we have actually do split the work
		{
			ar(Archives::createNamedValue("Use_discrete_distribution_to_select_particle", _useDiscreteDistribution));
			ar(Archives::createNamedValue("Save_individual_particle_settings", _saveParticleSettings));

			std::size_t ParNumber{ 1 };
			for (auto& tuple : _particleInformations)
			{
				ParticleName					Name{ std::get<0>(tuple) };
				PathToFile						File{ std::get<1>(tuple) };
				NumberOfParticles				Number{ std::get<2>(tuple) };
				ParticleSimulationParameters	SimPar{ std::get<3>(tuple) };

				if (Name.empty())
					Name = std::string{ "Particle_" + std::to_string(ParNumber) };

				std::experimental::filesystem::path Path{ File };
				
				//if (Path.empty())
				//	File = Name + ".ini";

				ar(Archives::createNamedValue("Particle_List", Archives::createNamedValue(Name, File)));
				ar(Archives::createNamedValue("Particle_Numbers", Archives::createNamedValue(Name, Number)));
				ar(Archives::createNamedValue("Used_Particle_Numbers", Archives::createNamedValue(Name, _numberOfUsedParticles.at(ParNumber-1))));

				if (_saveParticleSettings)
				{
					serializeParticleParameters(ar, Name, SimPar, Path);	
				}

				++ParNumber;
			}
		}

		ParticleSimulationParameters getProvidedObject() override final
		{
			unsigned long long ParNo{ 0 };
			if (_useDiscreteDistribution)
			{
				ParNo = _DistHelper.getValueFromDistribution();
				_numberOfUsedParticles[ParNo]++;
				//std::get<2>(_particleInformations[ParNo])++;
			}
			else
			{
				unsigned long long counter{ 0 };
				for (const auto& Pars : _particleInformations)
				{
					const auto& number = std::get<2>(Pars);
					if (_numberOfUsedParticles[counter] < number)
					{
						++_numberOfUsedParticles[counter];
						ParNo = counter;
						break;
					}
					else
					{
						++counter;
					}
				}
			}
			const auto& tuple = _particleInformations.at(ParNo);
			return static_cast<ParticleSimulationParameters>(std::get<3>(tuple));
		};
		virtual unsigned long long getNumberOfNecessarySimulations() const noexcept override final
		{
			unsigned long long result{ 0 };
			for (const auto& Pars : _particleInformations)
			{
				result += std::get<2>(Pars);
			}
			return result;
		};
	};
};


namespace Archives
{
	template<typename prec>
	class LoadConstructor<Provider::ParticleProvider<prec>>
	{
	private:
		using type = Provider::ParticleProvider<prec>;
	public:

		template <typename Archive>
		static inline type construct(InputArchive<Archive>& ar)
		{
			return construct(*static_cast<Archive * const>(&ar));
		}

		template <typename Archive>
		static inline std::enable_if_t<std::is_base_of<InputArchive<std::decay_t<Archive>>,std::decay_t<Archive>>::value, type> construct(Archive& ar)
		{
			//typedef prec											Precision;
			typedef std::string										ParticleName;
			typedef std::string										PathToFile;
			typedef std::size_t										NumberOfParticles;
			typedef	Parameters::ParticleSimulationParameters<prec>	ParticleSimulationParameters;
			typedef std::tuple<ParticleName, PathToFile, NumberOfParticles, ParticleSimulationParameters> ParticleInformation;

			std::vector<ParticleInformation> ParInfos;
			bool UseDiscreteDist{ false };
			bool saveSingleParticleSettings{ false };

			ar(Archives::createNamedValue(type::getSectionName(),Archives::createNamedValue("Use_discrete_distribution_to_select_particle", UseDiscreteDist)));
			ar(Archives::createNamedValue(type::getSectionName(),Archives::createNamedValue("Save_individual_particle_settings", saveSingleParticleSettings)));

			const auto& list{ ar.list(Archives::createNamedValue(type::getSectionName(),Archives::createNamedValue("Particle_List", nullptr))) };

			ParInfos.clear();
			for (const auto& elem : list)
			{
				const ParticleName&				Name{ elem.first };
				PathToFile						File;
				NumberOfParticles				Number;
				ParticleSimulationParameters	SimPar;

				ar(Archives::createNamedValue(type::getSectionName(),Archives::createNamedValue("Particle_List", Archives::createNamedValue(Name, File))));
				ar(Archives::createNamedValue(type::getSectionName(),Archives::createNamedValue("Particle_Numbers", Archives::createNamedValue(Name, Number))));

				if (!File.empty())
				{
					Archive ar2{ std::experimental::filesystem::path{ File } };
					ar2(SimPar);
				}
				else
				{
					ar(Archives::createNamedValue(Name, SimPar));
				}

				ParInfos.push_back(std::make_tuple(Name, File, Number, SimPar));
			}
			return type{ std::move(ParInfos), UseDiscreteDist, saveSingleParticleSettings };
		};

	};
}


#endif	// INC_ParticleProvider_H
// end of ParticleProvider.h
///---------------------------------------------------------------------------------------------------
