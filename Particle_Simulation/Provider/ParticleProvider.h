///---------------------------------------------------------------------------------------------------
// file:    ParticleProvider.h
//
// summary:     Declares the particle provider class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 05.06.2016

#ifndef INC_ParticleProvider_H
#define INC_ParticleProvider_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <SerAr/Core/InputArchive.h>
#include <SerAr/Core/LoadConstructor.h>
#include <SerAr/Core/NamedValue.h>

#include <filesystem>
#include <vector>
#include <string>

#include "PropertyProvider.h"
#include "Settings/ParticleSimulationParameters.h"
#include <Eigen/Core>

///-------------------------------------------------------------------------------------------------
/// <signature>    Provider </signature>
///
/// <summary>    summary: Namespace for the different Providers of Objects </summary>
///-------------------------------------------------------------------------------------------------
namespace Provider
{
    ///-------------------------------------------------------------------------------------------------
    /// <summary>    A particle provider. </summary>
    ///
    /// <remarks>    Alexander Neumann, 05.06.2016. </remarks>
    ///
    /// <typeparam name="prec">    Precission of used Floating Point. </typeparam>
    ///
    /// <seealso cref="T:PropertyProvider{prec}"/>
    ///-------------------------------------------------------------------------------------------------

    
    template <typename T>
    struct ParticleTemplateList
    {
        std::vector<T> list{};

        template <typename Archive>
        void serialize(Archive& ar)
        {
            for (auto& elem : list) {
                ar(elem);
            }
        }
    };

    template<typename prec>
    struct ParticleInformation
    {
        std::string particleName;
        std::string particleFile;
        std::size_t numberOfParticles;
        Parameters::ParticleSimulationParameters<prec> particleParameters;
    };

    struct ParticleNameFileMapping
    {
        std::string particleName;
        std::string particleFile;

        template <typename Archive>
        void serialize(Archive& ar)
        {
            ar(Archives::createNamedValue(particleName, particleFile));
        }
    };

    struct ParticleNameNumber
    {
        std::string particleName;
        std::size_t particleNumbers;

        template <typename Archive>
        void serialize(Archive& ar)
        {
            ar(Archives::createNamedValue(particleName, particleNumbers));
        }
    };

    template <typename prec>
    class ParticleProvider : public IGeneralProvider<prec>
    {
    private:
        typedef ParticleProvider<prec> ThisClass;

    public:
        typedef prec Precision;
        bool saveParticlesInSameFile{true};
    private:
        bool useDiscreteDistribution{false};
        bool saveParticleSettings{true};

        /// <summary>    Number of used particles. Internal memory </summary>
        std::vector<std::size_t> numberOfUsedParticles;

        /// <summary>    Particle Informations. </summary>
        std::vector<ParticleInformation<prec>> particleInformations;

        /// <summary>    Distributions used to get the next Particle Parameters </summary>
        Distribution::DistributionHelperDiscrete<std::size_t> DistHelper;

        Distribution::DistributionHelperDiscrete<std::size_t>
        buildDist(const std::vector<ParticleInformation<prec>>& ParInfos)
        {
            std::vector<std::size_t> tmp;
            for (const auto& vals : ParInfos) {
                tmp.emplace_back(vals.numberOfParticles);
            }
            Distribution::DistributionHelperDiscrete<std::size_t> tmp2{tmp};
            return tmp2;
        };

        template <typename Archive>
        void serializeParticleParameters(Archive& ar, const std::string& ParName,
                                    const Parameters::ParticleSimulationParameters<prec>& SimParams,
                                    const std::filesystem::path& Path) const
        {
            if (!Path.empty() && !saveParticlesInSameFile) {
                Archive ar2{Path};
                ar2(SimParams);
            }
            else {
                ar(Archives::createNamedValue(ParName, SimParams));
            }
        }

    protected:
    public:

    void setParticleSaveFileExtension(std::filesystem::path extension){
        for (auto& particle : particleInformations) {
            std::filesystem::path tmp =particle.particleFile;
            tmp.replace_extension(extension);
            particle.particleFile=tmp.string();
        }
    }
        constexpr bool usesDiscreteDistribution() const noexcept { return useDiscreteDistribution; };

        explicit ParticleProvider(const std::vector<ParticleInformation<prec>>& ParInfos, bool UseDiscreteDist,
                                  bool saveSingleParticleSettings)
            : useDiscreteDistribution(UseDiscreteDist)
            , saveParticleSettings(saveSingleParticleSettings)
            , numberOfUsedParticles(ParInfos.size(), 0)
            , particleInformations(ParInfos)
            , DistHelper(buildDist(ParInfos)){};

        virtual ~ParticleProvider() = default;
        ALLOW_DEFAULT_COPY_AND_ASSIGN(ParticleProvider)
        static inline std::string getSectionName() { return std::string{"Particle_Provider"}; };

        template <typename Archive>
        void save(Archive& ar) const // here we have actually do split the work
        {
            ar(Archives::createNamedValue("Use_discrete_distribution_to_select_particle", useDiscreteDistribution));
            ar(Archives::createNamedValue("Save_individual_particle_settings", saveParticleSettings));

            std::size_t ParNumber{1};

            //Particle List 
            //std::vector<ParticleNameFileMapping> particle_list;
            ParticleTemplateList<ParticleNameFileMapping> particle_list;
            particle_list.list.reserve(particleInformations.size());
            //std::vector<ParticleNameNumber>      particle_numbers;
            ParticleTemplateList<ParticleNameNumber> particle_numbers;
            particle_numbers.list.reserve(particleInformations.size());
            //std::vector<ParticleNameNumber>      particle_used;
            ParticleTemplateList<ParticleNameNumber> particle_used;
            particle_used.list.reserve(particleInformations.size());

            for (auto& info : particleInformations) {
                auto Name = info.particleName;
                if (Name.empty())
                    Name = std::string{"Particle_" + std::to_string(ParNumber)};
                particle_list.list.push_back({Name, info.particleFile});
                particle_numbers.list.push_back({Name, info.numberOfParticles});
                particle_used.list.push_back({Name, numberOfUsedParticles.at(ParNumber - 1)});

                std::filesystem::path Path{info.particleFile};
                if (saveParticleSettings) {
                    serializeParticleParameters(ar, Name, info.particleParameters, Path);
                }
                ++ParNumber;
            }
            ar(Archives::createNamedValue("Particle_List", particle_list));
            ar(Archives::createNamedValue("Particle_Numbers", particle_numbers));
            ar(Archives::createNamedValue("Used_Particle_Numbers", particle_used));
        }

        Parameters::ParticleSimulationParameters<prec> getProvidedObject() override final
        {
            std::size_t ParNo{0};
            if (useDiscreteDistribution) {
                ParNo = DistHelper.getValueFromDistribution();
                numberOfUsedParticles[ParNo]++;
            }
            else {
                std::size_t counter{0};
                for (const auto& Pars : particleInformations) {
                    const auto& number = Pars.numberOfParticles;
                    if (numberOfUsedParticles[counter] < number) {
                        ++numberOfUsedParticles[counter];
                        ParNo = counter;
                        break;
                    }
                    else {
                        ++counter;
                    }
                }
            }
            const auto& par_info = particleInformations.at(ParNo);
            return static_cast<Parameters::ParticleSimulationParameters<prec>>(par_info.particleParameters);
        };

        virtual std::size_t getNumberOfNecessarySimulations() const noexcept override final
        {
            std::size_t result{0};
            for (const auto& Pars : particleInformations) {
                result += Pars.numberOfParticles;
            }
            return result;
        };
    };
} // namespace Provider

namespace Archives
{
    template <typename prec>
    class LoadConstructor<Provider::ParticleProvider<prec>>
    {
    private:
        using type = Provider::ParticleProvider<prec>;

    public:
        template <typename Archive>
        static inline type construct(InputArchive<Archive>& ar)
        {
            return construct(*static_cast<Archive* const>(&ar));
        }

        template <typename Archive>
        static inline std::enable_if_t<
            std::is_base_of<InputArchive<std::decay_t<Archive>>, std::decay_t<Archive>>::value, type>
        construct(Archive& ar)
        {
            //typedef prec                                            Precision;
            typedef std::string ParticleName;
            typedef std::string PathToFile;
            typedef std::size_t NumberOfParticles;
            typedef Parameters::ParticleSimulationParameters<prec> ParticleSimulationParameters;

            std::vector<Provider::ParticleInformation<prec>> ParInfos;
            bool UseDiscreteDist{false};
            bool saveSingleParticleSettings{false};

            ar(Archives::createNamedValue(
                type::getSectionName(),
                Archives::createNamedValue("Use_discrete_distribution_to_select_particle", UseDiscreteDist)));
            ar(Archives::createNamedValue(
                type::getSectionName(),
                Archives::createNamedValue("Save_individual_particle_settings", saveSingleParticleSettings)));

            const auto& list{ar.list(Archives::createNamedValue(type::getSectionName(),
                                                                Archives::createNamedValue("Particle_List", nullptr)))};

            ParInfos.clear();
            for (const auto& elem : list) {
                const ParticleName& Name{elem.first};
                PathToFile File;
                NumberOfParticles Number;
                ParticleSimulationParameters SimPar;

                ar(Archives::createNamedValue(
                    type::getSectionName(),
                    Archives::createNamedValue("Particle_List", Archives::createNamedValue(Name, File))));
                ar(Archives::createNamedValue(
                    type::getSectionName(),
                    Archives::createNamedValue("Particle_Numbers", Archives::createNamedValue(Name, Number))));

                if (!File.empty()) {
                    Archive ar2{std::filesystem::path{File}};
                    ar2(SimPar);
                }
                else {
                    ar(Archives::createNamedValue(Name, SimPar));
                }
                ParInfos.push_back({Name, File, Number, SimPar});
            }
            return type{std::move(ParInfos), UseDiscreteDist, saveSingleParticleSettings};
        }
    };
} // namespace Archives

#endif // INC_ParticleProvider_H
// end of ParticleProvider.h
///---------------------------------------------------------------------------------------------------
