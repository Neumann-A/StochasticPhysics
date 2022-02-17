///---------------------------------------------------------------------------------------------------
// file:		ResultsSettings.h
//
// summary: 	Declares the results settings class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 20.06.2016

#ifndef INC_ResultsSettings_H
#define INC_ResultsSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <string>
#include <map>
#include <exception>

#include <filesystem>

#include <SerAr/Core/NamedValue.h>
#include <SerAr/AllArchiveEnums.hpp>

namespace Settings
{
    enum class IResultFileType { ResultFileType_undefined, ResultFileType_MATLAB, ResultFileType_HDF5, ResultFileType_JSON };

    extern const std::map<IResultFileType, std::string> IResultFileTypeMap;
    extern const std::map<IResultFileType, SerAr::ArchiveTypeEnum> ResultFileEnumToArchiveEnumMap;

    std::string to_string(const IResultFileType& field);

    template<typename T>
    T from_string(const std::string &string);

    template<>
    IResultFileType from_string<IResultFileType>(const std::string &string);

    template<IResultFileType T>
    class ResultArchiveSelector;

    class ResultSettings
    {
    private:
        typedef ResultSettings      ThisClass;

        bool m_SaveSingleSimulations{ false };
        bool m_UseExtraFileForSingleSimulations{ false };
        bool m_ScaleResult{ false };
        std::size_t _SaveInterval{ 1 };
        std::filesystem::path m_SaveFilepath{ "Results.mat" };
        std::filesystem::path m_SaveFilepathSingle{ "ResultsSingle.mat" };
        std::string m_SaveSingleFilePrefix{ "Simulation" };
        IResultFileType m_ResultFileType{ IResultFileType::ResultFileType_MATLAB };


    public:
        explicit ResultSettings(const bool& saveSingle, const uint64_t& interval, const std::filesystem::path& filepath, const std::filesystem::path& singlepath,
                                const std::string& singleprefix, IResultFileType type = IResultFileType::ResultFileType_MATLAB, const bool& scale = true) noexcept
            : m_SaveSingleSimulations(saveSingle), m_ScaleResult(scale), _SaveInterval(interval), m_SaveFilepath(filepath), 
            m_SaveFilepathSingle(singlepath), m_SaveSingleFilePrefix(singleprefix), m_ResultFileType(type){};

        ResultSettings() = default;

        inline const bool& saveSingleSimulations() const noexcept { return m_SaveSingleSimulations; };
        inline const bool& getScaleResult() const noexcept { return m_ScaleResult; };
        inline const bool& useExtraFileForSingleSimulations() const noexcept { return m_UseExtraFileForSingleSimulations; };
        inline const std::size_t& getSaveInterval() const noexcept { return _SaveInterval; };

        inline const std::filesystem::path& getFilepath() const noexcept { return m_SaveFilepath; };
        inline std::filesystem::path&       getFilepath() noexcept { return m_SaveFilepath; };
        inline void  setFilepath(const std::filesystem::path& saveFilepath) { m_SaveFilepath = saveFilepath; };

        inline const std::string& getSingleFilePrefix() const noexcept { return m_SaveSingleFilePrefix; };

        inline const IResultFileType& getFileType() const noexcept	{	return m_ResultFileType;	}
        inline SerAr::ArchiveTypeEnum getSerArFileType() const noexcept	{	return ResultFileEnumToArchiveEnumMap.at(m_ResultFileType);	}

        inline void setFileType(const IResultFileType& type) noexcept	{ m_ResultFileType = type;}
        std::string getExtensionFromType() const;

        // Access the SaveFilepathSingle
        const std::filesystem::path& getSaveFilepathSingle(void) const noexcept { return(m_SaveFilepathSingle); }
        std::filesystem::path&       getSaveFilepathSingle(void) noexcept { return(m_SaveFilepathSingle); }
        void setSaveFilepathSingle(const std::filesystem::path& saveFilepathSingle) { m_SaveFilepathSingle = saveFilepathSingle; }

        static inline std::string getSectionName() noexcept { return std::string{ "Result_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Save_single_simulations", m_SaveSingleSimulations));
            ar(Archives::createNamedValue("Save_intervall", _SaveInterval));
            ar(Archives::createNamedValue("Save_single_name_prefix", m_SaveSingleFilePrefix));
            ar(Archives::createNamedValue("Normalize_results", m_ScaleResult));
            ar(Archives::createNamedValue("Use_extra_file_for_single_simulations", m_UseExtraFileForSingleSimulations));
            
            std::string str{ to_string(m_ResultFileType) };
            ar(Archives::createNamedValue(std::string{ "Savefile_Type" }, str));
            m_ResultFileType = from_string<decltype(m_ResultFileType)>(str);
            
            std::string tmp = m_SaveFilepath.string();
            ar(Archives::createNamedValue("Savefile", tmp));
            m_SaveFilepath = std::filesystem::path{ tmp };

            //Append extension if necessary 
            if (!m_SaveFilepath.has_extension())
                m_SaveFilepath.append(getExtensionFromType());
            
            tmp = m_SaveFilepathSingle.string();
            ar(Archives::createNamedValue("Single_simulations_savefile", tmp));
            m_SaveFilepathSingle = std::filesystem::path{ tmp };

            //Append extension if necessary 
            if (!m_SaveFilepathSingle.has_extension())
                m_SaveFilepathSingle.append(getExtensionFromType());
        }

    };
}

#endif	// INC_ResultsSettings_H
// end of ResultsSettings.h
///---------------------------------------------------------------------------------------------------
