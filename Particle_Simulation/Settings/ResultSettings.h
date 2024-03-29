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

        bool _SaveSingleSimulations{ false };
        bool _UseExtraFileForSingleSimulations{ false };
        bool _ScaleResult{ false };
        std::size_t _SaveInterval{ 1 };
        std::filesystem::path _SaveFilepath{ "Results.mat" };
        std::filesystem::path _SaveFilepathSingle{ "ResultsSingle.mat" };
        std::string _SaveSingleFilePrefix{ "Simulation" };
        IResultFileType _ResultFileType{ IResultFileType::ResultFileType_MATLAB };


    public:
        explicit ResultSettings(const bool& saveSingle, const uint64_t& interval, const std::filesystem::path& filepath, const std::filesystem::path& singlepath,
                                const std::string& singleprefix, IResultFileType type = IResultFileType::ResultFileType_MATLAB, const bool& scale = true) noexcept
            : _SaveSingleSimulations(saveSingle), _ScaleResult(scale), _SaveInterval(interval), _SaveFilepath(filepath), 
            _SaveFilepathSingle(singlepath), _SaveSingleFilePrefix(singleprefix), _ResultFileType(type){};

        ResultSettings() = default;

        inline const bool& saveSingleSimulations() const noexcept { return _SaveSingleSimulations; };
        inline const bool& getScaleResult() const noexcept { return _ScaleResult; };
        inline const bool& useExtraFileForSingleSimulations() const noexcept { return _UseExtraFileForSingleSimulations; };
        inline const std::size_t& getSaveInterval() const noexcept { return _SaveInterval; };

        inline const std::filesystem::path& getFilepath() const noexcept { return _SaveFilepath; };
        inline std::filesystem::path&       getFilepath() noexcept { return _SaveFilepath; };
        inline void  setFilepath(const std::filesystem::path& saveFilepath) { _SaveFilepath = saveFilepath; };

        inline const std::string& getSingleFilePrefix() const noexcept { return _SaveSingleFilePrefix; };

        inline const IResultFileType& getFileType() const noexcept	{	return _ResultFileType;	}
        inline SerAr::ArchiveTypeEnum getSerArFileType() const noexcept	{	return ResultFileEnumToArchiveEnumMap.at(_ResultFileType);	}

        inline void setFileType(const IResultFileType& type) noexcept	{ _ResultFileType = type;}
        std::string getExtensionFromType() const;

        // Access the SaveFilepathSingle
        const std::filesystem::path& getSaveFilepathSingle(void) const noexcept { return(_SaveFilepathSingle); }
        std::filesystem::path&       getSaveFilepathSingle(void) noexcept { return(_SaveFilepathSingle); }
        void setSaveFilepathSingle(const std::filesystem::path& saveFilepathSingle) { _SaveFilepathSingle = saveFilepathSingle; }

        static inline std::string getSectionName() noexcept { return std::string{ "Result_Settings" }; };

        template<typename Archive>
        void serialize(Archive &ar)
        {
            ar(Archives::createNamedValue("Save_single_simulations", _SaveSingleSimulations));
            ar(Archives::createNamedValue("Save_intervall", _SaveInterval));
            ar(Archives::createNamedValue("Save_single_name_prefix", _SaveSingleFilePrefix));
            ar(Archives::createNamedValue("Normalize_results", _ScaleResult));
            ar(Archives::createNamedValue("Use_extra_file_for_single_simulations", _UseExtraFileForSingleSimulations));
            
            std::string str{ to_string(_ResultFileType) };
            ar(Archives::createNamedValue(std::string{ "Savefile_Type" }, str));
            _ResultFileType = from_string<decltype(_ResultFileType)>(str);
            
            std::string tmp = _SaveFilepath.string();
            ar(Archives::createNamedValue("Savefile", tmp));
            _SaveFilepath = std::filesystem::path{ tmp };

            //Append extension if necessary 
            if (!_SaveFilepath.has_extension())
                _SaveFilepath.append(getExtensionFromType());
            
            tmp = _SaveFilepathSingle.string();
            ar(Archives::createNamedValue("Single_simulations_savefile", tmp));
            _SaveFilepathSingle = std::filesystem::path{ tmp };

            //Append extension if necessary 
            if (!_SaveFilepathSingle.has_extension())
                _SaveFilepathSingle.append(getExtensionFromType());
        }

    };
}

#endif	// INC_ResultsSettings_H
// end of ResultsSettings.h
///---------------------------------------------------------------------------------------------------
