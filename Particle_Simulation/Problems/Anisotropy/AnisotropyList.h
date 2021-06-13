///---------------------------------------------------------------------------------------------------
// file:		Problems\Anisotropy\AnisotropyList.h
//
// summary: 	Declares the anisotropy list class
//
// Copyright (c) 2018 Alexander Neumann.
//
// author: Alexander
// date: 14.06.2018

#ifndef INC_AnisotropyList_H
#define INC_AnisotropyList_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <string_view>
#include <string>
#include <type_traits>

#include <MyCEL/types/static_map.hpp>

//Forward Declare all Anisotropies
namespace Problems::Anisotropy
{
    template <typename prec>
    class UniaxialAnisotropy;
    template <typename prec>
    class CubicAnisotropy;
    template <typename prec>
    class MixedAnisotropy;
    template <typename prec>
    class UniaxialCubicAnisotropy;

    template <typename anisotropy>
    struct AnisotropyTraits;
}

namespace Properties
{
    namespace
    {
        using namespace std::literals::string_view_literals;
    }
    /// <summary>	Values that represent anisotropies. </summary>
    enum class IAnisotropy { undefined = 0, Anisotropy_uniaxial = 1, Anisotropy_cubic, Anisotropy_mixed, Anisotropy_uniaxialcubic};

    /// <summary>	Map used to change the IAnisotropy enum to a string and vice versa. </summary>
    static constexpr const MyCEL::static_map<IAnisotropy, std::string_view, 5> IAnisotropyMap { { { 
                            { IAnisotropy::undefined,               "undefined"sv },
                            { IAnisotropy::Anisotropy_uniaxial,     "uniaxial"sv },
                            { IAnisotropy::Anisotropy_cubic,        "cubic"sv },
                            { IAnisotropy::Anisotropy_mixed,        "mixed"sv },
                            { IAnisotropy::Anisotropy_uniaxialcubic,"uniaxialcubic"sv }
                            } } };
    static constexpr const auto IAnisotropyValues{ IAnisotropyMap.get_key_array() };
    namespace {
        constexpr auto getValidIAnisotropyValues() { 
            std::array<IAnisotropy, IAnisotropyValues.size()-1> ret;
            std::copy(begin(IAnisotropyValues)+1,end(IAnisotropyValues), begin(ret));
            return ret;
        }
    }
    static constexpr const auto ValidIAnisotropyValues {getValidIAnisotropyValues()};
    static_assert(ValidIAnisotropyValues[0]!=IAnisotropy::undefined);

    template<typename T>
    T from_string(const std::string&);

    template<typename T> requires std::is_enum_v<T>
    static constexpr auto& get_enum_string_mapping(T);

    template<>
    constexpr auto& get_enum_string_mapping<IAnisotropy>(IAnisotropy)
    {
        return IAnisotropyMap;
    }

    ///-------------------------------------------------------------------------------------------------
    /// <summary>	Gets the enum IAnisotropy from a string. </summary>
    ///
    /// <param name="AnisoString">	The string to transform </param>
    ///
    /// <returns>	An Enum representing the string  </returns>
    ///-------------------------------------------------------------------------------------------------
    template<>
    IAnisotropy from_string<IAnisotropy>(const std::string& AnisoString);

    IAnisotropy from_string(std::string_view AnisoString, IAnisotropy& val);
    std::string to_string(const IAnisotropy& field);

    static constexpr std::string_view as_string_view(IAnisotropy field) {
        return IAnisotropyMap[field];
    }
}
#endif	// INC_AnisotropyList_H
// end of Problems\Anisotropy\AnisotropyList.h
