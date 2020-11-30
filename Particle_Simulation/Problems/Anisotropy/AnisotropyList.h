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
    enum class IAnisotropy { Anisotropy_uniaxial = 1, Anisotropy_cubic, Anisotropy_mixed, Anisotropy_uniaxialcubic};

    /// <summary>	Map used to change the IAnisotropy enum to a string and vice versa. </summary>
    constexpr const MyCEL::static_map<IAnisotropy, std::string_view, 3> IAnisotropyMap { { { 
                            { IAnisotropy::Anisotropy_uniaxial,     "uniaxial"sv },
                            { IAnisotropy::Anisotropy_cubic,        "cubic"sv },
                            { IAnisotropy::Anisotropy_mixed,        "mixed"sv },
                            { IAnisotropy::Anisotropy_uniaxialcubic,"uniaxialcubic"sv }
                            } } };
    constexpr const auto IAnisotropyValues = IAnisotropyMap.get_key_array();

    template<typename T>
    T from_string(const std::string&);

    template<IAnisotropy value>
    struct AnisotropySelector;

    ///-------------------------------------------------------------------------------------------------
    /// <summary>	Gets the enum IAnisotropy from a string. </summary>
    ///
    /// <param name="AnisoString">	The string to transform </param>
    ///
    /// <returns>	An Enum representing the string  </returns>
    ///-------------------------------------------------------------------------------------------------
    template<>
    IAnisotropy from_string<IAnisotropy>(const std::string& AnisoString);
    std::string to_string(const IAnisotropy& field);
}
#endif	// INC_AnisotropyList_H
// end of Problems\Anisotropy\AnisotropyList.h
