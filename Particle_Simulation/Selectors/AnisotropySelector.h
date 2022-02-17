///---------------------------------------------------------------------------------------------------
// file:		AnisotropySelector.h
//
// summary: 	Declares the anisotropy selector class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.07.2016

#ifndef INC_AnisotropySelector_H
#define INC_AnisotropySelector_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <MyCEL/basics/enumhelpers.h>
#include <MyCEL/basics/templatehelpers.h>
#include "Problems/Anisotropy/AnisotropyList.h"
#include "Properties/Anisotropy/All.hpp"

namespace Selectors
{
    using namespace Properties;

    template <IAnisotropy Anisotropy>
    struct AnisotropyTypeSelector;

    template <>
    struct AnisotropyTypeSelector<IAnisotropy::Anisotropy_uniaxial> : MyCEL::enum_value_type<IAnisotropy, IAnisotropy::Anisotropy_uniaxial>
    {
        template<typename prec>
        using type = typename Problems::Anisotropy::UniaxialAnisotropy<prec>;
        template<typename prec>
        using traits = typename Problems::Anisotropy::AnisotropyTraits<type<prec>>;
        template<typename prec>
        using input_parameter = typename ::Properties::Anisotropy::Uniaxial<prec>;
    };
    template <>
    struct AnisotropyTypeSelector<IAnisotropy::Anisotropy_cubic> : MyCEL::enum_value_type<IAnisotropy, IAnisotropy::Anisotropy_cubic>
    {
        template<typename prec>
        using type = typename Problems::Anisotropy::CubicAnisotropy<prec>;
        template<typename prec>
        using traits = typename Problems::Anisotropy::AnisotropyTraits<type<prec>>;
        template<typename prec>
        using input_parameter = typename ::Properties::Anisotropy::Cubic<prec>;
    };
    template <>
    struct AnisotropyTypeSelector<IAnisotropy::Anisotropy_mixed> : MyCEL::enum_value_type<IAnisotropy, IAnisotropy::Anisotropy_mixed>
    {
        template<typename prec>
        using type = typename Problems::Anisotropy::MixedAnisotropy<prec>;
        template<typename prec>
        using traits = typename Problems::Anisotropy::AnisotropyTraits<type<prec>>;
        template<typename prec>
        using input_parameter = typename ::Properties::Anisotropy::Mixed<prec>;
    };
    template <>
    struct AnisotropyTypeSelector<IAnisotropy::Anisotropy_uniaxialcubic> : MyCEL::enum_value_type<IAnisotropy, IAnisotropy::Anisotropy_uniaxialcubic>
    {
        template<typename prec>
        using type = typename Problems::Anisotropy::UniaxialCubicAnisotropy<prec>;
        template<typename prec>
        using traits = typename Problems::Anisotropy::AnisotropyTraits<type<prec>>;
        template<typename prec>
        using input_parameter = typename ::Properties::Anisotropy::UniaxialCubic<prec>;
    };
}

#endif	// INC_AnisotropySelector_H
// end of AnisotropySelector.h
///---------------------------------------------------------------------------------------------------
