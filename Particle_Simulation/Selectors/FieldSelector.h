///---------------------------------------------------------------------------------------------------
// file:        FieldSelector.h
//
// summary:     Declares the field selector class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.07.2016

#ifndef INC_FieldSelector_H
#define INC_FieldSelector_H
///---------------------------------------------------------------------------------------------------
#pragma once

//#include "Selectors/BasicSelector.h"
//#include "Properties/FieldProperties.h"
//
//#include "Fields/SinusoidalField.h"
//#include "Fields/LissajousField.h"
//#include "Fields/ZeroField.h"
//#include "Fields/ConstantField.h"
//#include "Fields/TriangularField.h"
//#include "Fields/RectangularField.h"
//#include "Fields/SincField.h"

#include <MyCEL/basics/enumhelpers.h>
#include <MyCEL/basics/templatehelpers.h>

#include "Fields/FieldList.h"
#include "Properties/Fields/AllFields.hpp"

//#define FIELDSELECTORMAKRO(EnumValue, FieldClass)                                \
// template <>                                                                    \
// struct FieldSelector< EnumValue > : public BasicSelector<FieldSelector< EnumValue >>        \
//{                                                                                \
//    template<typename prec>                                                        \
//    using FieldType = FieldClass <prec>;                                        \
//                                                                                \
//    template<typename prec>                                                        \
//    using Traits = FieldTraits< FieldType <prec> >;                                \
//                                                                                \
//    template<typename prec>                                                        \
//    using FieldParameters = typename Traits<prec>::FieldProperties;                \
//};

namespace Selectors
{
    using namespace Properties;

    template <IField field>
    struct FieldSelector{};

    template <>
    struct FieldSelector<IField::Field_Zero> :MyCEL::enum_value_type<IField, IField::Field_Zero>
    {
        template<typename prec>
        using FieldType = typename ZeroField <prec>;

        template<typename prec>
        using Traits = typename FieldTraits< FieldType <prec> >;

        template<typename prec>
        using FieldProperties = typename Traits<prec>::FieldProperties;

        template<typename prec>
        using FieldParameters = typename ::Properties::Fields::Zero<prec>;
    };

    template <>
    struct FieldSelector<IField::Field_Constant> :MyCEL::enum_value_type<IField, IField::Field_Constant>
    {
        template<typename prec>
        using FieldType = typename ConstantField <prec>;

        template<typename prec>
        using Traits = typename FieldTraits< FieldType <prec> >;

        template<typename prec>
        using FieldProperties = typename Traits<prec>::FieldProperties;

        template<typename prec>
        using FieldParameters = typename ::Properties::Fields::Constant<prec>;
    };

    template <>                                                                    
        struct FieldSelector<IField::Field_Sinusoidal> :MyCEL::enum_value_type<IField, IField::Field_Sinusoidal>
    {                                                                                
        template<typename prec>                                                        
        using FieldType = typename SinusoidalField <prec>;
        
        template<typename prec>                                                        
        using Traits = typename FieldTraits< FieldType <prec> >;

        template<typename prec>
        using FieldProperties = typename Traits<prec>::FieldProperties;
        
        template<typename prec>                                                        
        using FieldParameters = typename ::Properties::Fields::Sinusoidal<prec>;
    };

        template <>
        struct FieldSelector<IField::Field_Rectangular> :MyCEL::enum_value_type<IField, IField::Field_Rectangular>
        {
            template<typename prec>
            using FieldType = typename RectangularField <prec>;

            template<typename prec>
            using Traits = typename FieldTraits< FieldType <prec> >;

            template<typename prec>
            using FieldProperties = typename Traits<prec>::FieldProperties;

            template<typename prec>
            using FieldParameters = typename ::Properties::Fields::Rectangular<prec>;
        };

        template <>
        struct FieldSelector<IField::Field_Sinc> :MyCEL::enum_value_type<IField, IField::Field_Sinc>
        {
            template<typename prec>
            using FieldType = typename SincField <prec>;

            template<typename prec>
            using Traits = typename FieldTraits< FieldType <prec> >;

            template<typename prec>
            using FieldProperties = typename Traits<prec>::FieldProperties;

            template<typename prec>
            using FieldParameters = typename ::Properties::Fields::Sinc<prec>;
        };

        template <>
        struct FieldSelector<IField::Field_Lissajous> :MyCEL::enum_value_type<IField, IField::Field_Lissajous>
        {
            template<typename prec>
            using FieldType = typename LissajousField <prec>;

            template<typename prec>
            using Traits = typename FieldTraits< FieldType <prec> >;

            template<typename prec>
            using FieldProperties = typename Traits<prec>::FieldProperties;

            template<typename prec>
            using FieldParameters = typename ::Properties::Fields::Lissajous<prec>;
        };
    
        template <>
        struct FieldSelector<IField::Field_Triangular> :MyCEL::enum_value_type<IField, IField::Field_Triangular>
        {
            template<typename prec>
            using FieldType = typename TriangularField <prec>;

            template<typename prec>
            using Traits = typename FieldTraits< FieldType <prec> >;

            template<typename prec>
            using FieldProperties = typename Traits<prec>::FieldProperties;

            template<typename prec>
            using FieldParameters = typename ::Properties::Fields::Triangular<prec>;
        };

    //FIELDSELECTORMAKRO(IField::Field_Zero, ZeroField)
    //FIELDSELECTORMAKRO(IField::Field_Constant, ConstantField)
    //FIELDSELECTORMAKRO(IField::Field_Sinusoidal, SinusoidalField)
    //FIELDSELECTORMAKRO(IField::Field_Lissajous, LissajousField)
    //FIELDSELECTORMAKRO(IField::Field_Triangular, TriangularField)
    //FIELDSELECTORMAKRO(IField::Field_Rectangular, RectangularField)
    //FIELDSELECTORMAKRO(IField::Field_Sinc, SincField)
}

//#undef FIELDSELECTORMAKRO

#endif    // INC_FieldSelector_H
// end of FieldSelector.h
///---------------------------------------------------------------------------------------------------
