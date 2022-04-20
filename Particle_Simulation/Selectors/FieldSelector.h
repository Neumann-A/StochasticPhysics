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

#include "Properties/FieldProperties.h"


#include "Selectors/BasicSelector.h"
#include <MyCEL/basics/enumhelpers.h>
#include <MyCEL/basics/templatehelpers.h>

#include "Fields/FieldList.h"
#include "Properties/Fields/AllFields.hpp"

#define FIELDSELECTORMAKRO(EnumValue, FieldClass, FieldParameter)                                \
 template <>                                                                    \
 struct FieldSelector< EnumValue > : MyCEL::enum_value_type<IField, EnumValue>        \
{                                                                                \
    template<typename prec>                                                        \
    using FieldType = FieldClass <prec>;                                        \
                                                                                \
    template<typename prec>                                                        \
    using Traits = FieldTraits< FieldType <prec> >;                                \
                                                                                     \
   template<typename prec>                                                             \
   using FieldProperties = typename Traits<prec>::FieldProperties;             \
                                                                                \
    template<typename prec>                                                        \
    using FieldParameters = FieldParameter<prec>;                \
};

namespace Selectors
{

    using namespace Properties;

    template <IField field>
    struct FieldSelector{};

    FIELDSELECTORMAKRO(IField::Field_Zero, ZeroField, ::Properties::Fields::Zero)
    FIELDSELECTORMAKRO(IField::Field_Constant, ConstantField, ::Properties::Fields::Constant)
    FIELDSELECTORMAKRO(IField::Field_Sinusoidal, SinusoidalField, ::Properties::Fields::Sinusoidal)
    FIELDSELECTORMAKRO(IField::Field_Lissajous, LissajousField, ::Properties::Fields::Lissajous)
    FIELDSELECTORMAKRO(IField::Field_Triangular, TriangularField, ::Properties::Fields::Triangular)
    FIELDSELECTORMAKRO(IField::Field_Rectangular, RectangularField, ::Properties::Fields::Rectangular)
    FIELDSELECTORMAKRO(IField::Field_Sinc, SincField,::Properties::Fields::Sinc)
    FIELDSELECTORMAKRO(IField::Field_Modsinc, ModulatedSincField,::Properties::Fields::ModulatedSinc)
}

#undef FIELDSELECTORMAKRO

#endif    // INC_FieldSelector_H
// end of FieldSelector.h
///---------------------------------------------------------------------------------------------------
