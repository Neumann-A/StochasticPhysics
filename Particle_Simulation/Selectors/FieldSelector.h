///---------------------------------------------------------------------------------------------------
// file:		FieldSelector.h
//
// summary: 	Declares the field selector class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.07.2016

#ifndef INC_FieldSelector_H
#define INC_FieldSelector_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "Selectors/BasicSelector.h"
#include "Properties/FieldProperties.h"

#include "Fields/SinusoidalField.h"
#include "Fields/LissajousField.h"
#include "Fields/ZeroField.h"
#include "Fields/ConstantField.h"
#include "Fields/TriangularField.h"

#define FIELDSELECTORMAKRO(EnumValue, FieldClass)								\
 template <>																    \
 struct FieldSelector< EnumValue > : public BasicSelector<FieldSelector< EnumValue >>		\
{																			    \
    template<typename prec>													    \
    using FieldType = FieldClass <prec>;									    \
                                                                                \
    template<typename prec>												        \
    using Traits = FieldTraits< FieldType <prec> >;								\
                                                                                \
    template<typename prec>														\
    using FieldParameters = typename Traits<prec>::FieldProperties;				\
};

namespace Selectors
{
    using namespace Properties;

    template <IField field>
    struct FieldSelector : public BasicSelector<FieldSelector<field>> {};

    FIELDSELECTORMAKRO(IField::Field_Zero, ZeroField)
    FIELDSELECTORMAKRO(IField::Field_Constant, ConstantField)
    FIELDSELECTORMAKRO(IField::Field_Sinusoidal, SinusoidalField)
    FIELDSELECTORMAKRO(IField::Field_Lissajous, LissajousField)
    FIELDSELECTORMAKRO(IField::Field_Triangular, TriangularField)
}

#undef FIELDSELECTORMAKRO

#endif	// INC_FieldSelector_H
// end of FieldSelector.h
///---------------------------------------------------------------------------------------------------
