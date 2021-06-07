///---------------------------------------------------------------------------------------------------
// file:        FieldProperties.h
//
// summary:     Declares the field properties class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 20.06.2016

#ifndef INC_FieldProperties_H
#define INC_FieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <string>
#include <variant>

#include <MyCEL/basics/enumhelpers.h>
#include <MyCEL/basics/templatehelpers.h>

#include <SerAr/Core/NamedValue.h>
#include <SerAr/Core/NamedEnumVariant.hpp>
#include <SerAr/Core/InputArchive.h>
#include <SerAr/Core/OutputArchive.h>

#include <Eigen/Core>

#include "Selectors/FieldSelector.h"

namespace Properties
{   
    template <typename prec>
    class FieldProperties
    {
        using ThisClass = FieldProperties<prec>;
        template<IField value>
        struct field_enum_property_mapping { using type = typename Selectors::FieldSelector<value>::template FieldParameters<prec>; };
    public:
        using field_variant = MyCEL::enum_variant<IField, field_enum_property_mapping, IFieldValues>;
        using Precision = prec;
        field_variant                           fieldproperties{ {IField::Field_Zero}, {}};

        explicit FieldProperties(field_variant fieldP) : fieldproperties(fieldP) {};
        FieldProperties() = default;

        const IField& getTypeOfField() const noexcept { return fieldproperties.value; };

        template<IField value>
        const auto& getFieldParameters() const noexcept {
            return fieldproperties.template getEmumVariantType<value>();
        };
        template<IField value>
        auto& getFieldParameters() noexcept {
            return fieldproperties.template getEmumVariantType<value>();
        };

        template<typename Archive>
        void serialize(Archive& ar)
        {
            ar(::SerAr::createNamedEnumVariant("Type_of_field",{},fieldproperties));
        }

    };
}

#endif    // INC_FieldProperties_H
// end of FieldProperties.h
///---------------------------------------------------------------------------------------------------
