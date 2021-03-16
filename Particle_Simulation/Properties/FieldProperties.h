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
#include <SerAr/Core/InputArchive.h>
#include <SerAr/Core/OutputArchive.h>

#include <Eigen/Core>

#include "Selectors/FieldSelector.h"

namespace Properties
{   
    template <typename prec>
    class FieldProperties
    {
        template<IField value>
        struct field_enum_property_mapping { using type = typename Selectors::FieldSelector<value>::template FieldParameters<prec>; };
        template <IField... Values>
        using field_variant_helper_t = typename MyCEL::enum_variant_creator_t<IField, field_enum_property_mapping, Values...>;
    public:
        using field_variant = typename MyCEL::apply_nttp_t<IFieldValues, field_variant_helper_t>;
    private:
        typedef FieldProperties<prec>                                    ThisClass;

    public:
        typedef prec                                            Precision;

    private:
        IField                                   TypeOfField{ IField::Field_Zero };

    public:
        field_variant                           _FieldParameter{};

    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        explicit FieldProperties(const IField& field, field_variant fieldP)
            : TypeOfField(field), _FieldParameter(fieldP){
        };


        FieldProperties() {};

        const IField& getTypeOfField() const noexcept { return TypeOfField; };

        template<IField value>
        decltype(auto) getFieldParameters() const noexcept {
            using field_param_type = typename field_enum_property_mapping<value>::type;
            field_param_type res = std::get<field_param_type>(_FieldParameter);
            return res;
        };

        template<IField value>
        struct field_switch_case
        {
            template<typename Archive>
            void operator()(field_variant& field, Archive& ar)
            {
                using field_param_type = typename field_enum_property_mapping<value>::type;
                if (!std::holds_alternative<field_param_type>(field))
                {
                    field = field_param_type{};
                }
                Properties::Fields::serialize<prec, Archive>(std::get<field_param_type>(field), ar);
            }
        };

        struct field_default_switch_case
        {
            template<typename Archive>
            void operator()(field_variant& /* field */, Archive&/* ar */)
            {
                throw std::out_of_range{ "Type of field unknown!" };
            }
        };

        template<typename Archive>
        void serialize(Archive& ar)
        {
            std::string str{ to_string(TypeOfField) };
            ar(Archives::createNamedValue(std::string{ "Type_of_field" }, str));
            TypeOfField = from_string<decltype(TypeOfField)>(str);

            MyCEL::enum_switch::run<decltype(TypeOfField), field_default_switch_case, field_switch_case>(TypeOfField, _FieldParameter, ar);
        }

    };
}

#endif    // INC_FieldProperties_H
// end of FieldProperties.h
///---------------------------------------------------------------------------------------------------
