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
#include <type_traits>
#include <cstddef>
#include <exception>

#include <string>
#include <map>
#include <vector>
#include <algorithm>
#include <functional>
#include <variant>

#include <MyCEL/basics/enumhelpers.h>
#include <MyCEL/basics/templatehelpers.h>

#include <MyCEL/math/Geometry.h>
#include <MyCEL/basics/BasicIncludes.h>

#include <SerAr/Core/NamedValue.h>
#include <SerAr/Core/InputArchive.h>
#include <SerAr/Core/OutputArchive.h>

#include <Eigen/Core>
//#include <Eigen/StdVector>


//#include "Fields/FieldList.h"
#include "Selectors/FieldSelector.h"

//include "Properties/Fields/AllFields.hpp"

//#include "Fields/FieldList.h"
//#include "Properties/Fields/AllFields.hpp"

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
        typedef Eigen::Matrix<prec, 3, 1>                                Vec3D;
        //typedef std::vector<Vec3D,  std::allocator<Vec3D>>            Vec3DList;
        using Vec3DList = std::vector<Vec3D>;
    public:
        typedef prec                                            Precision;

    private:
        IField                                   _TypeOfField{ IField::Field_Zero };
        Vec3DList                                _Amplitudes{ Vec3D::Zero() };

    public:
        field_variant                           _FieldProperties{};

    private:
        static inline std::string buildSerilizationString(const char* name, const std::size_t& number)
        {
            return std::string{ name + BasicTools::toStringScientific(number) };
        }

        template<typename Archive, typename Container>
        static inline void serializeVector(Archive& ar, const char* sizevector, const char* vecname, Container& vector)
        {
            auto elements = vector.size();
            ar(Archives::createNamedValue(sizevector, elements));
            vector.resize(elements);

            std::size_t counter{ 0 };
            for (auto& it : vector)
            {
                ar(Archives::createNamedValue(buildSerilizationString(vecname, ++counter), it));
            }
        }
    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        explicit FieldProperties(const IField& field, const Vec3DList& amplitudes, field_variant fieldP)
            : _TypeOfField(field), _Amplitudes(amplitudes), _FieldProperties(fieldP){
        };


        FieldProperties() {};

        const IField& getTypeOfField() const noexcept { return _TypeOfField; };
        const Vec3DList& getAmplitudes() const noexcept { return _Amplitudes; };

        Vec3DList& getAmplitudes() noexcept { return _Amplitudes; };


        template<IField value>
        decltype(auto) getFieldParameters() const noexcept {
            using field_param_type = typename field_enum_property_mapping<value>::type;
            field_param_type res = std::get<field_param_type>(_FieldProperties);
            return res;
        };

        inline void setAmplitudes(const Vec3DList& amplitudes) noexcept { _Amplitudes = amplitudes; };

        static std::string getSectionName() noexcept { return std::string{ "Field_Properties" }; };


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
                ar(Archives::createNamedValue(::Properties::Fields::General<prec>::template getSectionName<value>(), std::get<field_param_type>(field)));
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
            std::string str{ to_string(_TypeOfField) };
            ar(Archives::createNamedValue(std::string{ "Type_of_field" }, str));
            _TypeOfField = from_string<decltype(_TypeOfField)>(str);

            serializeVector(ar, "Number_of_Amplitudes", "Amplitude_", _Amplitudes);

            MyCEL::enum_switch::run<decltype(_TypeOfField), field_default_switch_case, field_switch_case>(_TypeOfField, _FieldProperties, ar);
        }

    };
}

#endif    // INC_FieldProperties_H
// end of FieldProperties.h
///---------------------------------------------------------------------------------------------------
