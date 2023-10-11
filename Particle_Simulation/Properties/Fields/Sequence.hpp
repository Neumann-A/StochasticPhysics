
#ifndef INC_SequenceFieldProperties_H
#define INC_SequenceFieldProperties_H
#pragma once

#include <SerAr/Core/InputArchive.h>
#include <SerAr/Core/NamedValue.h>
#include <SerAr/Core/OutputArchive.h>

#include <MyCEL/basics/enumhelpers.h>
#include <MyCEL/basics/templatehelpers.h>

#include <string>
#include <variant>
#include <vector>

#include "Selectors/BasicSelector.h"
#include <Eigen/Core>
#include <SerAr/Core/NamedEnumVariant.hpp>


#include "Constant.hpp"
#include "Field.hpp"
#include "Lissajous.hpp"
#include "ModulatedSinc.hpp"
#include "Rectangular.hpp"
#include "Sinc.hpp"
#include "Sinusoidal.hpp"
#include "Triangular.hpp"
#include "Zero.hpp"

 template <typename Field>
 class FieldTraits;

namespace SequenceFieldProp
{
    namespace
    {
        using namespace std::literals::string_view_literals;
    }
    /// <summary>	Values that represent the used fields. </summary>
    enum class ISequenceFields
    {
        Field_Zero = 1,
        Field_Constant,
        Field_Sinusoidal,
        Field_Lissajous,
        Field_Triangular,
        Field_Rectangular,
        Field_Sinc,
        Field_Modsinc
    };

    /// <summary>	Map used to change the IField enum to a string and vice versa. </summary>
    inline constexpr MyCEL::static_map<ISequenceFields, std::string_view, 8> ISequenceFieldsMap{{{
        {ISequenceFields::Field_Zero, "none"sv},
        {ISequenceFields::Field_Constant, "constant"sv},
        {ISequenceFields::Field_Sinusoidal, "sinusoidal"sv},
        {ISequenceFields::Field_Lissajous, "lissajous"sv},
        {ISequenceFields::Field_Triangular, "triangular"sv},
        {ISequenceFields::Field_Rectangular, "rectangular"sv},
        {ISequenceFields::Field_Sinc, "sinc"sv},
        {ISequenceFields::Field_Modsinc, "modsinc"sv},
    }}};
    inline constexpr auto ISequenceFieldsValues{ISequenceFieldsMap.get_key_array()};

    template <typename T>
    T from_string(const std::string&);

    template <>
    inline ISequenceFields from_string<ISequenceFields>(const std::string& FieldString)
    {
        return ISequenceFieldsMap[FieldString];
    }

    inline ISequenceFields from_string(std::string_view FieldString, ISequenceFields& value)
    {
        return (value = ISequenceFieldsMap[FieldString]);
    }

    inline std::string to_string(const ISequenceFields& field) { return std::string{ISequenceFieldsMap[field]}; }

} // namespace SequenceFieldProp

///---------------------------------------------------------------------------------------------------


#define FIELDSELECTORMAKRO(EnumValue, FieldClass, FieldParameter)                                                      \
    template <>                                                                                                        \
    struct SequenceFieldSelector<EnumValue> : MyCEL::enum_value_type<SequenceFieldProp::ISequenceFields, EnumValue>    \
    {                                                                                                                  \
        template <typename prec>                                                                                       \
        using FieldType = FieldClass<prec>;                                                                            \
                                                                                                                       \
        template <typename prec>                                                                                       \
        using Traits = FieldTraits<FieldType<prec>>;                                                                   \
                                                                                                                       \
        template <typename prec>                                                                                       \
        using FieldParameters = FieldParameter<prec>;                                                                  \
    };

namespace Selectors
{

    using namespace Properties;

    template <SequenceFieldProp::ISequenceFields field>
    struct SequenceFieldSelector
    {
    };

    FIELDSELECTORMAKRO(SequenceFieldProp::ISequenceFields::Field_Zero, ZeroField, ::Properties::Fields::Zero)
    FIELDSELECTORMAKRO(SequenceFieldProp::ISequenceFields::Field_Constant, ConstantField,
                       ::Properties::Fields::Constant)
    FIELDSELECTORMAKRO(SequenceFieldProp::ISequenceFields::Field_Sinusoidal, SinusoidalField,
                       ::Properties::Fields::Sinusoidal)
    FIELDSELECTORMAKRO(SequenceFieldProp::ISequenceFields::Field_Lissajous, LissajousField,
                       ::Properties::Fields::Lissajous)
    FIELDSELECTORMAKRO(SequenceFieldProp::ISequenceFields::Field_Triangular, TriangularField,
                       ::Properties::Fields::Triangular)
    FIELDSELECTORMAKRO(SequenceFieldProp::ISequenceFields::Field_Rectangular, RectangularField,
                       ::Properties::Fields::Rectangular)
    FIELDSELECTORMAKRO(SequenceFieldProp::ISequenceFields::Field_Sinc, SincField, ::Properties::Fields::Sinc)
    FIELDSELECTORMAKRO(SequenceFieldProp::ISequenceFields::Field_Modsinc, ModulatedSincField,
                       ::Properties::Fields::ModulatedSinc)
} // namespace Selectors

#undef FIELDSELECTORMAKRO


namespace Properties::Fields
{
    template <typename prec>
    struct Sequence : General<prec>
    {

        typedef Eigen::Matrix<prec, 3, 1> Vec3D;

        using Precision = prec;

        static const IField TypeOfField{Properties::IField::Field_Sequence};
        template <SequenceFieldProp::ISequenceFields value>
        struct field_enum_property_mapping
        {
            using type = typename Selectors::SequenceFieldSelector<value>::template FieldParameters<prec>;
        };

        using field_variant = MyCEL::enum_variant<SequenceFieldProp::ISequenceFields, field_enum_property_mapping,
                                                  SequenceFieldProp::ISequenceFieldsValues>;
        std::vector<size_t> stepsPerField;
        std::vector<field_variant> fields;


        using ThisClass = Sequence<prec>;
    };

    template <typename Precision, typename Archive>
    void load(Sequence<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("stepsPerField",val.stepsPerField));
        val.fields.resize(val.stepsPerField.size());
        for (std::size_t i = 0; i < val.stepsPerField.size(); i++) {
            ar(::SerAr::createNamedEnumVariant("Field_" + std::to_string(i+1),
                                               "Field_" + std::to_string(i+1)+"_Parameters", val.fields[i]));

        }
    }

        template <typename Precision, typename Archive>
    void save(Sequence<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("stepsPerField",val.stepsPerField));
        for (std::size_t i = 0; i < val.stepsPerField.size(); i++) {
            ar(::SerAr::createNamedEnumVariant("Field_" + std::to_string(i+1),
                                               "Field_" + std::to_string(i+1)+"_Parameters", val.fields[i]));

        }
    }
} // namespace Properties::Fields
#endif
