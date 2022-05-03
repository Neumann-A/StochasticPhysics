
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

#include "Constant.hpp"
#include "Field.hpp"
#include "Lissajous.hpp"
#include "ModulatedSinc.hpp"
#include "Rectangular.hpp"
#include "Selectors/BasicSelector.h"
#include "Sinc.hpp"
#include "Sinusoidal.hpp"
#include "Triangular.hpp"
#include "Zero.hpp"
#include <Eigen/Core>
#include <SerAr/Core/NamedEnumVariant.hpp>

#ifndef INC_SEQUENCEFIELDS_H
#define INC_SEQUENCEFIELDS_H
template <typename prec>
class ZeroField;
template <typename prec>
class ConstantField;
template <typename prec>
class LissajousField;
template <typename prec>
class RectangularField;
template <typename prec>
class SinusoidalField;
template <typename prec>
class TriangularField;
template <typename prec>
class SincField;
template <typename prec>
class ModulatedSincField;

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
#endif // INC_FieldList_H
// end of Fields\FieldList.h

#ifndef INC_SEQUENCEFieldSelector_H
#define INC_SEQUENCEFieldSelector_H
///---------------------------------------------------------------------------------------------------
#pragma once

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

#endif

namespace Properties::Fields
{
    template <typename prec>
    struct Sequence : General<prec>
    {

        typedef Eigen::Matrix<prec, 3, 1> Vec3D;
        //std::vector<FieldProperties*> fields(numberOfFields,{{IField::Field_Zero}, {}});

        using Precision = prec;
        //field_variant                           fieldproperties{ {IField::Field_Zero}, {}};

        static const IField TypeOfField{Properties::IField::Field_Sequence};
        template <SequenceFieldProp::ISequenceFields value>
        struct field_enum_property_mapping
        {
            using type = typename Selectors::SequenceFieldSelector<value>::template FieldParameters<prec>;
        };

        using field_variant = MyCEL::enum_variant<SequenceFieldProp::ISequenceFields, field_enum_property_mapping,
                                                  SequenceFieldProp::ISequenceFieldsValues>;
        int numberOfFields  = 1;
        std::vector<field_variant> fields;

        using ThisClass = Sequence<prec>;
    };

    template <typename Precision, typename Archive>
    void serialize(Sequence<Precision>& val, Archive& ar)
    {

        ar(Archives::createNamedValue("numberOfFields", val.numberOfFields));
        for (int i = 0; i < val.numberOfFields; i++) {
            //val.fields.push_back({ {SequenceFieldProp::ISequenceFields::Field_Zero}, {}});
            ar(::SerAr::createNamedEnumVariant("Field_" + std::to_string(i+1),
                                               "Field_" + std::to_string(i+1), val.fields[i]));
        }
    }
} // namespace Properties::Fields
#endif
