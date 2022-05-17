
#ifndef INC_SequenceField_H
#define INC_SequenceField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <MyCEL/math/math_constants.h>

#include <cmath>
#include <exception>
#include <type_traits>
#include <variant>
#include <vector>

#include "Properties/FieldProperties.h"
#include "SDEFramework/GeneralField.h"

//Field Includes
#include "Fields/ConstantField.h"
#include "Fields/LissajousField.h"
#include "Fields/ModulatedSincField.h"
#include "Fields/RectangularField.h"
#include "Fields/SincField.h"
#include "Fields/SinusoidalField.h"
#include "Fields/TriangularField.h"
#include "Fields/ZeroField.h"

template <class... Ts>
struct SeqFieldOverload : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
SeqFieldOverload(Ts...) -> SeqFieldOverload<Ts...>;

template <typename precision>
class SequenceField : public GeneralField<SequenceField<precision>>
{
public:
    using ThisClass       = SequenceField<precision>;
    using Precision       = precision;
    using Base            = GeneralField<ThisClass>;
    using Traits          = typename Base::Traits;
    using FieldProperties = typename Traits::FieldProperties;
    using FieldVector     = typename Traits::FieldVector;
    using FieldParams     = typename Traits::FieldParameters;

    template <SequenceFieldProp::ISequenceFields Field>
    using type = typename Selectors::SequenceFieldSelector<Field>::template FieldParameters<Precision>;

    template <SequenceFieldProp::ISequenceFields Field>
    using ExFieldtype = typename Selectors::SequenceFieldSelector<Field>::template FieldType<Precision>;

    using impl_field = std::variant<ZeroField<precision>, ConstantField<precision>, SinusoidalField<precision>,
                                    LissajousField<precision>, TriangularField<precision>, RectangularField<precision>,
                                    SincField<precision>, ModulatedSincField<precision>>;

private:
    size_t step = 1;
    FieldParams params;
    std::vector<size_t> cumulatedSteps;
    std::vector<impl_field> exfield;

protected:
public:
    constexpr SequenceField(const typename Traits::FieldParameters &input)
        : params(input)
    {
        size_t lastVal = 0;
        for (size_t i = 0; i < input.stepsPerField.size(); i++) {
            cumulatedSteps.push_back(input.stepsPerField[i] + lastVal);
            lastVal = cumulatedSteps[i];
        }
        for (const auto &value : input.fields) {
            auto currentFieldVar = value.variant;

            std::visit(
                SeqFieldOverload{
                    [this](type<SequenceFieldProp::ISequenceFields::Field_Zero> & Fieldparams) {
                        ExFieldtype<SequenceFieldProp::ISequenceFields::Field_Zero> seqfield(Fieldparams);
                        impl_field tmp = seqfield;
                        (this->exfield).push_back(tmp);
                    },
                    [this](type<SequenceFieldProp::ISequenceFields::Field_Constant> &Fieldparams) {
                        ExFieldtype<SequenceFieldProp::ISequenceFields::Field_Constant> seqfield(Fieldparams);
                        impl_field tmp = seqfield;
                        (this->exfield).push_back(tmp);
                    },
                    [this](type<SequenceFieldProp::ISequenceFields::Field_Sinusoidal> &Fieldparams) {
                        ExFieldtype<SequenceFieldProp::ISequenceFields::Field_Sinusoidal> seqfield(Fieldparams);
                        impl_field tmp = seqfield;
                        (this->exfield).push_back(tmp);
                    },
                    [this](type<SequenceFieldProp::ISequenceFields::Field_Lissajous> &Fieldparams) {
                        ExFieldtype<SequenceFieldProp::ISequenceFields::Field_Lissajous> seqfield(Fieldparams);
                        impl_field tmp = seqfield;
                        (this->exfield).push_back(tmp);
                    },
                    [this](type<SequenceFieldProp::ISequenceFields::Field_Triangular> &Fieldparams) {
                        ExFieldtype<SequenceFieldProp::ISequenceFields::Field_Triangular> seqfield(Fieldparams);
                        impl_field tmp = seqfield;
                        (this->exfield).push_back(tmp);
                    },
                    [this](type<SequenceFieldProp::ISequenceFields::Field_Rectangular> &Fieldparams) {
                        ExFieldtype<SequenceFieldProp::ISequenceFields::Field_Rectangular> seqfield(Fieldparams);
                        impl_field tmp = seqfield;
                        (this->exfield).push_back(tmp);
                    },
                    [this](type<SequenceFieldProp::ISequenceFields::Field_Sinc> &Fieldparams) {
                        ExFieldtype<SequenceFieldProp::ISequenceFields::Field_Sinc> seqfield(Fieldparams);
                        impl_field tmp = seqfield;
                        (this->exfield).push_back(tmp);
                    },
                    [this](type<SequenceFieldProp::ISequenceFields::Field_Modsinc> &Fieldparams) {
                        ExFieldtype<SequenceFieldProp::ISequenceFields::Field_Modsinc> seqfield(Fieldparams);
                        impl_field tmp = seqfield;
                        (this->exfield).push_back(tmp);
                    },
                },
                currentFieldVar);
        }
    };

    inline FieldVector getCurrentField(impl_field &field, const Precision &time)
    {
        switch (field.index()) {
        case 0:{
            return std::get<ZeroField<precision>>(field).getField(time);
            break;}
        case 1:{
            return std::get<ConstantField<precision>>(field).getField(time);
            break;}
        case 2:{
            return std::get<SinusoidalField<precision>>(field).getField(time);
            break;}
        case 3:{
            return std::get<LissajousField<precision>>(field).getField(time);
            break;}
        case 4:{
            return std::get<TriangularField<precision>>(field).getField(time);
            break;}
        case 5:{
            return std::get<RectangularField<precision>>(field).getField(time);
            break;}
        case 6:{
            return std::get<SincField<precision>>(field).getField(time);
            break;}
        case 7:{
            return std::get<ModulatedSincField<precision>>(field).getField(time);
            break;}
        default:
            throw std::runtime_error{ "getCurrentField: unknown active variant type" };
            break;
        }
    }
    constexpr SequenceField(const FieldProperties &pars)
        : SequenceField(pars.template getFieldParameters<Traits::Field_type>()){

          };

    inline FieldVector getField(const Precision &time)
    {
        for (size_t i = 0; i < exfield.size(); i++) {
            if (step <= cumulatedSteps[i]) {
                    step++;
                    return getCurrentField(exfield[i], time);
            }
            else if (step > cumulatedSteps.back()) {
                    step=2;
                    return getField(time);
            }
        }
        throw std::runtime_error{ "no Valid field" };
    }
};

template <typename prec>
class FieldTraits<SequenceField<prec>>
{
public:
    using Precision                  = prec;
    using FieldProperties            = Properties::FieldProperties<Precision>;
    using FieldVector                = Eigen::Matrix<Precision, 3, 1>;
    using FieldVectorStdAllocator    = std::allocator<FieldVector>;
    using FieldParameters            = ::Properties::Fields::Sequence<Precision>;
    static constexpr auto Field_type = ::Properties::IField::Field_Sequence;
};

#endif // INC_SequenceField_H
// end of LissajousField.h
///---------------------------------------------------------------------------------------------------
