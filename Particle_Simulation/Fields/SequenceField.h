
#ifndef INC_SequenceField_H
#define INC_SequenceField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <cmath>
#include <vector>
#include <exception>
#include <variant>
#include <type_traits>

#include "Properties/FieldProperties.h"

#include "SDEFramework/GeneralField.h"

#include  <MyCEL/math/math_constants.h>

//Field Includes
#include "Fields/SinusoidalField.h"
#include "Fields/LissajousField.h"
#include "Fields/ZeroField.h"
#include "Fields/ConstantField.h"
#include "Fields/TriangularField.h"
#include "Fields/RectangularField.h"
#include "Fields/SincField.h"
#include "Fields/ModulatedSincField.h"

template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;

template <typename precision>
class SequenceField : public GeneralField<SequenceField<precision>>
{
public:
    using ThisClass = SequenceField<precision>;
    using Precision = precision;
    using Base = GeneralField<ThisClass>;
    using Traits = typename Base::Traits;
    using FieldProperties = typename Traits::FieldProperties;
    using FieldVector = typename Traits::FieldVector;
    using FieldParams = typename Traits::FieldParameters;

    //     template <SequenceFieldProp::ISequenceFields value>
    //     struct field_enum_property_mapping
    //     {
    //         using type = typename Selectors::SequenceFieldSelector<value>::template FieldParameters<Precision>;
    //     };

    // using field_variant = MyCEL::enum_variant<SequenceFieldProp::ISequenceFields, field_enum_property_mapping,
    //                                               SequenceFieldProp::ISequenceFieldsValues>;

    using impl_field=std::variant<ZeroField<precision>,ConstantField<precision>,SinusoidalField<precision>,LissajousField<precision>,TriangularField<precision>,RectangularField<precision>,SincField<precision>,ModulatedSincField<precision>>;
    using field_map=std::map<SequenceFieldProp::ISequenceFields,impl_field>;

private:
    
    FieldParams	params;
    std::vector<impl_field> exfield;


protected:
public:
    constexpr SequenceField(const typename Traits::FieldParameters &input):params(input)
    {
        //auto val=input;
        // for(const auto& val : input.fields){
        //    std::visit(overload{
        //        [](const typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Zero>::template FieldParameters<precision>& input){
        //            typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Zero>::template FieldType<precision> seqfield(input);
        //            //exfield.push_back(seqfield);
        //        },
        //        [](const typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Constant>::template FieldParameters<precision>&input){
        //         typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Constant>::template FieldType<precision> seqfield(input);
        //         //exfield.push_back(seqfield);
        //        },
        //        [](const typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Sinusoidal>::template FieldParameters<precision>&input){
        //         typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Sinusoidal>::template FieldType<precision> seqfield(input);
        //         //exfield.push_back(seqfield);
        //        },
        //        [](const typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Lissajous>::template FieldParameters<precision>&input){
        //            typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Lissajous>::template FieldType<precision> seqfield(input);
        //            //exfield.push_back(seqfield);
        //        },
        //        [](const typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Triangular>::template FieldParameters<precision>&input){
        //            typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Triangular>::template FieldType<precision> seqfield(input);
        //           // exfield.push_back(seqfield);
        //        },
        //        [](const typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Rectangular>::template FieldParameters<precision>&input){
        //            typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Rectangular>::template FieldType<precision> seqfield(input);
        //           // exfield.push_back(seqfield);
        //        },
        //        [](const typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Sinc>::template FieldParameters<precision>&input){
        //            typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Sinc>::template FieldType<precision> seqfield(input);
        //           // exfield.push_back(seqfield);
        //        },
        //        [](const typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Modsinc>::template FieldParameters<precision>&input){
        //            typename Selectors::SequenceFieldSelector<SequenceFieldProp::ISequenceFields::Field_Modsinc>::template FieldType<precision> seqfield(input);
        //           // exfield.push_back(seqfield);
        //        }
        //    },val);
        //}
    };
    constexpr SequenceField(const FieldProperties &pars):SequenceField(pars.template getFieldParameters<Traits::Field_type>())
    {
        
    };

    inline FieldVector getField(const Precision& time) const noexcept
    {
        return FieldVector::Zero();;
    }
};


template<typename prec>
class FieldTraits<SequenceField<prec>>
{
public:
    using Precision = prec;
    using FieldProperties = Properties::FieldProperties<Precision>;
    using FieldVector = Eigen::Matrix<Precision, 3, 1>;
    using FieldVectorStdAllocator =  std::allocator<FieldVector>;
    using FieldParameters = ::Properties::Fields::Sequence<Precision>;
    static constexpr auto Field_type = ::Properties::IField::Field_Sequence;
};

#endif	// INC_SequenceField_H
// end of LissajousField.h
///---------------------------------------------------------------------------------------------------
