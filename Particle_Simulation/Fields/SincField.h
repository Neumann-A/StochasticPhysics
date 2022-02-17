#ifndef INC_SincField_H
#define INC_SincField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "Properties/FieldProperties.h"

#include <cmath>
#include "boost/math/special_functions/sinc.hpp"
#include <MyCEL/math/math_constants.h>
#include "SDEFramework/GeneralField.h"


template<typename precision>
class SincField : public GeneralField<SincField<precision>>
{
public:
    using ThisClass = SincField<precision>;
    using Precision = precision;
    using Base = GeneralField<ThisClass>;
    using Traits = typename Base::Traits;
    using FieldProperties = typename Traits::FieldProperties;
    using FieldVector = typename Traits::FieldVector;
    using FieldParams = typename Traits::FieldParameters;

private:
    //const FieldProperties _params;
    FieldParams params;

    const precision mHalfPeriode;
    const precision sinc_a;
public:
    ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SincField(const typename Traits::FieldParameters& input)
        :params(input),mHalfPeriode(input.Periode *0.5),sinc_a(11.0/(mHalfPeriode))
    {};
    SincField(const FieldProperties& pars):SincField(pars.template getFieldParameters<Traits::Field_type>())
    {};

    inline FieldVector getField(const precision& time) const
    {
        const auto position = time < 0 ? std::fmod(time + params.Periode, params.Periode) - mHalfPeriode : std::fmod(time, params.Periode)-mHalfPeriode;
        return params.Amplitude * std::pow(boost::math::sinc_pi(math::constants::pi<precision>*sinc_a*position),2)+params.OffsetField;
    };
};

template<typename precision>
class FieldTraits<SincField<precision>>
{
public:
    using Precision = precision;
    using FieldProperties = Properties::FieldProperties<Precision>;
    using FieldVector = Eigen::Matrix<Precision, 3, 1>;
    using FieldVectorStdAllocator =  std::allocator<FieldVector>;
    using FieldParameters = ::Properties::Fields::Sinc<Precision>;
    static constexpr auto Field_type = ::Properties::IField::Field_Sinc;
};
#endif	// INC_Sinc_H
// end of Sinc.h
