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

    // set sinc a to approximatly (n+1/2)-1/((n+1/2)*pi^2) for a continous transition between to periods at the n-th extremum within the sinc-function
    const precision sinc_a;
    //const bool alternating;
public:
    ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SincField(const typename Traits::FieldParameters& input)
        :params(input),mHalfPeriode(input.Periode *0.5),sinc_a(input.Factor/(mHalfPeriode))
    {};
    SincField(const FieldProperties& pars):SincField(pars.template getFieldParameters<Traits::Field_type>())
    {};

    inline FieldVector getField(const precision& time) const
    {
        const auto position = time < 0 ? std::fmod(time + params.Periode, params.Periode) - mHalfPeriode : std::fmod(time, params.Periode)-mHalfPeriode;
        return params.Amplitude * boost::math::sinc_pi(math::constants::pi<precision>*(sinc_a)*position)+params.OffsetField;
    };

    // Alternating sinc
    //inline FieldVector getField(const precision& time) const
    //{
    //    const auto position = time < 0 ? std::fmod(time + params.Periode, params.Periode*0.5) - mHalfPeriode : std::fmod(time, params.Periode*0.5)-mHalfPeriode*0.5;
    //    const auto position2 = time < 0 ? std::fmod(time + params.Periode, params.Periode) - mHalfPeriode : std::fmod(time, params.Periode)-mHalfPeriode;
    //    if(position2<=0.0)
    //        return params.Amplitude * boost::math::sinc_pi(math::constants::pi<precision>*(sinc_a)*position)+params.OffsetField;
    //    else
    //        return -params.Amplitude * boost::math::sinc_pi(math::constants::pi<precision>*(sinc_a)*position)+params.OffsetField;
    //};

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
