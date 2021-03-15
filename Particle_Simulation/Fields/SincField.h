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

private:
    //const FieldProperties _params;
    const precision mPeriode;
    const precision mHalfPeriode;
    const FieldVector mAmplitude;
    const FieldVector mOffset;
    const precision sinc_a;
public:
    ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SincField(const typename Traits::Field_parameters& input)
        : mPeriode(input._Periodes.at(0)),mHalfPeriode(mPeriode*0.5),
        mAmplitude(input._Amplitudes.at(1)), mOffset(input._Amplitudes.at(0)),sinc_a(11.0/(mHalfPeriode))
    {};
    SincField(const FieldProperties& params):SincField(params.template getFieldParameters<Traits::Field_type>())
    {};

    inline FieldVector getField(const precision& time) const
    {
        const auto position = time < 0 ? std::fmod(time + mPeriode, mPeriode) - mHalfPeriode : std::fmod(time, mPeriode)-mHalfPeriode;
        return mAmplitude * std::pow(boost::math::sinc_pi(math::constants::pi<precision>*sinc_a*position),2);
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
    using Field_parameters = ::Properties::Fields::Sinc<Precision>;
    using Ftype = Properties::IField;
    static constexpr Ftype Field_type = Ftype::Field_Sinc;
};
#endif	// INC_Sinc_H
// end of Sinc.h
