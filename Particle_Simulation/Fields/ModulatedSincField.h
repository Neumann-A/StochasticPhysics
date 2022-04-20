#ifndef INC_ModulatedSincField_H
#define INC_ModulatedSincField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "Properties/FieldProperties.h"

#include <cmath>
#include "boost/math/special_functions/sinc.hpp"
#include <MyCEL/math/math_constants.h>
#include "SDEFramework/GeneralField.h"


template<typename precision>
class ModulatedSincField : public GeneralField<ModulatedSincField<precision>>
{
public:
    using ThisClass = ModulatedSincField<precision>;
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
    const precision angularModFreq;
    //const bool alternating;
public:
    ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ModulatedSincField(const typename Traits::FieldParameters& input)
        :params(input),mHalfPeriode(input.Periode *0.5),sinc_a(input.Factor/(mHalfPeriode)),angularModFreq(input.ModulationFrequency)
    {};
    ModulatedSincField(const FieldProperties& pars):ModulatedSincField(pars.template getFieldParameters<Traits::Field_type>())
    {};

    inline FieldVector getField(const precision& time) const
    {
        const auto position = time < 0 ? std::fmod(time + params.Periode, params.Periode) - mHalfPeriode : std::fmod(time, params.Periode)-mHalfPeriode;
        return params.Amplitude * boost::math::sinc_pi(math::constants::pi<precision>*(sinc_a)*position)*std::sin(angularModFreq*time)+params.OffsetField;
    };
};

template<typename precision>
class FieldTraits<ModulatedSincField<precision>>
{
public:
    using Precision = precision;
    using FieldProperties = Properties::FieldProperties<Precision>;
    using FieldVector = Eigen::Matrix<Precision, 3, 1>;
    using FieldVectorStdAllocator =  std::allocator<FieldVector>;
    using FieldParameters = ::Properties::Fields::ModulatedSinc<Precision>;
    static constexpr auto Field_type = ::Properties::IField::Field_Modsinc;
};
#endif	// INC_Sinc_H
// end of Sinc.h
