#ifndef INC_RectangularField_H
#define INC_RectangularField_H
///---------------------------------------------------------------------------------------------------
#pragma once


#include <cmath>

#include "SDEFramework/GeneralField.h"
#include "Properties/FieldProperties.h"

template<typename precision>
class RectangularField : public GeneralField<RectangularField<precision>>
{
public:
    using ThisClass = RectangularField<precision>;
    using Precision = precision;
    using Base = GeneralField<ThisClass>;
    using Traits = typename Base::Traits;
    using FieldProperties = typename Traits::FieldProperties;
    using FieldVector = typename Traits::FieldVector;
    using FieldParams = typename Traits::FieldParameters;

private:
    FieldParams params;

    const precision     mHalfPeriode=params.Periode / 2.0;
    const precision     mTau = std::abs(params.Tau) <= std::numeric_limits<precision>::min() ? std::numeric_limits<precision>::max() : 1.0 / params.Tau;

    const FieldVector   newmAmlitude = params.Alternating == true ? 2 * params.Amplitude : params.Amplitude;
    const FieldVector   maxField = newmAmlitude * (-expm1(-mHalfPeriode * mTau));

    const precision     newTimeoffset = params.Alternating == true ? params.TimeOffset - std::log(1 - maxField.norm() / (2* newmAmlitude.norm())) * params.Tau : params.TimeOffset;
    const FieldVector   newmoffset = params.Alternating == true ? newmAmlitude * (expm1(-newTimeoffset * mTau)) + params.OffsetField : params.OffsetField;
    
    

public:
    ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    constexpr RectangularField(const typename Traits::FieldParameters& input)
        :params(input)
        {};

    constexpr RectangularField(const FieldProperties &pars) : RectangularField(pars.template getFieldParameters<Traits::Field_type>())
    {};

    inline FieldVector getField(const precision& time) const
    {
        FieldVector Result;
        const auto newtime{ time + newTimeoffset };
        const auto position = newtime < 0 ? std::fmod(newtime+params.Periode, params.Periode) : std::fmod(newtime, params.Periode);

        if (position <= mHalfPeriode)
        { 
            Result = newmAmlitude *(-expm1(-position * mTau)) + newmoffset;
            //maxField = Result - mOffset;
        }
        else
        {
            Result = newmAmlitude *exp(-(position - mHalfPeriode) * mTau) + newmoffset -(newmAmlitude -maxField);
        }
        return Result;
    };

};

template<typename precision>
class FieldTraits<RectangularField<precision>>
{
public:
    using Precision = precision;
    using FieldProperties = Properties::FieldProperties<Precision>;
    using FieldVector = Eigen::Matrix<Precision, 3, 1>;
    using FieldVectorStdAllocator =  std::allocator<FieldVector>;
    using FieldParameters = ::Properties::Fields::Rectangular<Precision>;
    static constexpr auto Field_type = ::Properties::IField::Field_Rectangular;
};
#endif	// INC_RectangularField_H
// end of RectangularField.h
