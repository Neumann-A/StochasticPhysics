#ifndef INC_RectangularField_H
#define INC_RectangularField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "Properties/FieldProperties.h"
#include <cmath>

#include "SDEFramework/GeneralField.h"

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

private:
    //const FieldProperties _params;
    const precision mPeriode;
    const precision mHalfPeriode;
    const precision mTimeoffset;
    const FieldVector mAmplitude;
    const FieldVector mOffset;
    const precision tau;
    const precision _tau = tau == 0 ? std::numeric_limits<precision>::max() : 1.0 / tau;
    
    const bool alternating;

    const FieldVector newmAmlitude = alternating == true ? 2 * mAmplitude : mAmplitude;
    const FieldVector maxField = newmAmlitude * (-expm1(-mHalfPeriode * _tau));

    const precision newTimeoffset = alternating == true ? mTimeoffset - std::log(1 - maxField.norm() / (2* newmAmlitude.norm())) * tau : mTimeoffset;
    const FieldVector newmoffset = alternating == true ? newmAmlitude * (expm1(-newTimeoffset * _tau)) + mOffset : mOffset;
    
    

public:
    ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    constexpr RectangularField(const typename Traits::Field_parameters& input, const FieldProperties& params)
        : mPeriode(input._Periodes.at(0)), mHalfPeriode(mPeriode / 2.0),mTimeoffset(input._PhasesTimeOffsets.at(0)),
        mAmplitude(params.getAmplitudes().at(1)), mOffset(params.getAmplitudes().at(0)), tau(input._Tau), alternating(input._alternating) {};

    constexpr RectangularField(const FieldProperties &params) :RectangularField(params.template getFieldParameters<Traits::Field_type>(),params)
    {};

    inline FieldVector getField(const precision& time) const
    {
        FieldVector Result;
        const auto newtime{ time + newTimeoffset };
        const auto position = newtime < 0 ? std::fmod(newtime+mPeriode, mPeriode) : std::fmod(newtime, mPeriode);

        if (position <= mHalfPeriode)
        { 
            Result = newmAmlitude *(-expm1(-position * _tau)) + newmoffset;
            //maxField = Result - mOffset;
        }
        else
        {
            Result = newmAmlitude *exp(-(position - mHalfPeriode) * _tau) + newmoffset -(newmAmlitude -maxField);
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
    using Field_parameters = ::Properties::Fields::Rectangular<Precision>;
    using Ftype = Properties::IField;
    static constexpr Ftype Field_type = Ftype::Field_Rectangular;
};
#endif	// INC_RectangularField_H
// end of RectangularField.h
