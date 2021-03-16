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

    const precision     mHalfPeriode=params.Periodes/2.0;
    const precision     _Tau = std::abs(params.Tau) <= std::numeric_limits<precision>::min() 0 ? std::numeric_limits<precision>::max() : 1.0 / params.Tau;

    const FieldVector   newmAmlitude = params.Alternating == true ? 2 * params.Amplitudes : params.Amplitudes;
    const FieldVector   maxField = newmAmlitude * (-expm1(-mHalfPeriode * _Tau));

    const precision     newTimeoffset = params.Alternating == true ? params.PhasesTimeOffsets - std::log(1 - maxField.norm() / (2* newmAmlitude.norm())) * params.Tau : params.PhasesTimeOffsets;
    const FieldVector   newmoffset = params.Alternating == true ? newmAmlitude * (expm1(-newTimeoffset * _Tau)) + params.OffsetField : params.OffsetField;
    
    

public:
    ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    constexpr RectangularField(const typename Traits::FieldParameters& input)
        :params(input)
        {};

    constexpr RectangularField(const FieldProperties &params) :RectangularField(params.template getFieldParameters<Traits::Field_type>())
    {};

    inline FieldVector getField(const precision& time) const
    {
        FieldVector Result;
        const auto newtime{ time + newTimeoffset };
        const auto position = newtime < 0 ? std::fmod(newtime+params.Periodes, params.Periodes) : std::fmod(newtime, params.Periodes);

        if (position <= mHalfPeriode)
        { 
            Result = newmAmlitude *(-expm1(-position * _Tau)) + newmoffset;
            //maxField = Result - mOffset;
        }
        else
        {
            Result = newmAmlitude *exp(-(position - mHalfPeriode) * _Tau) + newmoffset -(newmAmlitude -maxField);
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
