///---------------------------------------------------------------------------------------------------
// file:		TriangularField.h
//
// summary: 	Declares the triangular field class
//
// Copyright (c) 2018 Alexander Neumann.
//
// author: Alexander
// date: 29.03.2018

#ifndef INC_TriangularField_H
#define INC_TriangularField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "Properties\FieldProperties.h"

#include <cmath>

#include "SDEFramework/GeneralField.h"


template<typename precision>
class TriangularField : public GeneralField<TriangularField<precision>>
{
    // Triangular field starts at - Amplitude moves to + Amplitude in half the Periode and then back to - Amplitude
    // ToDo: Implement Asymetric Triangular Field;
public:
    using ThisClass = TriangularField<precision>;
    using Precision = precision;
    using Base = GeneralField<ThisClass>;
    using Traits = typename Base::Traits;
    using FieldProperties = typename Traits::FieldProperties;
    using FieldVector = typename Traits::FieldVector;
    using FieldParams = typename Traits::FieldParameters;

private:
    FieldParams params;

    const precision mHalfPeriode;

public:
    ////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TriangularField(const typename Traits::FieldParameters& input)
        :params(input), mHalfPeriode(input.Periodes/2.0)
    {};
    TriangularField(const FieldProperties& pars):TriangularField(pars.template getFieldParameters<Traits::Field_type>())
    {};

    inline FieldVector getField(const precision& time) const
    {
        FieldVector Result;
        const auto newtime{ time + params.PhasesTimeOffsets };
        const auto position = newtime < 0 ? params.Periodes + std::fmod(newtime, params.Periodes) : std::fmod(newtime, params.Periodes);

        if (position > mHalfPeriode)
        { 
            //We are moving down
            Result = (2.0*position / mHalfPeriode - 1.0)*params.Amplitudes + params.OffsetField;
        }
        else
        {
            //We are moving up
            Result = (-2.0*position / mHalfPeriode + 3.0) * params.Amplitudes + params.OffsetField;
        }
        return Result;
    };

};

template<typename precision>
class FieldTraits<TriangularField<precision>>
{
public:
    using Precision = precision;
    using FieldProperties = Properties::FieldProperties<Precision>;
    using FieldVector = Eigen::Matrix<Precision, 3, 1>;
    using FieldVectorStdAllocator =  std::allocator<FieldVector>;
    using FieldParameters = ::Properties::Fields::Triangular<Precision>;
    static constexpr auto Field_type = ::Properties::IField::Field_Triangular;
};


#endif	// INC_TriangularField_H
// end of TriangularField.h
