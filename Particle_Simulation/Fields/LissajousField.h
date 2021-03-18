///---------------------------------------------------------------------------------------------------
// file:		LissajousField.h
//
// summary: 	Declares the lissajous field class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 02.02.2017

#ifndef INC_LissajousField_H
#define INC_LissajousField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <cmath>
#include <vector>
#include <exception>

#include "Properties/FieldProperties.h"

#include "SDEFramework/GeneralField.h"

#include  <MyCEL/math/math_constants.h>

//namespace
//{
//	constexpr auto m_pi = 3.14159265358979323846;
//}

template <typename precision>
class LissajousField : public GeneralField<LissajousField<precision>>
{
public:
    using ThisClass = LissajousField<precision>;
    using Precision = precision;
    using Base = GeneralField<ThisClass>;
    using Traits = typename Base::Traits;
    using FieldProperties = typename Traits::FieldProperties;
    using FieldVector = typename Traits::FieldVector;
    using FieldParams = typename Traits::FieldParameters;

private:

    FieldParams	params;

    const FieldVector AngFreq= createAngFreq(params);
    const FieldVector Phase= createPhase(params);

    FieldVector createAngFreq(const typename Traits::FieldParameters &input)
    {
        const auto& FreqVec = input.Frequencies;

        if (FreqVec.size() != 3)
        {
            throw std::runtime_error{ "LissajousField: Cannot create Frequency vector due to unsupported size of parameters in Fieldproperties" };
        }

        FieldVector tmp;
        tmp(0) = math::constants::two_pi<precision> *FreqVec(0);
        tmp(1) = math::constants::two_pi<precision> *FreqVec(1);
        tmp(2) = math::constants::two_pi<precision> *FreqVec(2);
        return tmp;
    }

    FieldVector createPhase(const typename Traits::FieldParameters & input)
    {
        const auto& PhaseVec = input.PhasesTimeOffsets;
        FieldVector tmp;
        if (PhaseVec.size() == 1)
        {
            tmp(0) = PhaseVec(0);
            tmp(1) = PhaseVec(0);
            tmp(2) = PhaseVec(0);
        }
        else if (PhaseVec.size() == 3)
        {
            tmp(0) = PhaseVec(0);
            tmp(1) = PhaseVec(1);
            tmp(2) = PhaseVec(2);
        }
        else
        {
            throw std::runtime_error{ "LissajousField: Cannot create Phase vector due to unsupported size of parameters in Fieldproperties" };
        }

        return tmp;
    }

protected:
public:
    constexpr LissajousField(const typename Traits::FieldParameters &input)
        : params(input)
    {};
    constexpr LissajousField(const FieldProperties &pars):LissajousField(pars.template getFieldParameters<Traits::Field_type>())
    {};

    inline FieldVector getField(const Precision& time) const noexcept
    {
        return (params.OffsetField + params.Amplitudes.cwiseProduct((AngFreq*time+Phase).array().sin().matrix()));
    }
};


template<typename prec>
class FieldTraits<LissajousField<prec>>
{
public:
    using Precision = prec;
    using FieldProperties = Properties::FieldProperties<Precision>;
    using FieldVector = Eigen::Matrix<Precision, 3, 1>;
    using FieldVectorStdAllocator =  std::allocator<FieldVector>;
    using FieldParameters = ::Properties::Fields::Lissajous<Precision>;
    static constexpr auto Field_type = ::Properties::IField::Field_Lissajous;
};

#endif	// INC_LissajousField_H
// end of LissajousField.h
///---------------------------------------------------------------------------------------------------
