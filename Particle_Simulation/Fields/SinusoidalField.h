///---------------------------------------------------------------------------------------------------
// file:		Fields\SinusoidalField.h
//
// summary: 	Declares the sinusoidal field class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Neumann
// date: 23.08.2015
#pragma once
#ifndef INC_SinusoidalField_H
#define INC_SinusoidalField_H
///---------------------------------------------------------------------------------------------------


#include "Properties/FieldProperties.h"
#include "SDEFramework/GeneralField.h"


template<typename precision>
class SinusoidalField :	public GeneralField<SinusoidalField<precision>>
{
public:
    using ThisClass = SinusoidalField<precision>;
    using Precision = precision;
    using Base = GeneralField<ThisClass>;
    using Traits = typename Base::Traits;
    using FieldProperties = typename Traits::FieldProperties;
    using FieldVector = typename Traits::FieldVector;
    using FieldParams = typename Traits::FieldParameters;

private:
    FieldParams params;

    const precision _angularfrequency;
    
public:
    SinusoidalField(const typename Traits::FieldParameters &input)
        :params(input),_angularfrequency(math::constants::two_pi<precision>* input.Frequencies)
    {
    }
    SinusoidalField(const FieldProperties& pars) : SinusoidalField(pars.template getFieldParameters<Traits::Field_type>())
    {
    }

    //Getter for the field Value; actual function is defined in the constructor
    FieldVector getField(const precision time)  const
    {
        const precision sinwt = std::sin(_angularfrequency * time + params.PhasesTimeOffsets);
        return (params.Amplitudes * sinwt + params.OffsetField).eval();
    }


};

template<typename precision>
class FieldTraits<SinusoidalField<precision>>
{
public:
    using Precision = precision;
    using FieldProperties = Properties::FieldProperties<Precision>;
    using FieldVector = Eigen::Matrix<Precision, 3, 1>;
    using FieldVectorStdAllocator =  std::allocator<FieldVector>;
    using FieldParameters = ::Properties::Fields::Sinusoidal<Precision>;
    static constexpr auto Field_type = ::Properties::IField::Field_Sinusoidal;
};


#endif	// INC_SinusoidalField_H
// end of Fields\SinusoidalField.h
///---------------------------------------------------------------------------------------------------
