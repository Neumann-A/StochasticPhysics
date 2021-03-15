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

private:
	//const FieldProperties _params;
	//Properties::Fields::Sinusoidal<precision> Selective_params;
	const precision _angularfrequency;
	const precision _phase;
	const FieldVector _ampDirection;
	const FieldVector _offset;
	
public:
	////EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//math::constants::two_pi<precision>*
	//:Selective_params(params.template getFieldProperties<Properties::IField::Field_Sinusoidal>())
	SinusoidalField(const typename Traits::Field_parameters &input)
		:_angularfrequency(math::constants::two_pi<precision>* input._Frequencies.at(0)),
		_phase(input._PhasesTimeOffsets.at(0)), _ampDirection(input._Amplitudes.at(1)), _offset(input._Amplitudes.at(0))
	{
	}
	SinusoidalField(const FieldProperties& params):SinusoidalField(params.template getFieldParameters<Traits::Field_type>())
	{
	}

	//Getter for the field Value; actual function is defined in the constructor
	FieldVector getField(const precision time)  const
	{
		const precision sinwt = std::sin(_angularfrequency * time + _phase);
		return (_ampDirection * sinwt + _offset).eval();
	}


};

//#include "SinusoidalField.inl"

template<typename precision>
class FieldTraits<SinusoidalField<precision>>
{
public:
	using Precision = precision;
	using FieldProperties = Properties::FieldProperties<Precision>;
	using FieldVector = Eigen::Matrix<Precision, 3, 1>;
	using FieldVectorStdAllocator =  std::allocator<FieldVector>;
	using Field_parameters = ::Properties::Fields::Sinusoidal<Precision>;
	using Ftype = Properties::IField;
	static constexpr Ftype Field_type = Ftype::Field_Sinusoidal;
};


#endif	// INC_SinusoidalField_H
// end of Fields\SinusoidalField.h
///---------------------------------------------------------------------------------------------------
