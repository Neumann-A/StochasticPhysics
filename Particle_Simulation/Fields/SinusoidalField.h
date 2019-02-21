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


#include "SDEFramework/GeneralField.h"

#include "Properties/FieldProperties.h"

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
	const precision _angularfrequency;
	const precision _phase;
	const FieldVector _ampDirection;
	const FieldVector _offset;
		
public:
	////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SinusoidalField(const FieldProperties &params);

	inline FieldVector getField(const precision time) const;

};

#include "SinusoidalField.inl"

template<typename precision>
class FieldTraits<SinusoidalField<precision>>
{
public:
	using Precision = precision;
	using FieldProperties = Properties::FieldProperties<Precision>;
	using FieldVector = Eigen::Matrix<Precision, 3, 1>;
	using FieldVectorStdAllocator =  std::allocator<FieldVector>;
};


#endif	// INC_SinusoidalField_H
// end of Fields\SinusoidalField.h
///---------------------------------------------------------------------------------------------------
