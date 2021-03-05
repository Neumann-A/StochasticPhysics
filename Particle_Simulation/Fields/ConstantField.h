///---------------------------------------------------------------------------------------------------
// file:		ConstantField.h
//
// summary: 	Declares the constant field class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 07.06.2017

#ifndef INC_ConstantField_H
#define INC_ConstantField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "Properties/FieldProperties.h"

#include "SDEFramework/GeneralField.h"


template<typename precision>
class ConstantField : public GeneralField<ConstantField<precision>>
{
public:
	using ThisClass = ConstantField<precision>;
	using Precision = precision;
	using Base = GeneralField<ThisClass>;
	using Traits = typename Base::Traits;
	using FieldProperties = typename Traits::FieldProperties;
	using FieldVector = typename Traits::FieldVector;

private:
	const FieldVector _field;

public:
	////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ConstantField(const FieldProperties &params) : _field(*params.getAmplitudes().begin()) {};

	inline const auto& getField(const precision) const noexcept { return _field; };

};

template<typename precision>
class FieldTraits<ConstantField<precision>>
{
public:
	using Precision = precision;
	using FieldProperties = Properties::FieldProperties<Precision>;
	using FieldVector = Eigen::Matrix<Precision, 3, 1>;
	using FieldVectorStdAllocator =  std::allocator<FieldVector>;
	using Field_parameters = ::Properties::Fields::Constant<Precision>;
	using Ftype = Properties::IField;
	static constexpr Ftype Field_type = Ftype::Field_Constant;
};


#endif	// INC_ConstantField_H
// end of ConstantField.h
///---------------------------------------------------------------------------------------------------
