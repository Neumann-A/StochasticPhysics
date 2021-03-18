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

#include "SDEFramework/GeneralField.h"

#include "Properties/FieldProperties.h"

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
	using FieldParams = typename Traits::FieldParameters;

private:
	FieldParams params;

public:
	////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ConstantField(const typename Traits::FieldParameters& input): params(input)
	{};
	ConstantField(const FieldProperties& pars) :ConstantField(pars.template getFieldParameters<Traits::Field_type>())
	{};

	inline const auto& getField(const precision) const noexcept { return params.OffsetField; };

};

template<typename precision>
class FieldTraits<ConstantField<precision>>
{
public:
	using Precision = precision;
	using FieldProperties = Properties::FieldProperties<Precision>;
	using FieldVector = Eigen::Matrix<Precision, 3, 1>;
	using FieldVectorStdAllocator =  std::allocator<FieldVector>;
	using FieldParameters = ::Properties::Fields::Constant<Precision>;
	static constexpr auto Field_type = ::Properties::IField::Field_Constant;
};


#endif	// INC_ConstantField_H
// end of ConstantField.h
///---------------------------------------------------------------------------------------------------
