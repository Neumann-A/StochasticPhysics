///---------------------------------------------------------------------------------------------------
// file:		ZeroField.h
//
// summary: 	Declares the zero field class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 06.06.2017

#ifndef INC_ZeroField_H
#define INC_ZeroField_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "SDEFramework/GeneralField.h"

#include "Properties/FieldProperties.h"

template<typename precision>
class ZeroField : public GeneralField<ZeroField<precision>>
{
public:
	using ThisClass = ZeroField<precision>;
	using Precision = precision;
	using Base = GeneralField<ThisClass>;
	using Traits = typename Base::Traits;
	using FieldProperties = typename Traits::FieldProperties;
	using FieldVector = typename Traits::FieldVector;

private:

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ZeroField(const FieldProperties &params) {};

	inline auto getField(const precision) const { return FieldVector::Zero(); };

};

template<typename precision>
class FieldTraits<ZeroField<precision>>
{
public:
	using Precision = precision;
	using FieldProperties = Properties::FieldProperties<Precision>;
	using FieldVector = Eigen::Matrix<Precision, 3, 1>;
	using FieldVectorStdAllocator = Eigen::aligned_allocator<FieldVector>;
};




#endif	// INC_ZeroField_H
// end of ZeroField.h
///---------------------------------------------------------------------------------------------------
