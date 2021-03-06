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

#include <cmath>

#include "SDEFramework/GeneralField.h"

#include "Properties/FieldProperties.h"

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

private:
	//const FieldProperties _params;
	const precision mPeriode;
	const precision mHalfPeriode;
	const precision mTimeoffset;
	const FieldVector mAmplitude;
	const FieldVector mOffset;

public:
	////EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	TriangularField(const FieldProperties &params) 
		: mPeriode(params.getPeriodes().at(0)), mHalfPeriode(mPeriode/2.0),
		mTimeoffset(params.getTimeOffsets().at(0)),
		mAmplitude(params.getAmplitudes().at(1)), mOffset(params.getAmplitudes().at(0))
	{

	};

	inline FieldVector getField(const precision& time) const
	{
		FieldVector Result;
		const auto newtime{ time + mTimeoffset };
		const auto position = newtime < 0 ? mPeriode + std::fmod(newtime, mPeriode) : std::fmod(newtime, mPeriode);

		if (position > mHalfPeriode)
		{ 
			//We are moving down
			Result = (2.0*position / mHalfPeriode - 1.0)*mAmplitude + mOffset;
		}
		else
		{
			//We are moving up
			Result = (-2.0*position / mHalfPeriode + 3.0)*mAmplitude + mOffset;
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
};


#endif	// INC_TriangularField_H
// end of TriangularField.h
