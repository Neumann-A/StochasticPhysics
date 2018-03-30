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

#include  "math/math_constants.h"

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

private:

	const FieldVector OffsetField;
	const FieldVector Field;
	const FieldVector AngFreq;
	const FieldVector Phase;

	FieldVector createAngFreq(const FieldProperties &params)
	{
		const auto& FreqVec = params.getFrequencies();

		if (FreqVec.size() != 3)
		{
			throw std::runtime_error{ "LissajousField: Cannot create Frequency vector due to unsupported size of parameters in Fieldproperties" };
		}

		FieldVector tmp;
		tmp(0) = math::constants::two_pi<precision> * FreqVec.at(0);
		tmp(1) = math::constants::two_pi<precision> * FreqVec.at(1);
		tmp(2) = math::constants::two_pi<precision> * FreqVec.at(2);
		return tmp;
	}

	FieldVector createPhase(const FieldProperties &params)
	{
		const auto& PhaseVec = params.getPhases();
		FieldVector tmp;
		if (PhaseVec.size() == 1)
		{
			tmp(0) = PhaseVec.at(0);
			tmp(1) = PhaseVec.at(0);
			tmp(2) = PhaseVec.at(0);
		}
		else if (PhaseVec.size() == 3)
		{
			tmp(0) = PhaseVec.at(0);
			tmp(1) = PhaseVec.at(1);
			tmp(2) = PhaseVec.at(2);
		}
		else
		{
			throw std::runtime_error{ "LissajousField: Cannot create Phase vector due to unsupported size of parameters in Fieldproperties" };
		}

		return tmp;
	}

protected:
public:
	constexpr LissajousField(const FieldProperties &params)
		: OffsetField(params.getAmplitudes().at(0)), Field(params.getAmplitudes().at(1)), AngFreq(createAngFreq(params)), Phase(createPhase(params))
	{};

	inline FieldVector getField(const Precision& time) const noexcept
	{
		//FieldVector tmp;
		//tmp(0) = Field(0)*sin(AngFreq(0)*time);
		//tmp(1) = Field(1)*sin(AngFreq(1)*time);
		//tmp(2) = Field(2)*sin(AngFreq(2)*time);
		return (OffsetField + Field.cwiseProduct((AngFreq*time).array().sin().matrix()));
	}
};


template<typename prec>
class FieldTraits<LissajousField<prec>>
{
public:
	using Precision = prec;
	using FieldProperties = Properties::FieldProperties<Precision>;
	using FieldVector = Eigen::Matrix<Precision, 3, 1>;
	using FieldVectorStdAllocator = Eigen::aligned_allocator<FieldVector>;
};

#endif	// INC_LissajousField_H
// end of LissajousField.h
///---------------------------------------------------------------------------------------------------
