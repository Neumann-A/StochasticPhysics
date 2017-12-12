///---------------------------------------------------------------------------------------------------
// file:		GeneralField.h
//
// summary: 	Declares the general field class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander Neumann
// date: 23.08.2015
#pragma once
#ifndef INC_GeneralField_H
#define INC_GeneralField_H
///---------------------------------------------------------------------------------------------------

#include <type_traits>
#include "basics/BasicMacros.h"

///-------------------------------------------------------------------------------------------------
/// <summary>	A field traits. </summary>
///
/// <typeparam name="T">	Type of Field  </typeparam>
///-------------------------------------------------------------------------------------------------
template<typename T>
class FieldTraits
{
	//static_assert(std::negation<std::is_same<T, GeneralField<T>::BaseField>>::value, "Must define FieldTraits for this type!");
};

//TODO: Check for expected FieldTraits;

///-------------------------------------------------------------------------------------------------
/// <summary>	CRTP Field class.  </summary>
///
/// <typeparam name="field">	Type of the field. </typeparam>
///-------------------------------------------------------------------------------------------------
template <typename field>
class GeneralField
{	
public:
	using Derived = field;
	using BaseField = field;
	using Traits = FieldTraits<field>;
	using Precision = typename Traits::Precision;
	using FieldVector = typename Traits::FieldVector;

	static_assert(std::is_floating_point_v<Precision>, "GeneralField can only be used with floating point types!");

protected:
	constexpr BASIC_ALWAYS_INLINE GeneralField() = default;

private:
	BASIC_ALWAYS_INLINE Derived& self() BASIC_NOEXCEPT
	{
		return *static_cast<Derived * const>(this);
	};

public:	
	BASIC_ALWAYS_INLINE auto getField(const Precision& time)
	{
		return self().getField(time);
	};
};


#endif	// INC_GeneralField_H
// end of GeneralField.h
///---------------------------------------------------------------------------------------------------
