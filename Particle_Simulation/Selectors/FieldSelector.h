///---------------------------------------------------------------------------------------------------
// file:		FieldSelector.h
//
// summary: 	Declares the field selector class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.07.2016

#ifndef INC_FieldSelector_H
#define INC_FieldSelector_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "Selectors/BasicSelector.h"
#include "Properties/FieldProperties.h"

namespace Selectors
{
	using namespace Properties;

	template <IField field>
	class FieldSelector : public BasicSelector<FieldSelector<field>> {};

	template <>
	class FieldSelector<IField::Field_Sinusoidal> : public BasicSelector<FieldSelector<IField::Field_Sinusoidal>>
	{
	public:
		template<typename prec>
		using FieldType = SinusoidalField<prec>;

		template<typename prec>
		using Traits = FieldTraits<FieldType<prec>>;
		
		template<typename prec>
		using FieldParameters = typename Traits<prec>::FieldProperties;
	};

	template <>
	class FieldSelector<IField::Field_Lissajous> : public BasicSelector<FieldSelector<IField::Field_Lissajous>>
	{
	public:
		template<typename prec>
		using FieldType = SinusoidalField<prec>;

		template<typename prec>
		using Traits = FieldTraits<FieldType<prec>>;

		template<typename prec>
		using FieldParameters = typename Traits<prec>::FieldProperties;
	};

}

#endif	// INC_FieldSelector_H
// end of FieldSelector.h
///---------------------------------------------------------------------------------------------------
