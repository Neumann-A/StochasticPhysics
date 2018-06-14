///---------------------------------------------------------------------------------------------------
// file:		AnisotropySelector.h
//
// summary: 	Declares the anisotropy selector class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 11.07.2016

#ifndef INC_AnisotropySelector_H
#define INC_AnisotropySelector_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include "Problems/Anisotropy/AnisotropyList.h"

namespace Selectors
{
	using namespace Properties;

	template <IAnisotropy Anisotropy>
	class AnisotropyTypeSelector;

	template <>
	class AnisotropyTypeSelector<IAnisotropy::Anisotropy_uniaxial>
	{
	public:
		template<typename prec>
		using type = typename Problems::Anisotropy::UniaxialAnisotropy<prec>;
		template<typename prec>
		using traits = typename Problems::Anisotropy::AnisotropyTraits<type<prec>>;
	};
	template <>
	class AnisotropyTypeSelector<IAnisotropy::Anisotropy_cubic>
	{
	public:
		template<typename prec>
		using type = typename Problems::Anisotropy::CubicAnisotropy<prec>;
		template<typename prec>
		using traits = typename Problems::Anisotropy::AnisotropyTraits<type<prec>>;
	};


}

#endif	// INC_AnisotropySelector_H
// end of AnisotropySelector.h
///---------------------------------------------------------------------------------------------------
