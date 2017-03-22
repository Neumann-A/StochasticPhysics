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

#include "Properties/MagneticProperties.h"

namespace Selectors
{
	using namespace Properties;

	template <IAnisotropy Enum>
	class AnisotropySelector;

	template<>
	class AnisotropySelector<IAnisotropy::Anisotropy_uniaxial>
	{
		template<typename prec>
		using type = UniaxialAnisotropy<prec>;
	};
}

#endif	// INC_AnisotropySelector_H
// end of AnisotropySelector.h
///---------------------------------------------------------------------------------------------------
