///---------------------------------------------------------------------------------------------------
// file:		Problems\Anisotropy\AnisotropyList.h
//
// summary: 	Declares the anisotropy list class
//
// Copyright (c) 2018 Alexander Neumann.
//
// author: Alexander
// date: 14.06.2018

#ifndef INC_AnisotropyList_H
#define INC_AnisotropyList_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <map>
#include <string>

//Forward Declare all Anisotropies
namespace Problems::Anisotropy
{
	template <typename prec>
	class UniaxialAnisotropy;
	template <typename prec>
	class CubicAnisotropy;

	template <typename anisotropy>
	struct AnisotropyTraits;
}

namespace Properties
{

	/// <summary>	Values that represent anisotropies. </summary>
	enum class IAnisotropy { Anisotropy_undefined, Anisotropy_uniaxial, Anisotropy_cubic};

	/// <summary>	Map used to change the IAnisotropy enum to a string and vice versa. </summary>
	extern const std::map<IAnisotropy, std::string> IAnisotropyMap;

	template<typename T>
	T from_string(const std::string&);

	///-------------------------------------------------------------------------------------------------
	/// <summary>	Gets the enum IAnisotropy from a string. </summary>
	///
	/// <param name="AnisoString">	The string to transform </param>
	///
	/// <returns>	An Enum representing the string  </returns>
	///-------------------------------------------------------------------------------------------------
	template<>
	IAnisotropy from_string<IAnisotropy>(const std::string &AnisoString);
	std::string to_string(const IAnisotropy& field);
}
#endif	// INC_AnisotropyList_H
// end of Problems\Anisotropy\AnisotropyList.h
