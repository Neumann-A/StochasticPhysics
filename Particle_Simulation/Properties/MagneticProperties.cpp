///-------------------------------------------------------------------------------------------------
// file:	MagneticProperties.cpp
//
// summary:	Implements the magnetic properties class
///-------------------------------------------------------------------------------------------------
#include "MagneticProperties.h"

namespace Properties
{
	std::string to_string(const IAnisotropy& field)
	{
		return IAnisotropyMap.at(field);
	};

	template<>
	IAnisotropy from_string<IAnisotropy>(const std::string &String)
	{
		for (const auto& it : IAnisotropyMap)
			if (it.second == String)
				return it.first;

		throw std::runtime_error{ (std::string{ "MagneticProperties: Type of Anisotropy unknown! Requested Type: " } +String) };
	};
}