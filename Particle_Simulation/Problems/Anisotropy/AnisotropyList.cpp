///-------------------------------------------------------------------------------------------------
// file:	Problems\Anisotropy\AnisotropyList.cpp
//
// summary:	Implements the anisotropy list class
///-------------------------------------------------------------------------------------------------
#include "AnisotropyList.h"

namespace Properties
{
	std::string to_string(const IAnisotropy& field)
	{
		return IAnisotropyMap.at(field);
	};

	template<>
	IAnisotropy from_string<IAnisotropy>(const std::string &AnisoString)
	{
		for (const auto& it : IAnisotropyMap)
			if (it.second == AnisoString)
				return it.first;

		throw std::runtime_error{ (std::string{ "Type of Anisotropy unknown! Requested Type: " } +AnisoString) };
	};
}