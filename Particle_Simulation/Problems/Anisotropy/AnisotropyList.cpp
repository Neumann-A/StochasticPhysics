///-------------------------------------------------------------------------------------------------
// file:	Problems\Anisotropy\AnisotropyList.cpp
//
// summary:	Implements the anisotropy list class
///-------------------------------------------------------------------------------------------------
#include "AnisotropyList.h"
#include <string>

namespace Properties
{
    std::string to_string(const IAnisotropy& aniso)
    {
        return std::string{IAnisotropyMap[aniso]};
    }

    template<>
    IAnisotropy from_string<IAnisotropy>(const std::string& AnisoString)
    {
        return IAnisotropyMap[AnisoString];
    }
    template<>
    IAnisotropy from_string<IAnisotropy>(std::string_view AnisoString, IAnisotropy& value)
    {
        return (value = IAnisotropyMap[AnisoString]);
    }
}
