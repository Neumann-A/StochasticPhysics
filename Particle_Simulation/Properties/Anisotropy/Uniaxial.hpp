#pragma once

#ifndef INC_STOPHYS_UNIAXIALANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_UNIAXIALANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct Uniaxial : General<prec> {
        prec K_uniaxial{0.0};
    };


    template<typename Precision, typename Archive>
    void serialize(Uniaxial<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("K_uniaxial", val.K_uniaxial));
    }
}
#endif //INC_STOPHYS_UNIAXIALANISOTROPYPROPERTY_HPP
