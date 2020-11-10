#pragma once

#ifndef INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct Mixed : General<prec> {

    };
    template<typename Precision, typename Archive>
    void serialize(Mixed<Precision>& val, Archive& ar)
    {

    }
}
#endif //INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP
