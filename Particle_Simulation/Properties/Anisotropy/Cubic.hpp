#pragma once

#ifndef INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct Cubic : General<prec> {

    };
    template<typename Precision, typename Archive>
    void serialize(Cubic<Precision>& val, Archive& ar)
    {

    }
}

#endif //INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP
