#pragma once

#ifndef INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct Mixed : General<prec> {

        using ThisClass = Mixed<prec>;
        using Distribution = Mixed_Distribution<prec>;

        ThisClass& operator+=(const ThisClass& rhs)
        {
            return *this;
        }
        template<typename Number>
        std::enable_if_t<std::is_arithmetic<Number>::value, ThisClass&> operator/=(const Number& scalar)
        {
            return *this;
        }
    };
    template<typename Precision, typename Archive>
    void serialize(Mixed<Precision>& val, Archive& ar)
    {

    }

}
#endif //INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP
