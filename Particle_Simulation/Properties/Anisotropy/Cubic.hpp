#pragma once

#ifndef INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct Cubic : General<prec> {
        prec K_cubic{0.0};

        using ThisClass = Cubic<prec>;
        using Distribution = Cubic_Distribution<prec>;

        ThisClass& operator+=(const ThisClass& rhs)
        {
            K_cubic += rhs.K_cubic;
            return *this;
        }
        template<typename Number>
        std::enable_if_t<std::is_arithmetic<Number>::value, ThisClass&> operator/=(const Number& scalar)
        {
            K_cubic /= (double)scalar;
            return *this;
        }
    };


    template<typename Precision, typename Archive>
    void serialize(Cubic<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("K_cubic", val.K_cubic));
    }
}

#endif //INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP
