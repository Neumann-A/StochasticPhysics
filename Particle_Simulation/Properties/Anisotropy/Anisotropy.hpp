#pragma once

#ifndef INC_STOPHYS_ANISOTROPY_HPP
#define INC_STOPHYS_ANISOTROPY_HPP

#include <exception

#include <MyCEL/math/DistributionHelper.h>
#include <SerAr/Core/NamedValue.h>

namespace Properties::Anisotropy
{
    template<typename prec>
    struct General {
        static inline std::string getSectionName() { return std::string{ "Anisotropy_Properties" }; };
    };

    template<typename prec>
    struct Distribution
    {
        virtual ~Distribution() = default;
        static inline std::string getSectionName() { return std::string{ "Anisotropy_Distribution" }; };
        
    };
}

#endif //INC_STOPHYS_ANISOTROPY_HPP
