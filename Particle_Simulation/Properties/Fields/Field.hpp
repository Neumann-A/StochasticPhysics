#pragma once

#ifndef INC_STOPHYS_FIELD_HPP
#define INC_STOPHYS_FIELD_HPP

#include <string>
#include <MyCEL/basics/BasicIncludes.h>
#include <SerAr/Core/NamedValue.h>

namespace Properties::Fields
{
    template<typename prec>
    struct General {
        static inline std::string getSectionName() { return "Field_Parameters"; };
    };
}

#endif //INC_STOPHYS_ANISOTROPY_HPP
