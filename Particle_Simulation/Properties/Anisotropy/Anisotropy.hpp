#pragma once

#ifndef INC_STOPHYS_ANISOTROPY_HPP
#define INC_STOPHYS_ANISOTROPY_HPP

namespace Properties::Anisotropy
{
    template<typename prec>
    struct General {
        static inline std::string getSectionName() { return std::string{ "Anisotropy_Properties" }; };
    };
}

#endif //INC_STOPHYS_ANISOTROPY_HPP
