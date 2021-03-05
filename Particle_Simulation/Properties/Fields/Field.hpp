#pragma once

#ifndef INC_STOPHYS_FIELD_HPP
#define INC_STOPHYS_FIELD_HPP

#include <SerAr/Core/NamedValue.h>
#include "Fields/FieldList.h"

namespace Properties::Fields
{
    template<typename prec>
    struct General {

        template<IField value>
        static inline std::string getSectionName() { return Properties::to_string(value) + std::string{ "_Field_Parameters" }; };
    };

    static inline std::string buildSerilizationString(const char* name, const std::size_t& number)
    {
        return std::string{ name + BasicTools::toStringScientific(number) };
    }

    template<typename Archive, typename Container>
    static inline void serializeVector(Archive& ar, const char* sizevector, const char* vecname, Container& vector)
    {
        auto elements = vector.size();
        ar(Archives::createNamedValue(sizevector, elements));
        vector.resize(elements);

        std::size_t counter{ 0 };
        for (auto& it : vector)
        {
            ar(Archives::createNamedValue(buildSerilizationString(vecname, ++counter), it));
        }
    }
}

#endif //INC_STOPHYS_ANISOTROPY_HPP
